"""
ROS 2 Bridge

Connects HumanoidOS to the ROS 2 ecosystem.  When rclpy is available the
bridge creates a real ROS 2 node with publishers and subscribers.  If rclpy
is not installed it operates in *stub mode* — the API is identical but no
messages are sent or received, so the rest of the code compiles and runs
without ROS 2 installed.

Topics published (HumanoidOS → ROS 2):
    /humanoid/joint_states     sensor_msgs/JointState
    /humanoid/imu              sensor_msgs/Imu
    /humanoid/odom             nav_msgs/Odometry
    /humanoid/robot_state      std_msgs/String  (JSON)

Topics subscribed (ROS 2 → HumanoidOS):
    /cmd_vel                   geometry_msgs/Twist
    /humanoid/emergency_stop   std_msgs/Bool

Services (optional, when rclpy is available):
    /humanoid/go_home          std_srvs/Trigger
"""

import json
import time
import threading
import numpy as np
from typing import Optional, Dict, List, Callable
from dataclasses import dataclass, field
import logging

logger = logging.getLogger(__name__)

# Try to import rclpy; gracefully degrade if unavailable
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    Node = object   # placeholder so class definition works


@dataclass
class ROS2Config:
    """Configuration for the ROS 2 bridge"""
    node_name: str = "humanoid_os"
    namespace: str = "/humanoid"

    # Publish rates (Hz)
    joint_state_rate: float = 100.0
    imu_rate: float         = 200.0
    odom_rate: float        = 50.0
    robot_state_rate: float = 10.0

    # QoS
    use_reliable_qos: bool = False  # True = RELIABLE, False = BEST_EFFORT (lower latency)

    # Frame IDs
    base_frame: str   = "base_link"
    odom_frame: str   = "odom"
    imu_frame: str    = "imu_link"


class _StubNode:
    """No-op node used when rclpy is unavailable."""
    def __init__(self):
        self._cmd_vel = np.zeros(3)
        self._emergency_stop = False

    def create_publisher(self, *a, **kw): return _StubPub()
    def create_subscription(self, *a, **kw): return None
    def create_service(self, *a, **kw): return None
    def get_logger(self): return logger
    def destroy_node(self): pass


class _StubPub:
    def publish(self, *a, **kw): pass


class ROS2Bridge:
    """
    Bidirectional bridge between HumanoidOS and ROS 2.

    Usage (typical)::

        from ros2.ros2_bridge import ROS2Bridge, ROS2Config

        bridge = ROS2Bridge(ROS2Config())
        bridge.start()

        # In control loop:
        bridge.publish_joint_states(joint_states_dict)
        bridge.publish_imu(imu_data)
        bridge.publish_odometry(position, orientation, linear_vel, angular_vel)
        cmd = bridge.get_velocity_command()

        # On shutdown:
        bridge.stop()

    The bridge spins its own daemon thread so it does not block the
    1 kHz control loop.
    """

    def __init__(self, config: Optional[ROS2Config] = None):
        self.config = config or ROS2Config()
        self._available = _ROS2_AVAILABLE
        self._node = None
        self._spin_thread: Optional[threading.Thread] = None
        self._running = False

        # Inbound command cache (thread-safe via GIL for simple types)
        self._cmd_vel: np.ndarray = np.zeros(3)   # [vx, vy, omega]
        self._emergency_stop: bool = False
        self._cmd_lock = threading.Lock()

        # Outbound publishers (set in start())
        self._pub_joint_states = None
        self._pub_imu           = None
        self._pub_odom          = None
        self._pub_robot_state   = None

        # Publish rate trackers
        self._last_pub_times: Dict[str, float] = {}

        if not self._available:
            logger.warning(
                "rclpy not found — ROS2Bridge running in stub mode. "
                "Install ROS 2 and source its setup.bash to enable."
            )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        """Initialise ROS 2 node and start background spin thread."""
        if self._available:
            if not rclpy.ok():
                rclpy.init()
            self._node = self._build_node()
            self._running = True
            self._spin_thread = threading.Thread(
                target=self._spin_loop, daemon=True, name="ros2_spin"
            )
            self._spin_thread.start()
            logger.info(f"ROS2Bridge started: node={self.config.node_name}")
        else:
            self._node = _StubNode()
            logger.info("ROS2Bridge started in stub mode (no ROS 2)")

    def stop(self):
        """Stop the bridge and clean up ROS 2 resources."""
        self._running = False
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        if self._available and self._node:
            self._node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass
        logger.info("ROS2Bridge stopped")

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def publish_joint_states(
        self,
        joint_states: Dict,  # Dict[str, JointState] from HAL
        timestamp: Optional[float] = None,
    ):
        """
        Publish sensor_msgs/JointState from HAL joint state dict.

        Args:
            joint_states: Dict mapping joint name -> JointState (or tuple).
            timestamp:    Unix timestamp (defaults to now).
        """
        if not self._should_publish("joint_states", self.config.joint_state_rate):
            return

        if self._available and self._pub_joint_states:
            try:
                from sensor_msgs.msg import JointState as ROSJointState
                from builtin_interfaces.msg import Time

                msg = ROSJointState()
                msg.header.frame_id = self.config.base_frame
                msg.header.stamp    = self._ros_time(timestamp)
                msg.name     = list(joint_states.keys())
                msg.position = [float(s.position) for s in joint_states.values()]
                msg.velocity = [float(s.velocity) for s in joint_states.values()]
                msg.effort   = [float(s.torque)   for s in joint_states.values()]
                self._pub_joint_states.publish(msg)
            except Exception as exc:
                logger.debug(f"publish_joint_states error: {exc}")

    def publish_imu(self, imu_data, timestamp: Optional[float] = None):
        """Publish sensor_msgs/Imu from IMUData."""
        if not self._should_publish("imu", self.config.imu_rate):
            return

        if self._available and self._pub_imu:
            try:
                from sensor_msgs.msg import Imu
                import math

                msg = Imu()
                msg.header.frame_id = self.config.imu_frame
                msg.header.stamp    = self._ros_time(timestamp)

                # Convert euler to quaternion if orientation is [roll,pitch,yaw]
                ori = imu_data.orientation
                if len(ori) == 3:
                    q = self._euler_to_quat(*ori)
                else:
                    q = ori  # already quaternion

                msg.orientation.x = float(q[0])
                msg.orientation.y = float(q[1])
                msg.orientation.z = float(q[2])
                msg.orientation.w = float(q[3])

                av = imu_data.angular_velocity
                msg.angular_velocity.x = float(av[0])
                msg.angular_velocity.y = float(av[1])
                msg.angular_velocity.z = float(av[2])

                la = imu_data.linear_acceleration
                msg.linear_acceleration.x = float(la[0])
                msg.linear_acceleration.y = float(la[1])
                msg.linear_acceleration.z = float(la[2])

                self._pub_imu.publish(msg)
            except Exception as exc:
                logger.debug(f"publish_imu error: {exc}")

    def publish_odometry(
        self,
        position: np.ndarray,
        orientation_euler: np.ndarray,
        linear_velocity: np.ndarray,
        angular_velocity: np.ndarray,
        timestamp: Optional[float] = None,
    ):
        """Publish nav_msgs/Odometry from base link state."""
        if not self._should_publish("odom", self.config.odom_rate):
            return

        if self._available and self._pub_odom:
            try:
                from nav_msgs.msg import Odometry

                msg = Odometry()
                msg.header.stamp    = self._ros_time(timestamp)
                msg.header.frame_id = self.config.odom_frame
                msg.child_frame_id  = self.config.base_frame

                q = self._euler_to_quat(*orientation_euler)
                msg.pose.pose.position.x    = float(position[0])
                msg.pose.pose.position.y    = float(position[1])
                msg.pose.pose.position.z    = float(position[2])
                msg.pose.pose.orientation.x = float(q[0])
                msg.pose.pose.orientation.y = float(q[1])
                msg.pose.pose.orientation.z = float(q[2])
                msg.pose.pose.orientation.w = float(q[3])

                msg.twist.twist.linear.x  = float(linear_velocity[0])
                msg.twist.twist.linear.y  = float(linear_velocity[1])
                msg.twist.twist.linear.z  = float(linear_velocity[2])
                msg.twist.twist.angular.x = float(angular_velocity[0])
                msg.twist.twist.angular.y = float(angular_velocity[1])
                msg.twist.twist.angular.z = float(angular_velocity[2])

                self._pub_odom.publish(msg)
            except Exception as exc:
                logger.debug(f"publish_odometry error: {exc}")

    def publish_robot_state(self, state_dict: Dict, timestamp: Optional[float] = None):
        """
        Publish robot system state as a JSON string (std_msgs/String).

        Args:
            state_dict: Arbitrary dict with robot state information.
        """
        if not self._should_publish("robot_state", self.config.robot_state_rate):
            return

        if self._available and self._pub_robot_state:
            try:
                from std_msgs.msg import String
                msg = String()
                msg.data = json.dumps(state_dict)
                self._pub_robot_state.publish(msg)
            except Exception as exc:
                logger.debug(f"publish_robot_state error: {exc}")

    # ------------------------------------------------------------------
    # Subscribers / inbound data
    # ------------------------------------------------------------------

    def get_velocity_command(self) -> np.ndarray:
        """
        Return latest /cmd_vel command as [vx, vy, omega] in robot frame.

        Thread-safe. Returns zeros if no command has been received.
        """
        with self._cmd_lock:
            return self._cmd_vel.copy()

    def is_emergency_stop_requested(self) -> bool:
        """Return True if /humanoid/emergency_stop has been published."""
        with self._cmd_lock:
            return self._emergency_stop

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _build_node(self):
        """Create ROS 2 node with all publishers and subscribers."""
        cfg = self.config

        if self.config.use_reliable_qos:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
        else:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )

        node = rclpy.create_node(cfg.node_name, namespace=cfg.namespace)

        # Publishers
        try:
            from sensor_msgs.msg import JointState as ROSJointState, Imu
            from nav_msgs.msg import Odometry
            from std_msgs.msg import String, Bool

            self._pub_joint_states = node.create_publisher(ROSJointState, "/humanoid/joint_states", qos)
            self._pub_imu          = node.create_publisher(Imu,           "/humanoid/imu",           qos)
            self._pub_odom         = node.create_publisher(Odometry,      "/humanoid/odom",           qos)
            self._pub_robot_state  = node.create_publisher(String,        "/humanoid/robot_state",    10)

            # Subscribers
            from geometry_msgs.msg import Twist
            node.create_subscription(Twist, "/cmd_vel",                    self._on_cmd_vel,        qos)
            node.create_subscription(Bool,  "/humanoid/emergency_stop",    self._on_estop,          10)

        except ImportError as exc:
            logger.warning(f"Some ROS 2 message types unavailable: {exc}")

        return node

    def _spin_loop(self):
        """Background thread that calls rclpy.spin_once repeatedly."""
        while self._running:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.005)
            except Exception as exc:
                if self._running:
                    logger.debug(f"spin_once error: {exc}")

    def _on_cmd_vel(self, msg):
        """Callback for incoming /cmd_vel."""
        with self._cmd_lock:
            self._cmd_vel = np.array([
                msg.linear.x,
                msg.linear.y,
                msg.angular.z,
            ])

    def _on_estop(self, msg):
        """Callback for emergency stop."""
        with self._cmd_lock:
            self._emergency_stop = bool(msg.data)
        if msg.data:
            logger.warning("ROS 2: emergency stop received")

    def _should_publish(self, topic: str, rate_hz: float) -> bool:
        """Rate limiter — returns True if enough time has elapsed."""
        now = time.monotonic()
        last = self._last_pub_times.get(topic, 0.0)
        if now - last >= 1.0 / rate_hz:
            self._last_pub_times[topic] = now
            return True
        return False

    @staticmethod
    def _ros_time(timestamp: Optional[float]):
        """Convert Unix timestamp to ROS 2 builtin_interfaces.Time."""
        t = timestamp or time.time()
        sec  = int(t)
        nsec = int((t - sec) * 1e9)
        try:
            from builtin_interfaces.msg import Time
            ros_t = Time()
            ros_t.sec     = sec
            ros_t.nanosec = nsec
            return ros_t
        except ImportError:
            return None

    @staticmethod
    def _euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Convert roll-pitch-yaw to quaternion [x, y, z, w]."""
        cr, cp, cy = np.cos(roll/2), np.cos(pitch/2), np.cos(yaw/2)
        sr, sp, sy = np.sin(roll/2), np.sin(pitch/2), np.sin(yaw/2)
        return np.array([
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
            cr*cp*cy + sr*sp*sy,
        ])

    @property
    def is_available(self) -> bool:
        """True if rclpy is installed and a real ROS 2 node is running."""
        return self._available and self._node is not None
