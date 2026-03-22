"""
Real Robot HAL — Serial / CAN bus backend

Stub implementation for physical hardware.  Extend this class for your
specific servo protocol (Dynamixel, ODrive, EtherCAT, etc.).

Provides a working stub that returns safe zero values so the rest of the
control stack can be developed and tested without real hardware.
"""

import numpy as np
import time
from typing import Dict, List, Optional
import logging

from hal.hardware_interface import (
    HardwareInterface,
    JointState,
    JointCommand,
    IMUData,
    FootContact,
    ControlMode,
)

logger = logging.getLogger(__name__)


class SerialBusConfig:
    """Configuration for serial / CAN communication"""
    port: str = "/dev/ttyUSB0"
    baud_rate: int = 1_000_000   # 1 Mbaud — typical for Dynamixel
    timeout: float = 0.01        # seconds
    can_interface: str = "can0"  # SocketCAN interface name
    use_can: bool = False        # False = serial, True = CAN


class RealRobotHAL(HardwareInterface):
    """
    Hardware Abstraction Layer for a real robot.

    This is a documented stub. To add hardware support:
    1. Install your servo SDK (e.g., dynamixel_sdk, python-can).
    2. Implement the `_open_bus`, `_read_servo`, `_write_servo` helpers.
    3. Fill in `read_imu` with your IMU driver (e.g., BNO055, ICM-42688).
    4. Fill in `read_foot_contacts` with your FSR / tactile sensor driver.

    The stub returns safe zeroed sensor data so the control stack runs
    without real hardware connected.
    """

    def __init__(
        self,
        joint_names: Optional[List[str]] = None,
        bus_config: Optional[SerialBusConfig] = None,
        name: str = "RealRobotHAL",
    ):
        super().__init__(name=name)
        self._joint_names = joint_names or []
        self._bus_config = bus_config or SerialBusConfig()
        self._bus = None           # Communication bus handle
        self._imu_handle = None    # IMU driver handle

        # Shadow copy of last commanded positions (for safety fallback)
        self._last_position: Dict[str, float] = {n: 0.0 for n in self._joint_names}

        # Timestamps
        self._connect_time: Optional[float] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def initialize(self) -> bool:
        """
        Open communication bus and configure actuators.

        TODO: Replace stub with real driver calls, e.g.::

            from dynamixel_sdk import PortHandler, PacketHandler
            self._port_handler = PortHandler(self._bus_config.port)
            if not self._port_handler.openPort():
                return False
            self._port_handler.setBaudRate(self._bus_config.baud_rate)
        """
        logger.info(f"{self.name}: opening bus (stub — no real hardware)")
        self._connected = True
        self._connect_time = time.time()
        return True

    def shutdown(self):
        """Disable actuators and close communication bus."""
        if self._connected:
            try:
                self.zero_all_torques()
            except Exception:
                pass
            # TODO: self._port_handler.closePort()
            self._connected = False
            logger.info(f"{self.name}: shutdown complete")

    # ------------------------------------------------------------------
    # Sensors
    # ------------------------------------------------------------------

    def read_joint_states(self) -> Dict[str, JointState]:
        """
        Read joint positions, velocities, and torques from servo bus.

        TODO: Replace stub with real reads, e.g.::

            for joint_id, name in self._id_to_name.items():
                pos = self._read_servo_position(joint_id)
                vel = self._read_servo_velocity(joint_id)
                tor = self._read_servo_current(joint_id) * TORQUE_CONSTANT
                states[name] = JointState(name=name, position=pos, ...)
        """
        return {
            name: JointState(
                name=name,
                position=self._last_position.get(name, 0.0),
                velocity=0.0,
                torque=0.0,
            )
            for name in self._joint_names
        }

    def read_imu(self) -> IMUData:
        """
        Read IMU data from onboard sensor.

        TODO: Replace stub with real IMU driver, e.g.::

            raw = self._imu_handle.read()
            return IMUData(
                orientation=np.array(raw["euler"]),
                angular_velocity=np.array(raw["gyro"]),
                linear_acceleration=np.array(raw["accel"]),
                timestamp=time.time(),
            )
        """
        return IMUData(
            orientation=np.zeros(3),
            angular_velocity=np.zeros(3),
            linear_acceleration=np.array([0.0, 0.0, 9.81]),
            timestamp=time.time(),
        )

    def read_foot_contacts(self) -> FootContact:
        """
        Read foot contact / force-sensing resistors.

        TODO: Replace stub with real FSR / load-cell reads.
        """
        return FootContact(left=False, right=False, left_force=0.0, right_force=0.0)

    # ------------------------------------------------------------------
    # Actuators
    # ------------------------------------------------------------------

    def write_joint_commands(self, commands: List[JointCommand]):
        """
        Send joint commands to servos.

        TODO: Replace stub with real writes, e.g.::

            for cmd in commands:
                servo_id = self._name_to_id[cmd.name]
                if cmd.mode == ControlMode.POSITION:
                    self._write_servo_position(servo_id, cmd.value)
                elif cmd.mode == ControlMode.TORQUE:
                    self._write_servo_torque(servo_id, cmd.value)
        """
        if self._emergency_stopped:
            return
        for cmd in commands:
            if cmd.mode in (ControlMode.POSITION, ControlMode.TORQUE):
                if cmd.mode == ControlMode.POSITION:
                    self._last_position[cmd.name] = cmd.value
                logger.debug(f"[HAL stub] {cmd.name}: {cmd.mode.value}={cmd.value:.3f}")

    def emergency_stop(self):
        """Immediately disable all motors (hardware e-stop)."""
        self._emergency_stopped = True
        # TODO: assert e-stop relay / broadcast zero-torque to all servos
        logger.critical(f"{self.name}: EMERGENCY STOP ACTIVATED")

    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------
    # Extension points (override in subclasses)
    # ------------------------------------------------------------------

    def _write_servo_position(self, servo_id: int, angle_rad: float):
        """Send position command to one servo. Override in subclass."""
        pass

    def _write_servo_torque(self, servo_id: int, torque_nm: float):
        """Send torque command to one servo. Override in subclass."""
        pass

    def _read_servo_position(self, servo_id: int) -> float:
        """Read position from one servo. Override in subclass."""
        return 0.0

    def _read_servo_velocity(self, servo_id: int) -> float:
        """Read velocity from one servo. Override in subclass."""
        return 0.0

    def _read_servo_current(self, servo_id: int) -> float:
        """Read motor current from one servo. Override in subclass."""
        return 0.0
