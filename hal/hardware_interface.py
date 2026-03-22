"""
Abstract Hardware Interface

Defines the contract that every HAL backend (simulation, real robot, mock)
must satisfy. All controllers talk exclusively to this interface.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional
import numpy as np
import logging

logger = logging.getLogger(__name__)


class ControlMode(Enum):
    TORQUE   = "torque"
    POSITION = "position"
    VELOCITY = "velocity"
    OFF      = "off"


@dataclass
class JointState:
    """Sensor readback from one joint"""
    name: str
    position: float      # rad
    velocity: float      # rad/s
    torque: float        # Nm
    temperature: float = 0.0   # °C (if available)


@dataclass
class JointCommand:
    """Desired command for one joint"""
    name: str
    mode: ControlMode    = ControlMode.TORQUE
    value: float         = 0.0     # torque (Nm) / angle (rad) / velocity (rad/s)
    # PD gains used in POSITION mode
    kp: float = 100.0
    kd: float = 10.0


@dataclass
class IMUData:
    """6-DOF inertial measurement"""
    orientation: np.ndarray        # [roll, pitch, yaw] rad  OR quaternion [x,y,z,w]
    angular_velocity: np.ndarray   # [wx, wy, wz]  rad/s
    linear_acceleration: np.ndarray # [ax, ay, az]  m/s²
    timestamp: float = 0.0


@dataclass
class FootContact:
    """Foot contact sensor data"""
    left: bool   = False
    right: bool  = False
    left_force: float  = 0.0   # N (if force sensor available)
    right_force: float = 0.0


class HardwareInterface(ABC):
    """
    Abstract base class for all hardware backends.

    Lifecycle:
        1. Construct
        2. initialize() — open connections, home, enable motors
        3. (control loop) read_* / write_* calls at control frequency
        4. shutdown()    — disable motors, close connections

    Thread safety:
        Not assumed. The caller (ControlLoop) is responsible for
        serialising access from a single thread.
    """

    def __init__(self, name: str = "HAL"):
        self.name = name
        self._connected = False
        self._emergency_stopped = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @abstractmethod
    def initialize(self) -> bool:
        """
        Open connections and bring hardware to a safe initial state.

        Returns:
            True on success, False on failure.
        """

    @abstractmethod
    def shutdown(self):
        """Disable actuators and close connections gracefully."""

    # ------------------------------------------------------------------
    # Sensors
    # ------------------------------------------------------------------

    @abstractmethod
    def read_joint_states(self) -> Dict[str, JointState]:
        """
        Read current position / velocity / torque for all joints.

        Returns:
            Dict mapping joint name -> JointState.
        """

    @abstractmethod
    def read_imu(self) -> IMUData:
        """Read IMU (orientation, angular velocity, linear acceleration)."""

    @abstractmethod
    def read_foot_contacts(self) -> FootContact:
        """Read foot contact / force sensors."""

    def read_base_state(self) -> Optional[Dict]:
        """
        Optional: return base link position and velocity if available.
        Simulation backends can provide this; real robots typically cannot.

        Returns:
            Dict with keys 'position', 'velocity', 'orientation', or None.
        """
        return None

    # ------------------------------------------------------------------
    # Actuators
    # ------------------------------------------------------------------

    @abstractmethod
    def write_joint_commands(self, commands: List[JointCommand]):
        """
        Send joint commands to hardware.

        Args:
            commands: List of JointCommand — one per joint to command.
        """

    @abstractmethod
    def emergency_stop(self):
        """
        Immediately disable all actuators (safety stop).

        Must be idempotent and callable from any thread.
        """

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    @abstractmethod
    def is_connected(self) -> bool:
        """Return True if the hardware connection is active."""

    def is_emergency_stopped(self) -> bool:
        return self._emergency_stopped

    def joint_names(self) -> List[str]:
        """Return list of all joint names known to this HAL."""
        return list(self.read_joint_states().keys())

    # ------------------------------------------------------------------
    # Convenience helpers (non-abstract)
    # ------------------------------------------------------------------

    def command_position(
        self,
        positions: Dict[str, float],
        kp: float = 100.0,
        kd: float = 10.0,
    ):
        """Convenience: send position commands to multiple joints."""
        cmds = [
            JointCommand(name=n, mode=ControlMode.POSITION, value=v, kp=kp, kd=kd)
            for n, v in positions.items()
        ]
        self.write_joint_commands(cmds)

    def command_torque(self, torques: Dict[str, float]):
        """Convenience: send torque commands to multiple joints."""
        cmds = [
            JointCommand(name=n, mode=ControlMode.TORQUE, value=v)
            for n, v in torques.items()
        ]
        self.write_joint_commands(cmds)

    def zero_all_torques(self):
        """Send zero torque to every joint."""
        states = self.read_joint_states()
        cmds = [
            JointCommand(name=n, mode=ControlMode.TORQUE, value=0.0)
            for n in states.keys()
        ]
        self.write_joint_commands(cmds)

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__} name={self.name!r} connected={self._connected}>"
