"""
Simulation HAL — PyBullet backend

Wraps HumanoidRobot (simulation/humanoid.py) and exposes the abstract
HardwareInterface so controllers need not know they are simulated.
"""

import numpy as np
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


class SimulationHAL(HardwareInterface):
    """
    Hardware Abstraction Layer backed by PyBullet HumanoidRobot.

    Usage::

        from simulation.humanoid import HumanoidRobot
        from hal.simulation_hal import SimulationHAL

        robot = HumanoidRobot(use_gui=True)
        hal   = SimulationHAL(robot)
        hal.initialize()

        # In control loop:
        states  = hal.read_joint_states()
        imu     = hal.read_imu()
        contact = hal.read_foot_contacts()
        hal.write_joint_commands(commands)
    """

    def __init__(self, robot=None):
        """
        Args:
            robot: HumanoidRobot instance (or None to lazy-init later).
        """
        super().__init__(name="SimulationHAL")
        self._robot = robot
        self._prev_velocity: np.ndarray = np.zeros(3)
        self._imu_dt: float = 0.001
        self._accel_alpha: float = 0.1   # EMA filter for acceleration

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def initialize(self) -> bool:
        if self._robot is None:
            try:
                from simulation.humanoid import HumanoidRobot
                self._robot = HumanoidRobot(use_gui=False)
            except Exception as exc:
                logger.error(f"SimulationHAL: failed to create robot — {exc}")
                return False

        self._connected = True
        self._emergency_stopped = False
        logger.info("SimulationHAL initialised")
        return True

    def shutdown(self):
        if self._robot is not None:
            try:
                self.zero_all_torques()
                self._robot.disconnect()
            except Exception:
                pass
        self._connected = False
        logger.info("SimulationHAL shutdown")

    # ------------------------------------------------------------------
    # Sensors
    # ------------------------------------------------------------------

    def read_joint_states(self) -> Dict[str, JointState]:
        if self._robot is None:
            return {}
        raw = self._robot.get_joint_states()
        return {
            name: JointState(
                name=name,
                position=float(pos),
                velocity=float(vel),
                torque=float(tor),
            )
            for name, (pos, vel, tor) in raw.items()
        }

    def read_imu(self) -> IMUData:
        if self._robot is None:
            return IMUData(
                orientation=np.zeros(3),
                angular_velocity=np.zeros(3),
                linear_acceleration=np.array([0.0, 0.0, 9.81]),
            )

        pos, euler, lin_vel, ang_vel = self._robot.get_base_state()

        # Estimate linear acceleration via first difference
        accel_raw = (lin_vel - self._prev_velocity) / self._imu_dt
        accel_raw[2] += 9.81   # add gravity component (as real IMU would)
        self._prev_velocity = lin_vel.copy()

        return IMUData(
            orientation=euler,
            angular_velocity=ang_vel,
            linear_acceleration=accel_raw,
        )

    def read_foot_contacts(self) -> FootContact:
        if self._robot is None:
            return FootContact()
        lc, rc = self._robot.get_foot_contacts()
        return FootContact(left=bool(lc), right=bool(rc))

    def read_base_state(self) -> Optional[Dict]:
        if self._robot is None:
            return None
        pos, euler, lin_vel, ang_vel = self._robot.get_base_state()
        return {
            "position":    pos,
            "orientation": euler,
            "velocity":    lin_vel,
            "angular_velocity": ang_vel,
        }

    # ------------------------------------------------------------------
    # Actuators
    # ------------------------------------------------------------------

    def write_joint_commands(self, commands: List[JointCommand]):
        if self._robot is None or self._emergency_stopped:
            return

        torques   = {}
        positions = {}

        for cmd in commands:
            if cmd.mode == ControlMode.TORQUE:
                torques[cmd.name] = cmd.value
            elif cmd.mode == ControlMode.POSITION:
                positions[cmd.name] = (cmd.value, cmd.kp, cmd.kd)
            elif cmd.mode == ControlMode.OFF:
                torques[cmd.name] = 0.0

        if torques:
            self._robot.apply_joint_torques(torques)

        if positions:
            # Build position dict and call with average gains
            pos_dict = {n: v for n, (v, kp, kd) in positions.items()}
            # Use gains from first command as representative
            first = next(iter(positions.values()))
            kp, kd = first[1], first[2]
            self._robot.set_joint_positions(pos_dict, kp=kp, kd=kd)

    def emergency_stop(self):
        self._emergency_stopped = True
        if self._robot is not None:
            try:
                self.zero_all_torques()
            except Exception:
                pass
        logger.warning("SimulationHAL: EMERGENCY STOP")

    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------
    # Simulation-specific helpers
    # ------------------------------------------------------------------

    def step(self):
        """Advance physics simulation by one timestep (dt)."""
        if self._robot is not None:
            self._robot.step_simulation()

    @property
    def robot(self):
        """Direct access to underlying HumanoidRobot (use with care)."""
        return self._robot
