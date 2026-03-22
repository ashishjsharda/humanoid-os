"""
Arm Manipulation Controller

Manages arm trajectories, reach-to-target tasks, and balance assistance
for a bipedal humanoid robot.
"""

import numpy as np
from typing import Optional, Dict, Tuple, List
from dataclasses import dataclass, field
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class ArmMode(Enum):
    IDLE = "idle"
    HOME = "home"
    REACH = "reach"
    WAVE = "wave"
    BALANCE_ASSIST = "balance_assist"
    GESTURE = "gesture"


@dataclass
class ArmConfig:
    """Configuration for arm controller"""
    # Shoulder position relative to robot torso centre (metres)
    left_shoulder_offset: np.ndarray = field(
        default_factory=lambda: np.array([-0.20, 0.20, 0.10])
    )
    right_shoulder_offset: np.ndarray = field(
        default_factory=lambda: np.array([-0.20, -0.20, 0.10])
    )

    # Workspace limits (metres from shoulder)
    reach_radius: float = 0.55
    min_reach: float = 0.10

    # PD gains for arm joint control
    kp: float = 80.0
    kd: float = 8.0

    # Max joint velocity (rad/s)
    max_joint_velocity: float = 2.0

    # Balance assist parameters
    balance_arm_gain: float = 0.3   # arm swing amplitude per metre of COM offset
    max_balance_angle: float = np.radians(40)


@dataclass
class ArmTarget:
    """End-effector target for one arm"""
    position: np.ndarray                 # [x, y, z] in robot base frame
    orientation: Optional[np.ndarray] = None  # quaternion [x,y,z,w]
    grip_closed: bool = False


@dataclass
class ArmState:
    """Current arm joint configuration"""
    left_angles: np.ndarray    # joint angles (rad), length = n_arm_joints
    right_angles: np.ndarray
    left_velocities: np.ndarray
    right_velocities: np.ndarray


@dataclass
class ArmCommand:
    """Desired joint angles output by arm controller"""
    left_angles: np.ndarray
    right_angles: np.ndarray


class ArmController:
    """
    High-level arm manipulation controller.

    Capabilities:
    - Move end-effectors to Cartesian targets (via IK)
    - Pre-programmed motions: home, wave, gesture
    - Balance assistance: counter-swing arms to stabilise walking
    - Smooth trajectory interpolation between configurations
    """

    # Default home pose joint angles (shoulder_roll, shoulder_pitch, shoulder_yaw,
    # elbow, wrist_yaw, wrist_pitch, wrist_roll) — approx natural hang
    _HOME_ANGLES = np.array([0.0, -0.20, 0.0, -0.40, 0.0, 0.0, 0.0])

    def __init__(
        self,
        config: Optional[ArmConfig] = None,
        ik_solver=None,       # Optional JacobianIKSolver or PyBulletIKSolver
        n_arm_joints: int = 7,
    ):
        self.config = config or ArmConfig()
        self.ik_solver = ik_solver
        self.n = n_arm_joints

        self.mode = ArmMode.HOME
        self._time = 0.0

        # Desired joint angles (tracked internally)
        self._left_desired = self._HOME_ANGLES.copy()
        self._right_desired = self._HOME_ANGLES.copy()

        # Trajectory state
        self._traj_start_l: Optional[np.ndarray] = None
        self._traj_goal_l: Optional[np.ndarray] = None
        self._traj_start_r: Optional[np.ndarray] = None
        self._traj_goal_r: Optional[np.ndarray] = None
        self._traj_duration = 1.0
        self._traj_elapsed = 0.0

        logger.info(f"Arm controller initialised ({n_arm_joints} DOF per arm)")

    # ------------------------------------------------------------------
    # Command interface
    # ------------------------------------------------------------------

    def set_mode(self, mode: ArmMode):
        """Switch arm operating mode."""
        if mode != self.mode:
            logger.info(f"Arm mode: {self.mode.value} -> {mode.value}")
            self.mode = mode

    def reach_to(
        self,
        target: ArmTarget,
        arm: str = "right",
        duration: float = 1.5,
    ):
        """
        Command an arm to reach toward a Cartesian target.

        Args:
            target:   End-effector target.
            arm:      'left' or 'right'.
            duration: Motion duration (seconds).
        """
        goal_angles = self._ik_for_target(target, arm)
        if goal_angles is None:
            logger.warning("IK failed — staying in current configuration")
            return

        self._start_trajectory(arm, goal_angles, duration)
        self.mode = ArmMode.REACH

    def go_home(self, duration: float = 1.0):
        """Return both arms to natural hang position."""
        self._start_trajectory("left", self._HOME_ANGLES, duration)
        self._start_trajectory("right", self._HOME_ANGLES, duration)
        self.mode = ArmMode.HOME

    def wave(self, arm: str = "right"):
        """Trigger a wave gesture (runs for ~2 s)."""
        self.mode = ArmMode.WAVE
        self._wave_arm = arm
        self._wave_phase = 0.0

    # ------------------------------------------------------------------
    # Main update (call every control cycle)
    # ------------------------------------------------------------------

    def update(
        self,
        dt: float,
        arm_state: Optional[ArmState] = None,
        com_velocity: Optional[np.ndarray] = None,
    ) -> ArmCommand:
        """
        Compute desired arm joint angles for this cycle.

        Args:
            dt:           Control timestep (seconds).
            arm_state:    Current arm joint angles/velocities (optional).
            com_velocity: Robot COM velocity [vx, vy, vz] for balance assist.

        Returns:
            ArmCommand with desired joint angles for both arms.
        """
        self._time += dt
        self._traj_elapsed += dt

        if self.mode == ArmMode.HOME:
            self._update_trajectory(dt)

        elif self.mode == ArmMode.REACH:
            self._update_trajectory(dt)

        elif self.mode == ArmMode.WAVE:
            self._update_wave(dt)

        elif self.mode == ArmMode.BALANCE_ASSIST:
            if com_velocity is not None:
                self._update_balance_assist(com_velocity)

        return ArmCommand(
            left_angles=self._left_desired.copy(),
            right_angles=self._right_desired.copy(),
        )

    def compute_joint_torques(
        self,
        command: ArmCommand,
        state: ArmState,
    ) -> Dict[str, float]:
        """
        Convert desired angles to PD torques for each arm joint.

        Returns:
            Dict mapping joint_name -> torque (Nm).
        """
        torques = {}
        cfg = self.config

        for side, desired, current_pos, current_vel in [
            ("left",  command.left_angles,  state.left_angles,  state.left_velocities),
            ("right", command.right_angles, state.right_angles, state.right_velocities),
        ]:
            for i in range(self.n):
                name = f"{side}_arm_joint_{i}"
                err = desired[i] - current_pos[i]
                torque = cfg.kp * err - cfg.kd * current_vel[i]
                torques[name] = float(np.clip(torque, -150.0, 150.0))

        return torques

    # ------------------------------------------------------------------
    # Balance assistance
    # ------------------------------------------------------------------

    def compute_balance_assist_angles(
        self, com_offset: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute arm angles that counteract a lateral COM offset.

        For example, if COM drifts right, extend left arm outward.

        Args:
            com_offset: [x, y] COM offset from support polygon centre.

        Returns:
            (left_angles, right_angles)
        """
        cfg = self.config
        lateral = float(com_offset[1]) if len(com_offset) >= 2 else 0.0

        # Shoulder roll (index 0): positive = raise arm outward
        left_roll  = np.clip(-lateral * cfg.balance_arm_gain, -cfg.max_balance_angle, cfg.max_balance_angle)
        right_roll = np.clip( lateral * cfg.balance_arm_gain, -cfg.max_balance_angle, cfg.max_balance_angle)

        left_q  = self._HOME_ANGLES.copy()
        right_q = self._HOME_ANGLES.copy()
        left_q[0]  = left_roll
        right_q[0] = right_roll

        return left_q, right_q

    def compute_walk_swing_angles(
        self, gait_phase: float, step_length: float = 0.15
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Counter-phase arm swing during walking (natural human-like motion).

        Args:
            gait_phase: Gait cycle phase [0, 1).
            step_length: Current step length (scales swing amplitude).

        Returns:
            (left_angles, right_angles)
        """
        amplitude = np.clip(step_length * 1.5, 0.0, np.radians(35))

        # Left arm swings forward when right leg swings (anti-phase)
        left_pitch  = amplitude * np.sin(2 * np.pi * gait_phase)
        right_pitch = -amplitude * np.sin(2 * np.pi * gait_phase)

        left_q  = self._HOME_ANGLES.copy()
        right_q = self._HOME_ANGLES.copy()

        # Shoulder pitch is index 1
        left_q[1]  = self._HOME_ANGLES[1] + left_pitch
        right_q[1] = self._HOME_ANGLES[1] + right_pitch

        return left_q, right_q

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _ik_for_target(
        self, target: ArmTarget, arm: str
    ) -> Optional[np.ndarray]:
        """Solve IK for a given target. Returns joint angles or None."""
        if self.ik_solver is None:
            # Approximate: use home angles + simple geometric heuristic
            logger.debug("No IK solver — using home angles as fallback")
            return self._HOME_ANGLES.copy()

        shoulder = (
            self.config.left_shoulder_offset
            if arm == "left"
            else self.config.right_shoulder_offset
        )
        local_target = target.position - shoulder
        result = self.ik_solver.solve(local_target, target.orientation)
        if result.success:
            return result.joint_angles[: self.n]
        return None

    def _start_trajectory(self, arm: str, goal: np.ndarray, duration: float):
        if arm == "left":
            self._traj_start_l = self._left_desired.copy()
            self._traj_goal_l  = goal[: self.n]
        else:
            self._traj_start_r = self._right_desired.copy()
            self._traj_goal_r  = goal[: self.n]
        self._traj_duration = max(duration, 0.1)
        self._traj_elapsed  = 0.0

    def _update_trajectory(self, dt: float):
        """Minimum-jerk trajectory interpolation."""
        t = min(self._traj_elapsed / self._traj_duration, 1.0)
        # Minimum-jerk s(t) = 10t^3 - 15t^4 + 6t^5
        s = 10 * t**3 - 15 * t**4 + 6 * t**5

        if self._traj_goal_l is not None and self._traj_start_l is not None:
            self._left_desired  = self._traj_start_l + s * (self._traj_goal_l  - self._traj_start_l)
        if self._traj_goal_r is not None and self._traj_start_r is not None:
            self._right_desired = self._traj_start_r + s * (self._traj_goal_r - self._traj_start_r)

    def _update_wave(self, dt: float):
        """Animate a wave gesture."""
        self._wave_phase += dt * 3.0  # ~3 rad/s oscillation
        arm = getattr(self, "_wave_arm", "right")

        wave_q = self._HOME_ANGLES.copy()
        wave_q[1] = -np.radians(60) + np.radians(20) * np.sin(self._wave_phase)  # shoulder pitch
        wave_q[0] =  np.radians(40)                                               # shoulder roll out

        if arm == "right":
            self._right_desired = wave_q
        else:
            self._left_desired  = wave_q

        if self._wave_phase > 4 * np.pi:
            self.go_home()

    def _update_balance_assist(self, com_velocity: np.ndarray):
        offset = com_velocity[:2] * 0.1  # rough offset estimate
        lq, rq = self.compute_balance_assist_angles(offset)
        self._left_desired  = lq
        self._right_desired = rq
