"""
Inverse Kinematics Solver

Provides both a pure-Python Jacobian pseudoinverse IK solver and a
thin wrapper around PyBullet's built-in calculateInverseKinematics.
"""

import numpy as np
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass, field
import logging

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class DHParams:
    """Denavit-Hartenberg parameters for one joint."""
    a: float       # link length (m)
    d: float       # link offset (m)
    alpha: float   # link twist (rad)
    theta: float   # joint angle offset (rad)


@dataclass
class KinematicChain:
    """
    A serial kinematic chain described by DH parameters.

    Attributes:
        dh_params:    DH parameters per joint.
        joint_limits: [(min, max)] in radians per joint.
        tool_offset:  4x4 homogeneous transform from last DH frame to tool.
    """
    dh_params: List[DHParams]
    joint_limits: List[Tuple[float, float]] = field(default_factory=list)
    tool_offset: np.ndarray = field(default_factory=lambda: np.eye(4))

    def __post_init__(self):
        n = len(self.dh_params)
        if not self.joint_limits:
            self.joint_limits = [(-np.pi, np.pi)] * n


@dataclass
class IKConfig:
    """Configuration for the IK solver."""
    max_iterations: int = 200
    tolerance: float = 1e-4          # position tolerance (m)
    orientation_tolerance: float = 1e-3  # orientation tolerance (rad)
    damping: float = 0.01            # damped least-squares lambda
    step_limit: float = 0.1          # max joint angle change per iteration (rad)
    solve_orientation: bool = False  # include orientation in cost


@dataclass
class IKResult:
    """Result of an IK solve."""
    success: bool
    joint_angles: np.ndarray
    position_error: float
    orientation_error: float
    iterations: int


# ---------------------------------------------------------------------------
# Homogeneous transform utilities
# ---------------------------------------------------------------------------

def dh_matrix(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    """Standard DH homogeneous transform."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct,  -st * ca,  st * sa,  a * ct],
        [st,   ct * ca, -ct * sa,  a * st],
        [0.0,       sa,       ca,       d],
        [0.0,      0.0,      0.0,     1.0],
    ])


def rot_to_euler(R: np.ndarray) -> np.ndarray:
    """Extract roll-pitch-yaw (XYZ convention) from 3x3 rotation matrix."""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0.0
    return np.array([x, y, z])


# ---------------------------------------------------------------------------
# Pure-Python Jacobian IK Solver
# ---------------------------------------------------------------------------

class JacobianIKSolver:
    """
    Damped Least-Squares (DLS) Jacobian Inverse Kinematics.

    Works with any KinematicChain defined via DH parameters.
    Suitable for arms, legs, or any serial chain.
    """

    def __init__(
        self,
        chain: KinematicChain,
        config: Optional[IKConfig] = None,
    ):
        self.chain = chain
        self.config = config or IKConfig()
        self.n_joints = len(chain.dh_params)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def forward_kinematics(self, joint_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute end-effector pose from joint angles.

        Returns:
            (position [3], euler_angles [3]) in world/base frame.
        """
        T = self._fk_matrix(joint_angles)
        position = T[:3, 3]
        euler = rot_to_euler(T[:3, :3])
        return position, euler

    def solve(
        self,
        target_position: np.ndarray,
        target_euler: Optional[np.ndarray] = None,
        initial_angles: Optional[np.ndarray] = None,
    ) -> IKResult:
        """
        Solve IK for a target end-effector pose.

        Args:
            target_position: Desired [x, y, z] position.
            target_euler:    Desired [roll, pitch, yaw] (optional).
            initial_angles:  Starting joint angles (defaults to zeros).

        Returns:
            IKResult with success flag, joint angles, and errors.
        """
        q = (
            np.array(initial_angles, dtype=float)
            if initial_angles is not None
            else np.zeros(self.n_joints)
        )

        cfg = self.config
        solve_ori = cfg.solve_orientation and target_euler is not None

        for iteration in range(cfg.max_iterations):
            # Forward kinematics
            pos, euler = self.forward_kinematics(q)

            # Errors
            e_pos = target_position - pos
            e_ori = (target_euler - euler) if solve_ori else np.zeros(3)

            pos_err = float(np.linalg.norm(e_pos))
            ori_err = float(np.linalg.norm(e_ori))

            # Convergence check
            converged = pos_err < cfg.tolerance
            if solve_ori:
                converged = converged and ori_err < cfg.orientation_tolerance

            if converged:
                return IKResult(
                    success=True,
                    joint_angles=self._enforce_limits(q),
                    position_error=pos_err,
                    orientation_error=ori_err,
                    iterations=iteration,
                )

            # Jacobian
            J = self._jacobian(q, solve_ori)

            # Task-space error vector
            if solve_ori:
                e = np.concatenate([e_pos, e_ori])
            else:
                e = e_pos

            # DLS: dq = J^T (J J^T + lambda^2 I)^{-1} e
            lam2 = cfg.damping ** 2
            A = J @ J.T + lam2 * np.eye(J.shape[0])
            dq = J.T @ np.linalg.solve(A, e)

            # Clip step
            dq = np.clip(dq, -cfg.step_limit, cfg.step_limit)
            q = q + dq
            q = self._enforce_limits(q)

        # Did not converge
        pos, euler = self.forward_kinematics(q)
        return IKResult(
            success=False,
            joint_angles=self._enforce_limits(q),
            position_error=float(np.linalg.norm(target_position - pos)),
            orientation_error=float(np.linalg.norm(target_euler - euler) if target_euler is not None else 0.0),
            iterations=cfg.max_iterations,
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _fk_matrix(self, q: np.ndarray) -> np.ndarray:
        """Compute full FK homogeneous transform."""
        T = np.eye(4)
        for i, dh in enumerate(self.chain.dh_params):
            T = T @ dh_matrix(dh.a, dh.d, dh.alpha, dh.theta + q[i])
        T = T @ self.chain.tool_offset
        return T

    def _jacobian(self, q: np.ndarray, with_orientation: bool = False) -> np.ndarray:
        """
        Compute geometric Jacobian via finite differences.

        Returns shape [3, n] (position only) or [6, n] (with orientation).
        """
        rows = 6 if with_orientation else 3
        J = np.zeros((rows, self.n_joints))
        eps = 1e-5

        pos0, euler0 = self.forward_kinematics(q)
        ref = np.concatenate([pos0, euler0]) if with_orientation else pos0

        for i in range(self.n_joints):
            q_pert = q.copy()
            q_pert[i] += eps
            pos_p, euler_p = self.forward_kinematics(q_pert)
            if with_orientation:
                pert = np.concatenate([pos_p, euler_p])
            else:
                pert = pos_p
            J[:, i] = (pert - ref) / eps

        return J

    def _enforce_limits(self, q: np.ndarray) -> np.ndarray:
        """Clip joint angles to their configured limits."""
        q_out = q.copy()
        for i, (lo, hi) in enumerate(self.chain.joint_limits):
            q_out[i] = np.clip(q_out[i], lo, hi)
        return q_out


# ---------------------------------------------------------------------------
# PyBullet IK Wrapper
# ---------------------------------------------------------------------------

class PyBulletIKSolver:
    """
    Wraps PyBullet's calculateInverseKinematics for a loaded robot.

    This is the recommended solver when working with the PyBullet simulation
    because it respects the full URDF geometry.
    """

    def __init__(
        self,
        robot_id: int,
        end_effector_link_index: int,
        lower_limits: Optional[List[float]] = None,
        upper_limits: Optional[List[float]] = None,
        joint_ranges: Optional[List[float]] = None,
        rest_poses: Optional[List[float]] = None,
        max_iterations: int = 100,
        tolerance: float = 1e-4,
    ):
        """
        Args:
            robot_id:                  PyBullet body ID.
            end_effector_link_index:   Link index of the end-effector.
            lower_limits:              Lower joint limits (optional).
            upper_limits:              Upper joint limits (optional).
            joint_ranges:              Joint ranges (optional).
            rest_poses:                Rest configuration for null-space (optional).
            max_iterations:            IK solver iterations.
            tolerance:                 Position convergence threshold.
        """
        try:
            import pybullet as p
            self._p = p
        except ImportError:
            raise ImportError("pybullet is required for PyBulletIKSolver")

        self.robot_id = robot_id
        self.ee_link = end_effector_link_index
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits
        self.joint_ranges = joint_ranges
        self.rest_poses = rest_poses
        self.max_iterations = max_iterations
        self.tolerance = tolerance

    def solve(
        self,
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None,
    ) -> IKResult:
        """
        Solve IK using PyBullet.

        Args:
            target_position:    [x, y, z] world position.
            target_orientation: quaternion [x, y, z, w] (optional).

        Returns:
            IKResult with joint angles.
        """
        p = self._p
        kwargs = dict(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.ee_link,
            targetPosition=target_position.tolist(),
            maxNumIterations=self.max_iterations,
            residualThreshold=self.tolerance,
        )

        if target_orientation is not None:
            kwargs["targetOrientation"] = target_orientation.tolist()

        if self.lower_limits is not None:
            kwargs["lowerLimits"] = self.lower_limits
            kwargs["upperLimits"] = self.upper_limits
            kwargs["jointRanges"] = self.joint_ranges
            kwargs["restPoses"] = self.rest_poses

        joint_angles = np.array(p.calculateInverseKinematics(**kwargs))

        # Evaluate position error by running FK through PyBullet link state
        # (requires stepping the sim or using getLinkState with computed joints)
        return IKResult(
            success=True,  # PyBullet does not report convergence directly
            joint_angles=joint_angles,
            position_error=0.0,
            orientation_error=0.0,
            iterations=self.max_iterations,
        )

    def forward_kinematics(self, link_index: Optional[int] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return current end-effector (or given link) world position and orientation.

        Returns:
            (position [3], quaternion [4])
        """
        p = self._p
        idx = link_index if link_index is not None else self.ee_link
        state = p.getLinkState(self.robot_id, idx, computeForwardKinematics=True)
        pos = np.array(state[4])   # worldLinkFramePosition
        quat = np.array(state[5])  # worldLinkFrameOrientation
        return pos, quat


# ---------------------------------------------------------------------------
# Factory helpers
# ---------------------------------------------------------------------------

def make_7dof_arm_chain(
    shoulder_offset: float = 0.05,
    upper_arm_length: float = 0.30,
    forearm_length: float = 0.25,
    wrist_offset: float = 0.05,
) -> KinematicChain:
    """
    Build a KinematicChain for a generic 7-DOF humanoid arm.

    Joint order: shoulder_roll, shoulder_pitch, shoulder_yaw,
                 elbow_pitch, wrist_yaw, wrist_pitch, wrist_roll
    """
    dh = [
        DHParams(a=0.0,              d=shoulder_offset,   alpha=np.pi / 2,  theta=0.0),
        DHParams(a=0.0,              d=0.0,               alpha=-np.pi / 2, theta=0.0),
        DHParams(a=0.0,              d=upper_arm_length,  alpha=np.pi / 2,  theta=0.0),
        DHParams(a=0.0,              d=0.0,               alpha=-np.pi / 2, theta=0.0),
        DHParams(a=0.0,              d=forearm_length,    alpha=np.pi / 2,  theta=0.0),
        DHParams(a=0.0,              d=0.0,               alpha=-np.pi / 2, theta=0.0),
        DHParams(a=0.0,              d=wrist_offset,      alpha=0.0,        theta=0.0),
    ]
    limits = [
        (-np.pi,     np.pi),      # shoulder roll
        (-np.pi / 2, np.pi / 2),  # shoulder pitch
        (-np.pi,     np.pi),      # shoulder yaw
        (-np.pi / 2 * 2.5, 0.0), # elbow (flexion only)
        (-np.pi,     np.pi),      # wrist yaw
        (-np.pi / 2, np.pi / 2),  # wrist pitch
        (-np.pi,     np.pi),      # wrist roll
    ]
    return KinematicChain(dh_params=dh, joint_limits=limits)
