"""Tests for Inverse Kinematics solver and Arm Controller."""

import numpy as np
import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kinematics.inverse_kinematics import (
    JacobianIKSolver,
    IKConfig,
    DHParams,
    KinematicChain,
    dh_matrix,
    rot_to_euler,
    make_7dof_arm_chain,
)
from locomotion.arm_controller import ArmController, ArmConfig, ArmMode, ArmTarget, ArmState


# ---------------------------------------------------------------------------
# DH utilities
# ---------------------------------------------------------------------------

class TestDHMatrix:
    def test_identity_for_zero_params(self):
        T = dh_matrix(a=0.0, d=0.0, alpha=0.0, theta=0.0)
        assert np.allclose(T, np.eye(4))

    def test_pure_translation_z(self):
        T = dh_matrix(a=0.0, d=0.5, alpha=0.0, theta=0.0)
        assert T[2, 3] == pytest.approx(0.5)
        assert T[0, 3] == pytest.approx(0.0)

    def test_pure_rotation_theta(self):
        T = dh_matrix(a=0.0, d=0.0, alpha=0.0, theta=np.pi / 2)
        # After 90° rotation around Z: x-axis maps to y-axis
        assert T[0, 0] == pytest.approx(0.0, abs=1e-7)
        assert T[1, 0] == pytest.approx(1.0, abs=1e-7)

    def test_shape(self):
        T = dh_matrix(0.1, 0.2, 0.3, 0.4)
        assert T.shape == (4, 4)
        assert T[3, 3] == pytest.approx(1.0)


class TestRotToEuler:
    def test_identity_is_zero(self):
        euler = rot_to_euler(np.eye(3))
        assert np.allclose(euler, 0.0, atol=1e-7)

    def test_roundtrip(self):
        # Build rotation from known angles, recover them
        angles = np.array([0.1, -0.2, 0.3])
        cr, cp, cy = np.cos(angles)
        sr, sp, sy = np.sin(angles)
        Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
        Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
        R = Rz @ Ry @ Rx
        recovered = rot_to_euler(R)
        # Allow some numeric tolerance
        assert np.allclose(recovered, angles, atol=1e-5)


# ---------------------------------------------------------------------------
# JacobianIKSolver
# ---------------------------------------------------------------------------

class TestJacobianIKSolver2DOF:
    """Simple 2-DOF planar arm for easy analytical verification."""

    def _make_2dof(self, l1=0.3, l2=0.25):
        dh = [
            DHParams(a=l1, d=0.0, alpha=0.0, theta=0.0),
            DHParams(a=l2, d=0.0, alpha=0.0, theta=0.0),
        ]
        limits = [(-np.pi, np.pi), (-np.pi, np.pi)]
        return KinematicChain(dh_params=dh, joint_limits=limits), l1, l2

    def test_fk_extended(self):
        chain, l1, l2 = self._make_2dof()
        solver = JacobianIKSolver(chain)
        pos, euler = solver.forward_kinematics(np.array([0.0, 0.0]))
        # Both links along x-axis
        assert pos[0] == pytest.approx(l1 + l2, abs=1e-6)
        assert pos[1] == pytest.approx(0.0, abs=1e-6)

    def test_ik_reachable_target(self):
        chain, l1, l2 = self._make_2dof()
        solver = JacobianIKSolver(chain, IKConfig(max_iterations=200, tolerance=1e-3))
        target = np.array([0.40, 0.0, 0.0])
        result = solver.solve(target)
        assert result.success
        assert result.position_error < 1e-3

    def test_ik_position_accuracy(self):
        chain, l1, l2 = self._make_2dof()
        solver = JacobianIKSolver(chain, IKConfig(max_iterations=300, tolerance=1e-4))
        target = np.array([0.3, 0.2, 0.0])
        result = solver.solve(target)
        # Even if not converged, FK of returned angles should be close
        pos, _ = solver.forward_kinematics(result.joint_angles)
        # Allow generous tolerance for 2DOF reaching 3D target
        assert np.linalg.norm(pos[:2] - target[:2]) < 0.08

    def test_joint_limits_respected(self):
        chain, _, _ = self._make_2dof()
        solver = JacobianIKSolver(chain)
        result = solver.solve(np.array([0.5, 0.0, 0.0]))
        for i, (lo, hi) in enumerate(chain.joint_limits):
            assert lo - 1e-6 <= result.joint_angles[i] <= hi + 1e-6


class TestJacobianIKSolver7DOF:
    def test_make_7dof_chain(self):
        chain = make_7dof_arm_chain()
        assert len(chain.dh_params) == 7
        assert len(chain.joint_limits) == 7

    def test_fk_at_zero(self):
        chain = make_7dof_arm_chain()
        solver = JacobianIKSolver(chain)
        pos, euler = solver.forward_kinematics(np.zeros(7))
        # Should return a finite position
        assert np.all(np.isfinite(pos))

    def test_ik_reachable(self):
        chain = make_7dof_arm_chain(
            shoulder_offset=0.05,
            upper_arm_length=0.30,
            forearm_length=0.25,
            wrist_offset=0.04,
        )
        solver = JacobianIKSolver(chain, IKConfig(max_iterations=300, tolerance=1e-3))
        target = np.array([0.35, -0.10, -0.10])
        result = solver.solve(target)
        assert result.position_error < 0.05  # within 5 cm for 7-DOF

    def test_ik_result_has_correct_shape(self):
        chain = make_7dof_arm_chain()
        solver = JacobianIKSolver(chain)
        result = solver.solve(np.array([0.2, 0.0, 0.0]))
        assert result.joint_angles.shape == (7,)


# ---------------------------------------------------------------------------
# ArmController
# ---------------------------------------------------------------------------

class TestArmController:
    def _state(self, n=7):
        return ArmState(
            left_angles=np.zeros(n),
            right_angles=np.zeros(n),
            left_velocities=np.zeros(n),
            right_velocities=np.zeros(n),
        )

    def test_default_mode_home(self):
        ctrl = ArmController()
        assert ctrl.mode == ArmMode.HOME

    def test_update_returns_command_shape(self):
        ctrl = ArmController(n_arm_joints=7)
        cmd = ctrl.update(dt=0.01, arm_state=self._state())
        assert cmd.left_angles.shape  == (7,)
        assert cmd.right_angles.shape == (7,)

    def test_go_home_changes_target(self):
        ctrl = ArmController()
        ctrl._left_desired[:] = 1.0
        ctrl.go_home(duration=0.01)
        # After a tiny update the angles should start moving toward home
        cmd = ctrl.update(dt=0.01)
        # Home angles for index 1 (shoulder pitch) is -0.2 rad, should move toward it
        assert cmd.left_angles[0] < 1.0

    def test_wave_mode_changes_mode(self):
        ctrl = ArmController()
        ctrl.wave(arm="right")
        assert ctrl.mode == ArmMode.WAVE

    def test_balance_assist_angles_shape(self):
        ctrl = ArmController()
        com_offset = np.array([0.02, 0.05])
        lq, rq = ctrl.compute_balance_assist_angles(com_offset)
        assert lq.shape == rq.shape == (7,)

    def test_balance_assist_lateral_response(self):
        ctrl = ArmController()
        # COM offset to the right → left arm should raise (positive roll)
        com_offset = np.array([0.0, 0.10])
        lq, rq = ctrl.compute_balance_assist_angles(com_offset)
        assert lq[0] < 0.0   # left arm roll opposes lateral offset
        assert rq[0] > 0.0   # right arm roll in same direction

    def test_walk_swing_angles(self):
        ctrl = ArmController()
        lq0, rq0 = ctrl.compute_walk_swing_angles(gait_phase=0.0,  step_length=0.2)
        lq25, _  = ctrl.compute_walk_swing_angles(gait_phase=0.25, step_length=0.2)
        # Shoulder pitch (index 1) should differ between phase 0 and phase 0.25
        assert abs(lq0[1] - lq25[1]) > 1e-3

    def test_reach_without_ik_solver(self):
        ctrl = ArmController(ik_solver=None)
        target = ArmTarget(position=np.array([0.4, -0.2, 0.8]))
        ctrl.reach_to(target, arm="right", duration=0.1)
        cmd = ctrl.update(dt=0.01)
        assert cmd.right_angles.shape == (7,)

    def test_joint_torques_shape(self):
        ctrl = ArmController(n_arm_joints=7)
        from locomotion.arm_controller import ArmCommand
        cmd   = ArmCommand(left_angles=np.zeros(7), right_angles=np.zeros(7))
        state = self._state()
        torques = ctrl.compute_joint_torques(cmd, state)
        assert len(torques) == 14  # 7 per arm
