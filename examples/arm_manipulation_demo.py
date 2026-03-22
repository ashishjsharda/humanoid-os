"""
Arm Manipulation & IK Demo

Demonstrates:
  1. Loading the IK solver for a 7-DOF arm chain
  2. Solving reach-to-target poses
  3. ArmController motions: home, wave, balance-assist arm swing
  4. Integration with the simulated humanoid
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import time
import logging

logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(name)s  %(message)s")
logger = logging.getLogger("arm_demo")


def run_arm_demo(duration: float = 15.0, use_gui: bool = True):
    """
    Run arm manipulation demo.

    Sequence (3-second phases):
      0 – 3 s   : home position
      3 – 6 s   : reach right hand to a target in front
      6 – 9 s   : wave (right arm)
      9 – 12 s  : balance-assist arm swing (walking counter-swing)
      12 – 15 s : return home
    """
    from kinematics.inverse_kinematics import (
        JacobianIKSolver,
        IKConfig,
        make_7dof_arm_chain,
    )
    from locomotion.arm_controller import ArmController, ArmConfig, ArmMode, ArmTarget, ArmState

    # ---- IK solver for a generic 7-DOF arm ----
    chain  = make_7dof_arm_chain(
        shoulder_offset=0.05,
        upper_arm_length=0.28,
        forearm_length=0.24,
        wrist_offset=0.04,
    )
    ik_cfg = IKConfig(max_iterations=150, tolerance=1e-3, damping=0.02)
    solver = JacobianIKSolver(chain, ik_cfg)

    # ---- Quick IK test ----
    target_pos = np.array([0.30, -0.20, -0.15])  # in shoulder frame
    result = solver.solve(target_pos)
    logger.info(f"IK test: success={result.success}  pos_err={result.position_error:.4f}m  iters={result.iterations}")
    ee_pos, ee_euler = solver.forward_kinematics(result.joint_angles)
    logger.info(f"  FK at solution: {ee_pos.round(3)}")

    # ---- Arm controller ----
    arm_cfg = ArmConfig(kp=80.0, kd=8.0)
    arm_ctrl = ArmController(config=arm_cfg, ik_solver=solver, n_arm_joints=7)

    # ---- Simulated arm state (7 joints per arm, all zero) ----
    def make_state(lq=None, rq=None):
        n = 7
        return ArmState(
            left_angles     = lq if lq is not None else np.zeros(n),
            right_angles    = rq if rq is not None else np.zeros(n),
            left_velocities = np.zeros(n),
            right_velocities= np.zeros(n),
        )

    logger.info(f"Running arm manipulation demo for {duration}s …")
    t = 0.0
    dt = 0.01  # 100 Hz demo loop
    n_steps = int(duration / dt)
    phase = "home"

    for step in range(n_steps):
        t = step * dt

        # Phase transitions
        if t < 3.0:
            if phase != "home":
                phase = "home"
                arm_ctrl.go_home(duration=0.5)
                logger.info("Phase: home")
        elif t < 6.0:
            if phase != "reach":
                phase = "reach"
                target = ArmTarget(position=np.array([0.45, -0.20, 0.80]))
                arm_ctrl.reach_to(target, arm="right", duration=1.5)
                logger.info("Phase: reach to target")
        elif t < 9.0:
            if phase != "wave":
                phase = "wave"
                arm_ctrl.wave(arm="right")
                logger.info("Phase: wave")
        elif t < 12.0:
            if phase != "balance":
                phase = "balance"
                arm_ctrl.set_mode(ArmMode.BALANCE_ASSIST)
                logger.info("Phase: balance-assist swing")
        else:
            if phase != "home2":
                phase = "home2"
                arm_ctrl.go_home(duration=1.0)
                logger.info("Phase: return home")

        # Compute gait phase for balance-assist swing
        gait_phase = (t % 1.0)  # 1 Hz simulated gait
        com_velocity = np.array([0.3, 0.05, 0.0])  # simulated walking COM velocity

        # Update arm controller
        cmd = arm_ctrl.update(dt, arm_state=make_state(), com_velocity=com_velocity)

        # Walk-swing angles (would be used during actual walking)
        lq_swing, rq_swing = arm_ctrl.compute_walk_swing_angles(gait_phase, step_length=0.15)

        # Log progress
        if step % 100 == 0:
            logger.info(
                f"t={t:.2f}s  phase={phase}  "
                f"left[0]={np.degrees(cmd.left_angles[0]):+.1f}°  "
                f"right[1]={np.degrees(cmd.right_angles[1]):+.1f}°"
            )

    # ---- IK workspace sweep demo ----
    logger.info("\nIK workspace sweep:")
    test_targets = [
        np.array([0.30,  0.10, -0.10]),   # forward-up
        np.array([0.20, -0.30, -0.20]),   # forward-low-right
        np.array([0.40,  0.00,  0.10]),   # extended forward
    ]
    for tgt in test_targets:
        res = solver.solve(tgt)
        ee, _ = solver.forward_kinematics(res.joint_angles)
        logger.info(
            f"  target={tgt.round(3)}  "
            f"achieved={ee.round(3)}  "
            f"err={res.position_error:.4f}m  "
            f"ok={res.success}"
        )

    logger.info("Arm manipulation demo complete!")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Arm Manipulation & IK Demo")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--no-gui", action="store_true")
    args = parser.parse_args()

    run_arm_demo(duration=args.duration, use_gui=not args.no_gui)
