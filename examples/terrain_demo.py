"""
Terrain Adaptation Demo

Demonstrates the robot walking over:
  1. A 10° slope
  2. A staircase (6 steps × 12 cm)

Uses:
  - simulation.terrain.TerrainGenerator  (builds PyBullet terrain)
  - locomotion.terrain_adaptation.TerrainAdaptationController
  - locomotion.balance.BalanceController
  - locomotion.gait.GaitGenerator
  - simulation.humanoid.HumanoidRobot
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import time
import logging

logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(name)s  %(message)s")
logger = logging.getLogger("terrain_demo")


def run_terrain_demo(terrain_type_name: str = "slope", duration: float = 20.0, use_gui: bool = True):
    """
    Run the terrain adaptation demo.

    Args:
        terrain_type_name: 'slope', 'stairs', or 'uneven'
        duration:          Simulation duration in seconds
        use_gui:           Show PyBullet GUI
    """
    import pybullet as p
    import pybullet_data

    from simulation.terrain import TerrainGenerator, TerrainConfig, TerrainType
    from locomotion.terrain_adaptation import TerrainAdaptationController, TerrainAdaptationConfig
    from locomotion.balance import BalanceController, BalanceConfig, create_balance_state_from_simulation
    from locomotion.gait import GaitGenerator, GaitConfig

    # ------------------------------------------------------------------ Setup
    terrain_map = {
        "slope":  TerrainType.SLOPE,
        "stairs": TerrainType.STAIRS,
        "uneven": TerrainType.UNEVEN,
        "flat":   TerrainType.FLAT,
    }
    terrain_type = terrain_map.get(terrain_type_name, TerrainType.SLOPE)

    # PyBullet session
    if use_gui:
        client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    else:
        client = p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    dt = 0.001
    p.setTimeStep(dt)

    # ---- Terrain ----
    terrain_cfg = TerrainConfig(
        terrain_type=terrain_type,
        slope_angle_deg=12.0,
        num_steps=6,
        step_height=0.12,
        step_depth=0.28,
        stair_width=1.5,
    )
    terrain_gen = TerrainGenerator(terrain_cfg)
    terrain_info = terrain_gen.create()
    logger.info(f"Terrain: {terrain_type.value}, bodies={terrain_info.body_ids}")

    # ---- Robot ----
    start_pos = [0.0, 0.0, 1.05]
    orient = p.getQuaternionFromEuler([0, 0, 0])
    robot_id = p.loadURDF("humanoid/humanoid.urdf", start_pos, orient,
                           useFixedBase=False, flags=p.URDF_USE_SELF_COLLISION)
    num_joints = p.getNumJoints(robot_id)

    # Enable torque control
    for i in range(num_joints):
        p.setJointMotorControl2(robot_id, i, p.VELOCITY_CONTROL, force=0)

    # ---- Controllers ----
    balance_ctrl = BalanceController(BalanceConfig())
    gait_gen     = GaitGenerator(GaitConfig(step_length=0.15, step_height=0.08))

    adapt_cfg  = TerrainAdaptationConfig(look_ahead_distance=0.25, step_clearance=0.06)
    adapt_ctrl = TerrainAdaptationController(adapt_cfg)
    adapt_ctrl.set_terrain_generator(terrain_gen)

    # ------------------------------------------------------------------ Loop
    logger.info(f"Running {terrain_type_name} terrain demo for {duration}s …")
    start_time = time.time()
    step = 0
    walked = False

    while time.time() - start_time < duration:
        step += 1
        t = step * dt

        # Start walking after 2 s stabilisation
        if t > 2.0 and not walked:
            gait_gen.start_walking(direction=np.array([1.0, 0.0]))
            walked = True

        # --- Read robot state ---
        pos, quat = p.getBasePositionAndOrientation(robot_id)
        vel, ang_vel = p.getBaseVelocity(robot_id)
        euler = p.getEulerFromQuaternion(quat)

        pos_arr    = np.array(pos)
        euler_arr  = np.array(euler)
        vel_arr    = np.array(vel)
        ang_vel_arr = np.array(ang_vel)

        # --- Terrain height under robot ---
        ground_h = terrain_gen.get_height_at(pos_arr[0], pos_arr[1])
        terrain_normal = terrain_gen.get_normal_at(pos_arr[0], pos_arr[1])

        # --- Balance ---
        balance_state = create_balance_state_from_simulation(
            pos_arr, vel_arr, euler_arr, ang_vel_arr
        )
        ankle_t, hip_t = balance_ctrl.compute_control(balance_state, dt)

        # --- Body lean compensation ---
        lean_comp = adapt_ctrl.get_body_lean_compensation(pos_arr)

        # --- Gait ---
        gait_state = gait_gen.update(dt)

        # --- Terrain-adapted foot placement ---
        left_xy  = np.array([pos_arr[0] + 0.05, pos_arr[1] + 0.10])
        right_xy = np.array([pos_arr[0] + 0.05, pos_arr[1] - 0.10])
        foot_placement = adapt_ctrl.adapt_foot_targets(left_xy, right_xy, 0.0, 0.0)

        # --- COM height adjustment ---
        com_h_adj = adapt_ctrl.get_com_height_adjustment()

        # --- Apply simple PD control (stance — terrain corrected) ---
        kp, kd = 80.0, 8.0
        for i in range(num_joints):
            j_state = p.getJointState(robot_id, i)
            jpos, jvel = j_state[0], j_state[1]

            # Add slope lean to pitch joints
            target = 0.0
            info = p.getJointInfo(robot_id, i)
            name = info[1].decode()
            if "hip" in name.lower() and "pitch" in name.lower():
                target += lean_comp[1]
            if "ankle" in name.lower():
                target += ankle_t[1] * 0.01  # scale down

            torque = kp * (target - jpos) - kd * jvel
            p.setJointMotorControl2(robot_id, i, p.TORQUE_CONTROL,
                                    force=np.clip(torque, -150, 150))

        p.stepSimulation()

        # Progress log
        if step % 1000 == 0:
            terrain_h = terrain_gen.get_height_at(pos_arr[0], pos_arr[1])
            mode = adapt_ctrl.current_mode.value
            logger.info(
                f"t={t:.1f}s  pos=({pos_arr[0]:.2f},{pos_arr[1]:.2f},{pos_arr[2]:.2f})  "
                f"terrain_h={terrain_h:.3f}m  mode={mode}"
            )

        if use_gui:
            time.sleep(dt)

    # ---- Summary ----
    pos, _, _, _ = p.getBaseVelocity(robot_id)
    final_pos, _ = p.getBasePositionAndOrientation(robot_id)
    logger.info(f"\nDone! Final position: {np.array(final_pos).round(3)}")

    p.disconnect(client)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Terrain Adaptation Demo")
    parser.add_argument("--terrain", default="slope",
                        choices=["flat", "slope", "stairs", "uneven"],
                        help="Terrain type")
    parser.add_argument("--duration", type=float, default=20.0,
                        help="Simulation duration (seconds)")
    parser.add_argument("--no-gui", action="store_true",
                        help="Run headless (faster)")
    args = parser.parse_args()

    run_terrain_demo(
        terrain_type_name=args.terrain,
        duration=args.duration,
        use_gui=not args.no_gui,
    )
