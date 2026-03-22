"""
Vision-Based Navigation Demo

Demonstrates:
  1. VisionSystem depth scanning with PyBullet raycasts
  2. Occupancy grid construction from depth data
  3. A* path planning around obstacles
  4. NavigationController driving the robot to a goal

Layout:
  - Robot starts at (0, 0)
  - A few box obstacles are placed at mid-field
  - Goal is at (4, 0)
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import time
import logging

logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(name)s  %(message)s")
logger = logging.getLogger("navigation_demo")


def place_obstacles(num: int = 4):
    """Place random box obstacles in PyBullet."""
    import pybullet as p

    rng = np.random.RandomState(7)
    ids = []
    for _ in range(num):
        x = rng.uniform(1.0, 3.0)
        y = rng.uniform(-0.8, 0.8)
        h = rng.uniform(0.3, 0.8)
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.15, 0.15, h / 2])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.15, 0.15, h / 2],
                                  rgbaColor=[0.8, 0.3, 0.2, 1.0])
        body = p.createMultiBody(baseMass=0,
                                 baseCollisionShapeIndex=col,
                                 baseVisualShapeIndex=vis,
                                 basePosition=[x, y, h / 2])
        ids.append(body)
    return ids


def run_navigation_demo(duration: float = 40.0, use_gui: bool = True):
    """Run the vision-based navigation demo."""
    import pybullet as p
    import pybullet_data

    from sensors.vision import VisionSystem, VisionConfig
    from navigation.path_planner import PathPlanner, NavigationController, NavigationGoal, PathPlannerConfig, NavigationConfig

    # ------------------------------------------------------------------ PyBullet setup
    if use_gui:
        client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(
            cameraDistance=6.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[2.0, 0.0, 0.5],
        )
    else:
        client = p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    dt = 0.001
    p.setTimeStep(dt)

    # Ground
    plane_id = p.loadURDF("plane.urdf")

    # Robot
    start_pos = [0.0, 0.0, 1.0]
    robot_id = p.loadURDF(
        "humanoid/humanoid.urdf", start_pos,
        p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=False,
        flags=p.URDF_USE_SELF_COLLISION,
    )
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        p.setJointMotorControl2(robot_id, i, p.VELOCITY_CONTROL, force=0)

    # Obstacles
    obs_ids = place_obstacles(num=5)
    logger.info(f"Placed {len(obs_ids)} obstacles: {obs_ids}")

    # ------------------------------------------------------------------ Vision & Navigation
    vision_cfg = VisionConfig(
        horizontal_rays=24,
        vertical_rays=16,
        max_range=4.0,
        obstacle_height_min=0.05,
        obstacle_height_max=2.0,
        grid_resolution=0.15,
        grid_radius=3.0,
    )
    vision = VisionSystem(vision_cfg)

    planner_cfg = PathPlannerConfig(heuristic_weight=1.3, obstacle_inflation=2)
    nav_cfg     = NavigationConfig(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        lookahead_distance=0.6,
        goal_slowdown_radius=1.0,
        obstacle_brake_distance=0.3,
        replan_interval=2.0,
    )
    nav_ctrl = NavigationController(
        planner=PathPlanner(planner_cfg),
        config=nav_cfg,
    )

    goal = NavigationGoal(position=np.array([4.0, 0.0]), tolerance=0.25)
    nav_ctrl.set_goal(goal)
    logger.info(f"Navigation goal: {goal.position}")

    # ------------------------------------------------------------------ Control loop
    logger.info(f"Running navigation demo for {duration}s …")
    start_time = time.time()
    step = 0
    kp, kd = 80.0, 8.0

    # Simple standing PD controller baseline (robot stays upright)
    def maintain_upright():
        for i in range(num_joints):
            js = p.getJointState(robot_id, i)
            tau = kp * (0.0 - js[0]) - kd * js[1]
            p.setJointMotorControl2(robot_id, i, p.TORQUE_CONTROL,
                                    force=np.clip(tau, -100, 100))

    vision_update_every = 200   # update vision every 200 ms (200 steps @ 1kHz)
    nav_update_every    = 50

    vel_cmd = np.zeros(3)

    while time.time() - start_time < duration:
        step += 1
        t = step * dt

        pos, quat = p.getBasePositionAndOrientation(robot_id)
        euler = p.getEulerFromQuaternion(quat)
        pos_arr  = np.array(pos)
        yaw      = float(euler[2])

        # Vision update
        if step % vision_update_every == 0:
            vision.update(pos_arr, yaw)
            obstacles = vision.detect_obstacles(pos_arr)
            if obstacles:
                logger.debug(f"Obstacles: {[f'({o.position[0]:.1f},{o.position[1]:.1f}) d={o.distance:.2f}m' for o in obstacles[:3]]}")

        # Navigation update
        if step % nav_update_every == 0:
            nav_dt = nav_update_every * dt
            vel_cmd = nav_ctrl.update(
                dt=nav_dt,
                robot_position=pos_arr,
                robot_yaw=yaw,
                vision_system=vision,
            )

        # Apply velocity command as base force (simplified — real system uses leg control)
        vx, vy, omega = vel_cmd
        base_force = [vx * 30.0, vy * 30.0, 0.0]   # proportional forward push
        base_torque = [0.0, 0.0, omega * 5.0]
        p.applyExternalForce(robot_id, -1, base_force, [0, 0, 0], p.LINK_FRAME)
        p.applyExternalTorque(robot_id, -1, base_torque, p.LINK_FRAME)

        maintain_upright()
        p.stepSimulation()

        if step % 1000 == 0:
            dist_to_goal = np.linalg.norm(pos_arr[:2] - goal.position)
            logger.info(
                f"t={t:.1f}s  pos=({pos_arr[0]:.2f},{pos_arr[1]:.2f})  "
                f"dist_to_goal={dist_to_goal:.2f}m  "
                f"nav={nav_ctrl.nav_state.value}  "
                f"cmd=({vx:.2f},{vy:.2f},{omega:.2f})"
            )

        if nav_ctrl.nav_state.value == "arrived":
            logger.info(f"GOAL REACHED at t={t:.1f}s!")
            break

        if use_gui:
            time.sleep(dt)

    # Summary
    final_pos, _ = p.getBasePositionAndOrientation(robot_id)
    final_pos = np.array(final_pos)
    dist = np.linalg.norm(final_pos[:2] - goal.position)
    logger.info(f"\nDemo finished.  Final position: {final_pos.round(3)}  Distance to goal: {dist:.3f}m")

    p.disconnect(client)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Vision-Based Navigation Demo")
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--no-gui", action="store_true")
    args = parser.parse_args()

    run_navigation_demo(duration=args.duration, use_gui=not args.no_gui)
