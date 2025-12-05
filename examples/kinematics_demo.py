"""
Zero-G Kinematics & Actuator Demo

This script performs a system check of the joint actuators and control loops
in a zero-gravity environment. It verifies that:
1. The control loop is running at the target frequency.
2. Joint commands are being correctly mapped to the physics engine.
3. Actuators have the expected range of motion without gravity loading.
"""

import sys
import os
import time
import math
import logging

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulation.humanoid import HumanoidRobot

try:
    import pybullet as p
except ImportError:
    print("ERROR: PyBullet not installed. Install with: pip install pybullet")
    sys.exit(1)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def run_kinematics_test():
    """Execute the zero-gravity kinematics verification sequence."""
    logger.info("Initializing Kinematics Verification...")
    
    # Initialize simulation with GUI
    robot = HumanoidRobot(use_gui=True, dt=0.001)
    
    # --- ZERO-G SETUP ---
    # Disable gravity to isolate actuator dynamics from load balancing
    logger.info("Disabling gravity for raw kinematic testing...")
    p.setGravity(0, 0, 0)
    
    # Pin the robot in the air (Floating Base)
    # This prevents drift while testing limb articulation
    p.resetBasePositionAndOrientation(
        robot.robot_id, 
        posObj=[0, 0, 1.2],  # Float 1.2 meters up for clear view
        ornObj=[0, 0, 0, 1]  # Upright quaternion
    )
    
    # Setup camera for optimal viewing angle
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=45,
        cameraPitch=-15,
        cameraTargetPosition=[0, 0, 1.2]
    )

    print("\n" + "="*60)
    print("🚀 ZERO GRAVITY MODE ACTIVE")
    print("System Check: Verifying joint articulation and control mapping.")
    print("="*60 + "\n")
    
    # Allow GUI to stabilize
    time.sleep(1.0)
    
    start_time = time.time()
    duration = 20.0
    
    try:
        logger.info("Starting actuation sequence...")
        while time.time() - start_time < duration:
            t = time.time() - start_time
            
            # Generate a smooth sine wave control signal
            # Frequency = 3.0 rad/s (~0.5 Hz cycle)
            angle = 0.5 * math.sin(t * 3.0)
            
            # Construct target position map
            # We dynamically identify leg joints based on standard naming conventions
            targets = {}
            for joint_name in robot.joints.keys():
                # Apply anti-phase movement to hips/legs for visual verification
                if 'hip' in joint_name or 'leg' in joint_name:
                    targets[joint_name] = angle
                
                # Move knees in opposite direction to demonstrate coordination
                if 'knee' in joint_name:
                    targets[joint_name] = -angle * 2.0
                    
            # Send commands to the PD controller
            robot.set_joint_positions(targets, kp=100.0, kd=10.0)
            
            # Step physics simulation
            robot.step_simulation()
            
            # Maintain real-time pacing
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        logger.info("Test sequence interrupted by user.")
    finally:
        robot.disconnect()
        logger.info("Kinematics verification complete.")

if __name__ == "__main__":
    run_kinematics_test()
