"""
Integrated Balance Control Demo

Demonstrates HumanoidOS balance controller with simulated robot.
"""

import sys
import os
import numpy as np
import logging
import time

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.control_loop import ControlLoop, ControlConfig, SystemState
from locomotion.balance import (
    BalanceController, BalanceConfig, 
    create_balance_state_from_simulation
)
from simulation.humanoid import HumanoidRobot

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class BalanceControlDemo:
    """
    Integrated demo of balance control system.
    
    Combines:
    - HumanoidOS control loop
    - Balance controller
    - PyBullet simulation
    """
    
    def __init__(self, use_gui: bool = True):
        # Initialize components
        logger.info("Initializing balance control demo...")
        
        # Control loop at 1kHz
        control_config = ControlConfig(frequency_hz=1000.0)
        self.control_loop = ControlLoop(control_config)
        
        # Balance controller
        balance_config = BalanceConfig()
        self.balance_controller = BalanceController(balance_config)
        
        # Simulation (at 1kHz to match control)
        self.robot = HumanoidRobot(use_gui=use_gui, dt=0.001)
        
        # State tracking
        self.time = 0.0
        self.balance_violations = 0
        self.total_steps = 0
        
        # Register subsystems
        self.control_loop.register_subsystem("balance", self._balance_subsystem)
        self.control_loop.add_pre_step_callback(self._read_sensors)
        self.control_loop.add_post_step_callback(self._apply_control)
        
        # Storage for current state
        self.current_balance_state = None
        self.current_control = None
        
        logger.info("Demo initialized successfully")
    
    def _read_sensors(self, dt: float, state: SystemState):
        """Read sensor data from simulation"""
        # Get robot state from simulation
        position, orientation, velocity, angular_velocity = self.robot.get_base_state()
        left_contact, right_contact = self.robot.get_foot_contacts()
        
        # Create balance state
        self.current_balance_state = create_balance_state_from_simulation(
            position=position,
            velocity=velocity,
            orientation=orientation,
            angular_velocity=angular_velocity,
            left_contact=left_contact,
            right_contact=right_contact
        )
        
        self.time += dt
        self.total_steps += 1
    
    def _balance_subsystem(self, dt: float, state: SystemState):
        """Balance control subsystem"""
        if self.current_balance_state is None:
            return None
        
        # Compute balance control
        ankle_torques, hip_torques = self.balance_controller.compute_control(
            self.current_balance_state, dt
        )
        
        # Check stability
        if not self.balance_controller.is_stable(self.current_balance_state):
            self.balance_violations += 1
            if self.balance_violations > 100:  # Allow some tolerance
                logger.warning("Balance lost!")
        else:
            self.balance_violations = 0
        
        # Store control outputs
        self.current_control = {
            'ankle_torques': ankle_torques,
            'hip_torques': hip_torques,
            'balance_state': self.current_balance_state
        }
        
        return self.current_control
    
    def _apply_control(self, dt: float, state: SystemState, outputs: dict):
        """Apply control to simulation"""
        if self.current_control is None:
            return
        
        # Map control outputs to joint torques
        # This is simplified - real robot would have proper joint mapping
        torques = {}
        
        # Apply ankle torques (simplified mapping)
        ankle_torques = self.current_control['ankle_torques']
        for joint_name in self.robot.leg_joints:
            if 'ankle' in joint_name.lower():
                # Apply roll/pitch torques
                torques[joint_name] = ankle_torques[0] if 'roll' in joint_name.lower() else ankle_torques[1]
        
        # Apply hip torques
        hip_torques = self.current_control['hip_torques']
        for joint_name in self.robot.leg_joints:
            if 'hip' in joint_name.lower():
                torques[joint_name] = hip_torques[0]
        
        # Apply torques to robot
        self.robot.apply_joint_torques(torques)
        
        # Step simulation
        self.robot.step_simulation()
        
        # Add debug visualization
        if self.total_steps % 100 == 0:  # Every 100ms
            self.robot.add_debug_lines()
    
    def run(self, duration: float = 10.0, apply_disturbance: bool = False):
        """
        Run the demo.
        
        Args:
            duration: How long to run (seconds)
            apply_disturbance: Whether to apply external disturbances
        """
        logger.info(f"Running balance control demo for {duration} seconds...")
        
        if apply_disturbance:
            logger.info("External disturbances will be applied")
        
        start_time = time.time()
        
        # Run control loop
        try:
            iteration = 0
            while time.time() - start_time < duration:
                # Optional: Apply external disturbance
                if apply_disturbance and iteration == 2000:  # At 2 seconds
                    logger.info("Applying external push...")
                    # Apply impulse to robot base
                    import pybullet as p
                    p.applyExternalForce(
                        self.robot.robot_id,
                        -1,  # Base link
                        [200, 0, 0],  # Force in x direction
                        [0, 0, 0],
                        p.WORLD_FRAME
                    )
                
                # Execute control step
                self.control_loop.step()
                iteration += 1
                
                # Real-time pacing
                time.sleep(0.0001)  # Small sleep to prevent CPU spinning
                
        except KeyboardInterrupt:
            logger.info("Demo interrupted by user")
        
        # Print statistics
        self._print_statistics()
    
    def _print_statistics(self):
        """Print demo statistics"""
        logger.info("=" * 60)
        logger.info("Balance Control Demo Statistics:")
        logger.info(f"  Total time: {self.time:.2f}s")
        logger.info(f"  Total control steps: {self.total_steps}")
        logger.info(f"  Average frequency: {self.total_steps/self.time:.1f} Hz")
        
        if self.current_balance_state:
            logger.info(f"  Final height: {self.current_balance_state.com_position[2]:.3f}m")
            logger.info(f"  Final balance: {'✓' if self.current_balance_state.is_balanced else '✗'}")
            logger.info(f"  Stability margin: {self.current_balance_state.stability_margin:.3f}m")
        
        logger.info("=" * 60)
    
    def cleanup(self):
        """Cleanup resources"""
        self.robot.disconnect()
        logger.info("Demo cleanup complete")


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='HumanoidOS Balance Control Demo')
    parser.add_argument('--no-gui', action='store_true', help='Run without GUI')
    parser.add_argument('--duration', type=float, default=10.0, help='Duration in seconds')
    parser.add_argument('--disturbance', action='store_true', help='Apply external disturbance')
    
    args = parser.parse_args()
    
    print("\n" + "=" * 60)
    print("HumanoidOS - Balance Control Demo")
    print("=" * 60)
    print("\nThis demo shows:")
    print("  • Real-time control loop at 1kHz")
    print("  • ZMP-based balance controller")
    print("  • PyBullet physics simulation")
    print("  • Integrated sensor feedback")
    print("\nThe humanoid robot will maintain balance in standing pose.")
    if args.disturbance:
        print("An external push will be applied after 2 seconds.")
    print("\nPress Ctrl+C to stop early.")
    print("=" * 60 + "\n")
    
    # Run demo
    demo = BalanceControlDemo(use_gui=not args.no_gui)
    
    try:
        demo.run(duration=args.duration, apply_disturbance=args.disturbance)
    finally:
        demo.cleanup()
    
    print("\n" + "=" * 60)
    print("Demo Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
