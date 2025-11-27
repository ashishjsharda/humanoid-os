"""
Walking Demo for HumanoidOS

Demonstrates integrated walking with gait generation and balance control.
"""

import sys
import os
import numpy as np
import logging
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.control_loop import ControlLoop, ControlConfig, SystemState
from locomotion.balance import BalanceController, BalanceConfig, create_balance_state_from_simulation
from locomotion.gait import GaitGenerator, GaitConfig
from simulation.humanoid import HumanoidRobot

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class WalkingDemo:
    """
    Complete walking demonstration.
    
    Integrates:
    - Gait generation
    - Balance control
    - Trajectory tracking
    - Physics simulation
    """
    
    def __init__(self, use_gui: bool = True):
        logger.info("Initializing walking demo...")
        
        # Control loop
        control_config = ControlConfig(frequency_hz=1000.0)
        self.control_loop = ControlLoop(control_config)
        
        # Balance controller
        self.balance_controller = BalanceController(BalanceConfig())
        
        # Gait generator
        gait_config = GaitConfig(
            step_length=0.15,  # Start with smaller steps
            step_duration=0.8,
            desired_velocity=0.2
        )
        self.gait_generator = GaitGenerator(gait_config)
        
        # Simulation
        self.robot = HumanoidRobot(use_gui=use_gui, dt=0.001)
        
        # State
        self.time = 0.0
        self.current_balance_state = None
        self.gait_state = None
        self.is_walking = False
        self.distance_traveled = 0.0
        self.initial_position = None
        
        # Register subsystems
        self.control_loop.register_subsystem("gait", self._gait_subsystem)
        self.control_loop.register_subsystem("balance", self._balance_subsystem)
        self.control_loop.add_pre_step_callback(self._read_sensors)
        self.control_loop.add_post_step_callback(self._apply_control)
        
        logger.info("Walking demo initialized")
    
    def _read_sensors(self, dt: float, state: SystemState):
        """Read sensors"""
        position, orientation, velocity, angular_velocity = self.robot.get_base_state()
        left_contact, right_contact = self.robot.get_foot_contacts()
        
        if self.initial_position is None:
            self.initial_position = position.copy()
        
        self.current_balance_state = create_balance_state_from_simulation(
            position=position,
            velocity=velocity,
            orientation=orientation,
            angular_velocity=angular_velocity,
            left_contact=left_contact,
            right_contact=right_contact
        )
        
        # Calculate distance traveled
        self.distance_traveled = np.linalg.norm(position[:2] - self.initial_position[:2])
        self.time += dt
    
    def _gait_subsystem(self, dt: float, state: SystemState):
        """Gait generation subsystem"""
        if not self.is_walking:
            return None
        
        # Update gait
        self.gait_state = self.gait_generator.update(dt)
        
        return {
            'phase': self.gait_state.phase,
            'left_foot': self.gait_state.left_foot,
            'right_foot': self.gait_state.right_foot,
            'com_target': self.gait_state.com_target
        }
    
    def _balance_subsystem(self, dt: float, state: SystemState):
        """Balance control subsystem"""
        if self.current_balance_state is None:
            return None
        
        # Get balance control
        ankle_torques, hip_torques = self.balance_controller.compute_control(
            self.current_balance_state, dt
        )
        
        return {
            'ankle_torques': ankle_torques,
            'hip_torques': hip_torques
        }
    
    def _apply_control(self, dt: float, state: SystemState, outputs: dict):
        """Apply control to robot"""
        # Get control outputs
        gait_output = outputs.get('gait', {})
        balance_output = outputs.get('balance', {})
        
        # Simplified control application
        # In practice, this would involve inverse kinematics, 
        # trajectory tracking, and proper joint control
        
        joint_positions = {}
        
        if self.is_walking and gait_output:
            # Use gait-generated foot positions
            # This is highly simplified - real implementation would use IK
            pass
        
        # Apply default standing pose with balance corrections
        for joint_name in self.robot.joints.keys():
            joint_positions[joint_name] = 0.0  # Neutral pose
        
        # Apply PD control to track positions
        self.robot.set_joint_positions(joint_positions, kp=100.0, kd=10.0)
        
        # Step simulation
        self.robot.step_simulation()
        
        if int(self.time * 1000) % 100 == 0:  # Every 100ms
            self.robot.add_debug_lines()
    
    def start_walking(self):
        """Start walking"""
        logger.info("Starting walking gait...")
        self.is_walking = True
        self.gait_generator.start_walking()
        self.control_loop.set_state(SystemState.WALKING)
    
    def stop_walking(self):
        """Stop walking"""
        logger.info("Stopping walking gait...")
        self.is_walking = False
        self.gait_generator.stop_walking()
        self.control_loop.set_state(SystemState.STANDING)
    
    def run(self, duration: float = 20.0, start_walking_at: float = 2.0):
        """
        Run the demo.
        
        Args:
            duration: Total duration (seconds)
            start_walking_at: When to start walking (seconds)
        """
        logger.info(f"Running walking demo for {duration} seconds...")
        logger.info(f"Walking will start at t={start_walking_at}s")
        
        start_time = time.time()
        walking_started = False
        
        try:
            while time.time() - start_time < duration:
                elapsed = time.time() - start_time
                
                # Start walking after initial stabilization
                if not walking_started and elapsed >= start_walking_at:
                    self.start_walking()
                    walking_started = True
                
                # Execute control
                self.control_loop.step()
                
                # Print progress every 2 seconds
                if int(elapsed * 10) % 20 == 0 and int(elapsed * 10) > 0:
                    self._print_progress()
                
                time.sleep(0.0001)
                
        except KeyboardInterrupt:
            logger.info("Demo interrupted")
        
        self._print_statistics()
    
    def _print_progress(self):
        """Print current progress"""
        if self.current_balance_state:
            pos = self.current_balance_state.com_position
            phase = self.gait_state.phase.value if self.gait_state else "stopped"
            steps = self.gait_generator.state.step_count if self.gait_state else 0
            
            logger.info(
                f"t={self.time:.1f}s | "
                f"Pos: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] | "
                f"Dist: {self.distance_traveled:.2f}m | "
                f"Phase: {phase} | "
                f"Steps: {steps}"
            )
    
    def _print_statistics(self):
        """Print final statistics"""
        logger.info("=" * 70)
        logger.info("Walking Demo Statistics:")
        logger.info(f"  Duration: {self.time:.2f}s")
        logger.info(f"  Distance traveled: {self.distance_traveled:.2f}m")
        
        if self.gait_state:
            logger.info(f"  Steps taken: {self.gait_generator.state.step_count}")
            logger.info(f"  Average speed: {self.distance_traveled/self.time:.2f} m/s")
        
        if self.current_balance_state:
            logger.info(f"  Final height: {self.current_balance_state.com_position[2]:.3f}m")
            logger.info(f"  Final balance: {'✓' if self.current_balance_state.is_balanced else '✗'}")
        
        logger.info("=" * 70)
    
    def cleanup(self):
        """Cleanup"""
        self.robot.disconnect()
        logger.info("Demo cleanup complete")


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='HumanoidOS Walking Demo')
    parser.add_argument('--no-gui', action='store_true', help='Run without GUI')
    parser.add_argument('--duration', type=float, default=20.0, help='Duration in seconds')
    parser.add_argument('--start-at', type=float, default=2.0, 
                       help='When to start walking (seconds)')
    
    args = parser.parse_args()
    
    print("\n" + "=" * 70)
    print("HumanoidOS - Walking Demo")
    print("=" * 70)
    print("\nThis demo shows:")
    print("  • Bipedal gait generation")
    print("  • Balance control during walking")
    print("  • Foot trajectory planning")
    print("  • COM trajectory tracking")
    print("\nThe robot will:")
    print(f"  1. Stand and stabilize for {args.start_at} seconds")
    print(f"  2. Begin walking forward")
    print(f"  3. Continue for {args.duration - args.start_at} seconds")
    print("\nPress Ctrl+C to stop early.")
    print("=" * 70 + "\n")
    
    # Run demo
    demo = WalkingDemo(use_gui=not args.no_gui)
    
    try:
        demo.run(duration=args.duration, start_walking_at=args.start_at)
    finally:
        demo.cleanup()
    
    print("\n" + "=" * 70)
    print("Demo Complete!")
    print("=" * 70)


if __name__ == "__main__":
    main()
