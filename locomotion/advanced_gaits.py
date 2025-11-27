"""
Advanced Gait Patterns for HumanoidOS

Extends the basic gait generator with:
- Turning (left/right)
- Sideways walking (left/right)
- Backward walking
- Dynamic speed adjustment
- Smooth transitions between gaits
"""

import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class GaitType(Enum):
    """Types of gaits available"""
    STOPPED = "stopped"
    FORWARD_WALK = "forward_walk"
    BACKWARD_WALK = "backward_walk"
    SIDESTEP_LEFT = "sidestep_left"
    SIDESTEP_RIGHT = "sidestep_right"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    FORWARD_RUN = "forward_run"


@dataclass
class AdvancedGaitConfig:
    """Extended configuration for advanced gaits"""
    # Base parameters
    step_length: float = 0.2
    step_width: float = 0.15
    step_height: float = 0.05
    step_duration: float = 0.6
    
    # Turning parameters
    turn_angle_per_step: float = 15.0  # degrees
    turn_com_offset: float = 0.03  # lateral COM shift during turns
    
    # Sideways walking
    sidestep_length: float = 0.15  # lateral step distance
    sidestep_forward_offset: float = 0.02  # slight forward motion during sidestep
    
    # Running parameters
    run_step_length: float = 0.4
    run_step_height: float = 0.08
    run_step_duration: float = 0.35
    run_flight_phase_ratio: float = 0.3  # Time with no ground contact
    
    # Speed adjustment
    min_speed: float = 0.1  # m/s
    max_speed: float = 1.5  # m/s
    acceleration: float = 0.5  # m/s²
    
    # Transition parameters
    transition_steps: int = 2  # Number of steps to transition between gaits


class AdvancedGaitGenerator:
    """
    Advanced gait generator with multiple locomotion modes.
    
    Supports forward/backward walking, sidestepping, turning,
    and smooth transitions between different gaits.
    """
    
    def __init__(self, config: Optional[AdvancedGaitConfig] = None):
        self.config = config or AdvancedGaitConfig()
        
        # Current state
        self.current_gait = GaitType.STOPPED
        self.target_gait = GaitType.STOPPED
        self.transition_progress = 0
        
        # Movement state
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.current_heading = 0.0  # radians
        self.total_distance = 0.0
        self.total_rotation = 0.0
        
        # Foot positions
        self.left_foot_pos = np.array([0.0, self.config.step_width/2, 0.0])
        self.right_foot_pos = np.array([0.0, -self.config.step_width/2, 0.0])
        
        # Timing
        self.phase_time = 0.0
        self.step_count = 0
        self.is_left_swing = True
        
        logger.info("Advanced gait generator initialized")
    
    def set_gait(self, gait_type: GaitType, speed: Optional[float] = None):
        """
        Set target gait and speed.
        
        Args:
            gait_type: Desired gait type
            speed: Optional speed override (m/s)
        """
        if gait_type != self.current_gait:
            logger.info(f"Gait transition: {self.current_gait.value} -> {gait_type.value}")
            self.target_gait = gait_type
            self.transition_progress = 0
        
        if speed is not None:
            self.target_speed = np.clip(speed, self.config.min_speed, self.config.max_speed)
    
    def update(self, dt: float) -> dict:
        """
        Update gait state.
        
        Args:
            dt: Time step
            
        Returns:
            Dictionary with current gait state
        """
        # Update speed with acceleration limits
        speed_error = self.target_speed - self.current_speed
        speed_change = np.clip(
            speed_error,
            -self.config.acceleration * dt,
            self.config.acceleration * dt
        )
        self.current_speed += speed_change
        
        # Handle gait transitions
        if self.current_gait != self.target_gait:
            self._handle_transition()
        
        # Update phase timing
        self.phase_time += dt
        
        # Generate trajectories based on current gait
        if self.current_gait == GaitType.STOPPED:
            return self._generate_stopped()
        
        # Determine step duration based on gait
        step_duration = self._get_step_duration()
        
        # Check for phase transition
        if self.phase_time >= step_duration:
            self._next_step()
            self.phase_time = 0.0
        
        # Generate trajectories
        if self.current_gait == GaitType.FORWARD_WALK:
            return self._generate_forward_walk(dt)
        elif self.current_gait == GaitType.BACKWARD_WALK:
            return self._generate_backward_walk(dt)
        elif self.current_gait == GaitType.SIDESTEP_LEFT:
            return self._generate_sidestep_left(dt)
        elif self.current_gait == GaitType.SIDESTEP_RIGHT:
            return self._generate_sidestep_right(dt)
        elif self.current_gait == GaitType.TURN_LEFT:
            return self._generate_turn_left(dt)
        elif self.current_gait == GaitType.TURN_RIGHT:
            return self._generate_turn_right(dt)
        elif self.current_gait == GaitType.FORWARD_RUN:
            return self._generate_forward_run(dt)
        else:
            return self._generate_stopped()
    
    def _get_step_duration(self) -> float:
        """Get step duration for current gait"""
        if self.current_gait == GaitType.FORWARD_RUN:
            return self.config.run_step_duration
        else:
            # Scale duration with speed (faster = shorter steps)
            base_duration = self.config.step_duration
            speed_factor = self.current_speed / self.config.max_speed
            return base_duration * (1.0 - 0.3 * speed_factor)
    
    def _handle_transition(self):
        """Handle transition between gaits"""
        self.transition_progress += 1
        
        if self.transition_progress >= self.config.transition_steps:
            self.current_gait = self.target_gait
            self.transition_progress = 0
            logger.info(f"Gait transition complete: {self.current_gait.value}")
    
    def _next_step(self):
        """Advance to next step"""
        self.step_count += 1
        self.is_left_swing = not self.is_left_swing
    
    def _cycloid_trajectory(
        self,
        start_pos: np.ndarray,
        end_pos: np.ndarray,
        step_height: float,
        progress: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate smooth cycloid trajectory for foot.
        
        Args:
            start_pos: Starting position [x, y, z]
            end_pos: Ending position [x, y, z]
            step_height: Maximum height during swing
            progress: Progress through step [0, 1]
            
        Returns:
            (position, velocity)
        """
        # Horizontal interpolation
        horizontal_pos = start_pos[:2] + (end_pos[:2] - start_pos[:2]) * progress
        
        # Vertical cycloid
        z = step_height * np.sin(np.pi * progress)
        
        position = np.array([horizontal_pos[0], horizontal_pos[1], z])
        
        # Velocity
        horizontal_vel = (end_pos[:2] - start_pos[:2]) / self._get_step_duration()
        z_vel = step_height * np.pi * np.cos(np.pi * progress) / self._get_step_duration()
        
        velocity = np.array([horizontal_vel[0], horizontal_vel[1], z_vel])
        
        return position, velocity
    
    def _generate_forward_walk(self, dt: float) -> dict:
        """Generate forward walking trajectory"""
        progress = self.phase_time / self._get_step_duration()
        
        # Calculate step length based on current speed
        step_length = self.current_speed * self._get_step_duration()
        
        if self.is_left_swing:
            # Left foot swings
            start = self.left_foot_pos.copy()
            end = start + np.array([step_length, 0.0, 0.0])
            
            pos, vel = self._cycloid_trajectory(
                start, end, self.config.step_height, progress
            )
            
            self.left_foot_pos = pos if progress >= 1.0 else start
            
            return {
                'left_foot': {'position': pos, 'velocity': vel, 'contact': False},
                'right_foot': {'position': self.right_foot_pos, 'velocity': np.zeros(3), 'contact': True},
                'com_offset': np.array([0.0, self.config.step_width/4, 0.0])
            }
        else:
            # Right foot swings
            start = self.right_foot_pos.copy()
            end = start + np.array([step_length, 0.0, 0.0])
            
            pos, vel = self._cycloid_trajectory(
                start, end, self.config.step_height, progress
            )
            
            self.right_foot_pos = pos if progress >= 1.0 else start
            
            return {
                'left_foot': {'position': self.left_foot_pos, 'velocity': np.zeros(3), 'contact': True},
                'right_foot': {'position': pos, 'velocity': vel, 'contact': False},
                'com_offset': np.array([0.0, -self.config.step_width/4, 0.0])
            }
    
    def _generate_backward_walk(self, dt: float) -> dict:
        """Generate backward walking trajectory"""
        progress = self.phase_time / self._get_step_duration()
        
        # Negative step length for backward
        step_length = -self.current_speed * self._get_step_duration() * 0.7  # Slower backward
        
        if self.is_left_swing:
            start = self.left_foot_pos.copy()
            end = start + np.array([step_length, 0.0, 0.0])
            
            pos, vel = self._cycloid_trajectory(
                start, end, self.config.step_height * 0.8, progress
            )
            
            self.left_foot_pos = pos if progress >= 1.0 else start
            
            return {
                'left_foot': {'position': pos, 'velocity': vel, 'contact': False},
                'right_foot': {'position': self.right_foot_pos, 'velocity': np.zeros(3), 'contact': True},
                'com_offset': np.array([0.0, self.config.step_width/4, 0.0])
            }
        else:
            start = self.right_foot_pos.copy()
            end = start + np.array([step_length, 0.0, 0.0])
            
            pos, vel = self._cycloid_trajectory(
                start, end, self.config.step_height * 0.8, progress
            )
            
            self.right_foot_pos = pos if progress >= 1.0 else start
            
            return {
                'left_foot': {'position': self.left_foot_pos, 'velocity': np.zeros(3), 'contact': True},
                'right_foot': {'position': pos, 'velocity': vel, 'contact': False},
                'com_offset': np.array([0.0, -self.config.step_width/4, 0.0])
            }
    
    def _generate_sidestep_left(self, dt: float) -> dict:
        """Generate left sidestepping trajectory"""
        progress = self.phase_time / self._get_step_duration()
        
        if self.is_left_swing:
            start = self.left_foot_pos.copy()
            end = start + np.array([
                self.config.sidestep_forward_offset,
                self.config.sidestep_length,
                0.0
            ])
            
            pos, vel = self._cycloid_trajectory(
                start, end, self.config.step_height, progress
            )
            
            self.left_foot_pos = pos if progress >= 1.0 else start
            
            return {
                'left_foot': {'position': pos, 'velocity': vel, 'contact': False},
                'right_foot': {'position': self.right_foot_pos, 'velocity': np.zeros(3), 'contact': True},
                'com_offset': np.array([0.0, -self.config.turn_com_offset, 0.0])
            }
        else:
            # Right foot follows
            start = self.right_foot_pos.copy()
            end = start + np.array([
                self.config.sidestep_forward_offset,
                self.config.sidestep_length * 0.8,
                0.0
            ])
            
            pos, vel = self._cycloid_trajectory(
                start, end, self.config.step_height, progress
            )
            
            self.right_foot_pos = pos if progress >= 1.0 else start
            
            return {
                'left_foot': {'position': self.left_foot_pos, 'velocity': np.zeros(3), 'contact': True},
                'right_foot': {'position': pos, 'velocity': vel, 'contact': False},
                'com_offset': np.array([0.0, self.config.turn_com_offset, 0.0])
            }
    
    def _generate_sidestep_right(self, dt: float) -> dict:
        """Generate right sidestepping trajectory"""
        progress = self.phase_time / self._get_step_duration()
        
        if self.is_left_swing:
            # Left foot follows
            start = self.left_foot_pos.copy()
            end = start + np.array([
                self.config.sidestep_forward_offset,
                -self.config.sidestep_length * 0.8,
                0.0
            ])
            
            pos, vel = self._cycloid_trajectory(
                start, end, self.config.step_height, progress
            )
            
            self.left_foot_pos = pos if progress >= 1.0 else start
            
            return {
                'left_foot': {'position': pos, 'velocity': vel, 'contact': False},
                'right_foot': {'position': self.right_foot_pos, 'velocity': np.zeros(3), 'contact': True},
                'com_offset': np.array([0.0, -self.config.turn_com_offset, 0.0])
            }
        else:
            start = self.right_foot_pos.copy()
            end = start + np.array([
                self.config.sidestep_forward_offset,
                -self.config.sidestep_length,
                0.0
            ])
            
            pos, vel = self._cycloid_trajectory(
                start, end, self.config.step_height, progress
            )
            
            self.right_foot_pos = pos if progress >= 1.0 else start
            
            return {
                'left_foot': {'position': self.left_foot_pos, 'velocity': np.zeros(3), 'contact': True},
                'right_foot': {'position': pos, 'velocity': vel, 'contact': False},
                'com_offset': np.array([0.0, self.config.turn_com_offset, 0.0])
            }
    
    def _generate_turn_left(self, dt: float) -> dict:
        """Generate left turning trajectory"""
        progress = self.phase_time / self._get_step_duration()
        
        # Calculate arc for turning
        angle_rad = np.radians(self.config.turn_angle_per_step)
        
        if self.is_left_swing:
            # Left foot steps forward and outward
            start = self.left_foot_pos.copy()
            forward = self.config.step_length * 0.6
            lateral = self.config.step_width * 0.3
            
            end = start + np.array([forward, lateral, 0.0])
            
            pos, vel = self._cycloid_trajectory(
                start, end, self.config.step_height, progress
            )
            
            self.left_foot_pos = pos if progress >= 1.0 else start
            self.current_heading += angle_rad / 2 if progress >= 1.0 else 0
            
            return {
                'left_foot': {'position': pos, 'velocity': vel, 'contact': False},
                'right_foot': {'position': self.right_foot_pos, 'velocity': np.zeros(3), 'contact': True},
                'com_offset': np.array([0.0, -self.config.turn_com_offset, 0.0]),
                'heading': self.current_heading
            }
        else:
            # Right foot steps to align
            start = self.right_foot_pos.copy()
            forward = self.config.step_length * 0.4
            lateral = self.config.step_width * 0.2
            
            end = start + np.array([forward, lateral, 0.0])
            
            pos, vel = self._cycloid_trajectory(
                start, end, self.config.step_height, progress
            )
            
            self.right_foot_pos = pos if progress >= 1.0 else start
            self.current_heading += angle_rad / 2 if progress >= 1.0 else 0
            
            return {
                'left_foot': {'position': self.left_foot_pos, 'velocity': np.zeros(3), 'contact': True},
                'right_foot': {'position': pos, 'velocity': vel, 'contact': False},
                'com_offset': np.array([0.0, self.config.turn_com_offset, 0.0]),
                'heading': self.current_heading
            }
    
    def _generate_turn_right(self, dt: float) -> dict:
        """Generate right turning trajectory"""
        progress = self.phase_time / self._get_step_duration()
        
        angle_rad = np.radians(self.config.turn_angle_per_step)
        
        if self.is_left_swing:
            # Left foot steps to align
            start = self.left_foot_pos.copy()
            forward = self.config.step_length * 0.4
            lateral = -self.config.step_width * 0.2
            
            end = start + np.array([forward, lateral, 0.0])
            
            pos, vel = self._cycloid_trajectory(
                start, end, self.config.step_height, progress
            )
            
            self.left_foot_pos = pos if progress >= 1.0 else start
            self.current_heading -= angle_rad / 2 if progress >= 1.0 else 0
            
            return {
                'left_foot': {'position': pos, 'velocity': vel, 'contact': False},
                'right_foot': {'position': self.right_foot_pos, 'velocity': np.zeros(3), 'contact': True},
                'com_offset': np.array([0.0, self.config.turn_com_offset, 0.0]),
                'heading': self.current_heading
            }
        else:
            # Right foot steps forward and inward
            start = self.right_foot_pos.copy()
            forward = self.config.step_length * 0.6
            lateral = -self.config.step_width * 0.3
            
            end = start + np.array([forward, lateral, 0.0])
            
            pos, vel = self._cycloid_trajectory(
                start, end, self.config.step_height, progress
            )
            
            self.right_foot_pos = pos if progress >= 1.0 else start
            self.current_heading -= angle_rad / 2 if progress >= 1.0 else 0
            
            return {
                'left_foot': {'position': self.left_foot_pos, 'velocity': np.zeros(3), 'contact': True},
                'right_foot': {'position': pos, 'velocity': vel, 'contact': False},
                'com_offset': np.array([0.0, -self.config.turn_com_offset, 0.0]),
                'heading': self.current_heading
            }
    
    def _generate_forward_run(self, dt: float) -> dict:
        """Generate running trajectory with flight phase"""
        progress = self.phase_time / self._get_step_duration()
        
        step_length = self.current_speed * self._get_step_duration()
        
        # Check if in flight phase (both feet off ground)
        flight_start = 0.3
        flight_end = 0.3 + self.config.run_flight_phase_ratio
        in_flight = flight_start < progress < flight_end
        
        if self.is_left_swing:
            start = self.left_foot_pos.copy()
            end = start + np.array([step_length, 0.0, 0.0])
            
            pos, vel = self._cycloid_trajectory(
                start, end, self.config.run_step_height, progress
            )
            
            self.left_foot_pos = pos if progress >= 1.0 else start
            
            return {
                'left_foot': {'position': pos, 'velocity': vel, 'contact': not in_flight and progress > flight_end},
                'right_foot': {'position': self.right_foot_pos, 'velocity': np.zeros(3), 'contact': not in_flight},
                'com_offset': np.array([0.0, self.config.step_width/4, 0.0]),
                'is_flight': in_flight
            }
        else:
            start = self.right_foot_pos.copy()
            end = start + np.array([step_length, 0.0, 0.0])
            
            pos, vel = self._cycloid_trajectory(
                start, end, self.config.run_step_height, progress
            )
            
            self.right_foot_pos = pos if progress >= 1.0 else start
            
            return {
                'left_foot': {'position': self.left_foot_pos, 'velocity': np.zeros(3), 'contact': not in_flight},
                'right_foot': {'position': pos, 'velocity': vel, 'contact': not in_flight and progress > flight_end},
                'com_offset': np.array([0.0, -self.config.step_width/4, 0.0]),
                'is_flight': in_flight
            }
    
    def _generate_stopped(self) -> dict:
        """Generate stopped state"""
        return {
            'left_foot': {'position': self.left_foot_pos, 'velocity': np.zeros(3), 'contact': True},
            'right_foot': {'position': self.right_foot_pos, 'velocity': np.zeros(3), 'contact': True},
            'com_offset': np.zeros(3)
        }


if __name__ == "__main__":
    # Test advanced gaits
    logging.basicConfig(level=logging.INFO)
    
    config = AdvancedGaitConfig()
    gait = AdvancedGaitGenerator(config)
    
    print("Testing advanced gait patterns...")
    
    # Test forward walk
    gait.set_gait(GaitType.FORWARD_WALK, speed=0.5)
    for _ in range(10):
        state = gait.update(0.01)
    
    # Test turning
    gait.set_gait(GaitType.TURN_LEFT, speed=0.3)
    for _ in range(10):
        state = gait.update(0.01)
    
    print(f"Current heading: {np.degrees(gait.current_heading):.1f}°")
    print("Advanced gait test complete!")
