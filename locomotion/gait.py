"""
Gait Generator for Bipedal Walking

Implements trajectory generation for walking gaits using ZMP planning.
"""

import numpy as np
from typing import Tuple, List, Optional
from dataclasses import dataclass
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class GaitPhase(Enum):
    """Walking gait phases"""
    DOUBLE_SUPPORT = "double_support"  # Both feet on ground
    LEFT_SWING = "left_swing"          # Left foot in air
    RIGHT_SWING = "right_swing"        # Right foot in air
    STOPPED = "stopped"


@dataclass
class GaitConfig:
    """Configuration for gait generation"""
    # Step parameters
    step_length: float = 0.2  # meters
    step_width: float = 0.15  # meters (lateral separation)
    step_height: float = 0.05  # meters (foot lift height)
    step_duration: float = 0.6  # seconds per step
    double_support_ratio: float = 0.2  # Fraction of time in double support
    
    # Walking speed
    desired_velocity: float = 0.3  # m/s
    
    # COM trajectory
    com_height: float = 0.9  # meters
    com_lateral_sway: float = 0.02  # meters
    
    # Safety margins
    min_step_length: float = 0.05
    max_step_length: float = 0.4
    min_step_duration: float = 0.4
    max_step_duration: float = 1.0


@dataclass
class FootTrajectory:
    """Trajectory for a single foot"""
    position: np.ndarray  # [x, y, z]
    velocity: np.ndarray  # [vx, vy, vz]
    orientation: np.ndarray  # [roll, pitch, yaw]
    is_contact: bool  # Whether foot should be in contact with ground


@dataclass
class GaitState:
    """Current state of the gait"""
    phase: GaitPhase
    phase_time: float  # Time in current phase
    step_count: int
    
    # Foot states
    left_foot: FootTrajectory
    right_foot: FootTrajectory
    
    # COM trajectory
    com_target: np.ndarray
    com_velocity_target: np.ndarray


class GaitGenerator:
    """
    Generates walking trajectories for bipedal locomotion.
    
    Uses cycloid foot trajectories and sinusoidal COM motion
    for stable walking.
    """
    
    def __init__(self, config: Optional[GaitConfig] = None):
        self.config = config or GaitConfig()
        
        # Current state
        self.state = GaitState(
            phase=GaitPhase.STOPPED,
            phase_time=0.0,
            step_count=0,
            left_foot=FootTrajectory(
                position=np.array([0.0, self.config.step_width/2, 0.0]),
                velocity=np.zeros(3),
                orientation=np.zeros(3),
                is_contact=True
            ),
            right_foot=FootTrajectory(
                position=np.array([0.0, -self.config.step_width/2, 0.0]),
                velocity=np.zeros(3),
                orientation=np.zeros(3),
                is_contact=True
            ),
            com_target=np.array([0.0, 0.0, self.config.com_height]),
            com_velocity_target=np.zeros(3)
        )
        
        # Phase timing
        self.single_support_duration = (
            self.config.step_duration * (1 - self.config.double_support_ratio)
        )
        self.double_support_duration = (
            self.config.step_duration * self.config.double_support_ratio
        )
        
        logger.info("Gait generator initialized")
    
    def start_walking(self):
        """Initiate walking gait"""
        if self.state.phase == GaitPhase.STOPPED:
            self.state.phase = GaitPhase.DOUBLE_SUPPORT
            self.state.phase_time = 0.0
            logger.info("Walking started")
    
    def stop_walking(self):
        """Stop walking and return to stable stance"""
        self.state.phase = GaitPhase.STOPPED
        logger.info("Walking stopped")
    
    def update(self, dt: float) -> GaitState:
        """
        Update gait state for current timestep.
        
        Args:
            dt: Time step (seconds)
            
        Returns:
            Updated gait state with target trajectories
        """
        if self.state.phase == GaitPhase.STOPPED:
            return self.state
        
        self.state.phase_time += dt
        
        # Phase transitions
        if self.state.phase == GaitPhase.DOUBLE_SUPPORT:
            if self.state.phase_time >= self.double_support_duration:
                # Alternate between left and right swing
                if self.state.step_count % 2 == 0:
                    self.state.phase = GaitPhase.RIGHT_SWING
                else:
                    self.state.phase = GaitPhase.LEFT_SWING
                self.state.phase_time = 0.0
                self.state.step_count += 1
        
        elif self.state.phase in [GaitPhase.LEFT_SWING, GaitPhase.RIGHT_SWING]:
            if self.state.phase_time >= self.single_support_duration:
                self.state.phase = GaitPhase.DOUBLE_SUPPORT
                self.state.phase_time = 0.0
        
        # Generate trajectories based on current phase
        self._generate_foot_trajectories()
        self._generate_com_trajectory()
        
        return self.state
    
    def _generate_foot_trajectories(self):
        """Generate foot trajectories based on current phase"""
        t = self.state.phase_time
        
        if self.state.phase == GaitPhase.DOUBLE_SUPPORT:
            # Both feet stationary on ground
            self.state.left_foot.is_contact = True
            self.state.right_foot.is_contact = True
            self.state.left_foot.velocity = np.zeros(3)
            self.state.right_foot.velocity = np.zeros(3)
        
        elif self.state.phase == GaitPhase.RIGHT_SWING:
            # Right foot swings forward
            self.state.left_foot.is_contact = True
            self.state.right_foot.is_contact = False
            
            # Use cycloid trajectory for smooth foot motion
            s = t / self.single_support_duration  # Normalized time [0, 1]
            
            # Forward motion
            x_start = self.state.right_foot.position[0]
            x_end = x_start + self.config.step_length
            x = x_start + (x_end - x_start) * s
            
            # Vertical motion (cycloid)
            z = self.config.step_height * np.sin(np.pi * s)
            
            # Lateral position (constant)
            y = -self.config.step_width / 2
            
            self.state.right_foot.position = np.array([x, y, z])
            
            # Velocity
            vx = self.config.step_length / self.single_support_duration
            vz = (self.config.step_height * np.pi / self.single_support_duration) * np.cos(np.pi * s)
            self.state.right_foot.velocity = np.array([vx, 0.0, vz])
        
        elif self.state.phase == GaitPhase.LEFT_SWING:
            # Left foot swings forward
            self.state.left_foot.is_contact = False
            self.state.right_foot.is_contact = True
            
            s = t / self.single_support_duration
            
            x_start = self.state.left_foot.position[0]
            x_end = x_start + self.config.step_length
            x = x_start + (x_end - x_start) * s
            
            z = self.config.step_height * np.sin(np.pi * s)
            y = self.config.step_width / 2
            
            self.state.left_foot.position = np.array([x, y, z])
            
            vx = self.config.step_length / self.single_support_duration
            vz = (self.config.step_height * np.pi / self.single_support_duration) * np.cos(np.pi * s)
            self.state.left_foot.velocity = np.array([vx, 0.0, vz])
    
    def _generate_com_trajectory(self):
        """Generate COM trajectory for stable walking"""
        # COM moves forward with walking
        forward_offset = self.state.step_count * self.config.step_length / 2
        
        # Lateral sway for weight shift
        if self.state.phase == GaitPhase.RIGHT_SWING:
            # Weight on left foot
            lateral_offset = self.config.com_lateral_sway
        elif self.state.phase == GaitPhase.LEFT_SWING:
            # Weight on right foot
            lateral_offset = -self.config.com_lateral_sway
        else:
            # Centered during double support
            lateral_offset = 0.0
        
        # Smooth transition using sinusoid
        if self.state.phase != GaitPhase.STOPPED:
            t = self.state.phase_time
            duration = (self.double_support_duration if self.state.phase == GaitPhase.DOUBLE_SUPPORT 
                       else self.single_support_duration)
            s = t / duration if duration > 0 else 0
            lateral_offset *= np.sin(np.pi * s)
        
        self.state.com_target = np.array([
            forward_offset,
            lateral_offset,
            self.config.com_height
        ])
        
        self.state.com_velocity_target = np.array([
            self.config.desired_velocity,
            0.0,
            0.0
        ])
    
    def get_foot_targets(self) -> Tuple[FootTrajectory, FootTrajectory]:
        """Get current foot trajectory targets"""
        return self.state.left_foot, self.state.right_foot
    
    def get_com_target(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get current COM target (position, velocity)"""
        return self.state.com_target, self.state.com_velocity_target
    
    def is_walking(self) -> bool:
        """Check if currently walking"""
        return self.state.phase != GaitPhase.STOPPED


if __name__ == "__main__":
    # Simple test
    logging.basicConfig(level=logging.INFO)
    
    config = GaitConfig(
        step_length=0.3,
        step_duration=0.8,
        desired_velocity=0.375
    )
    
    gait = GaitGenerator(config)
    gait.start_walking()
    
    # Simulate for a few steps
    dt = 0.01  # 10ms
    total_time = 0.0
    max_time = 5.0  # 5 seconds
    
    print("Starting walking simulation...")
    print(f"{'Time':<8} {'Phase':<20} {'Left Foot Z':<12} {'Right Foot Z':<12} {'COM X':<10}")
    print("-" * 70)
    
    while total_time < max_time:
        state = gait.update(dt)
        
        if int(total_time * 100) % 50 == 0:  # Print every 0.5s
            print(f"{total_time:<8.2f} {state.phase.value:<20} "
                  f"{state.left_foot.position[2]:<12.3f} "
                  f"{state.right_foot.position[2]:<12.3f} "
                  f"{state.com_target[0]:<10.3f}")
        
        total_time += dt
    
    print(f"\nCompleted {gait.state.step_count} steps")
