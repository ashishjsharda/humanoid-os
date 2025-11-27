"""
Push Recovery System for HumanoidOS

Implements disturbance detection and recovery stepping to prevent falls.
Uses capture point dynamics for robust balance recovery.
"""

import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class RecoveryState(Enum):
    """Recovery states"""
    STABLE = "stable"
    DETECTING = "detecting"
    RECOVERING = "recovering"
    CRITICAL = "critical"


@dataclass
class DisturbanceConfig:
    """Configuration for disturbance detection"""
    # Thresholds for disturbance detection
    com_velocity_threshold: float = 0.5  # m/s
    com_acceleration_threshold: float = 2.0  # m/s²
    angular_velocity_threshold: float = 1.0  # rad/s
    zmp_distance_threshold: float = 0.06  # meters from support center
    
    # Capture point parameters
    gravity: float = 9.81
    com_height: float = 0.9  # meters
    
    # Recovery parameters
    max_recovery_steps: int = 3
    recovery_step_length_factor: float = 1.5  # Multiplier for recovery step length
    recovery_step_speed: float = 0.4  # seconds per recovery step
    
    # Safety margins
    min_stability_margin: float = 0.02  # meters
    critical_stability_margin: float = 0.01  # meters


@dataclass
class DisturbanceInfo:
    """Information about detected disturbance"""
    magnitude: float  # Severity of disturbance [0, 1]
    direction: np.ndarray  # Direction of disturbance (2D)
    type: str  # Type of disturbance (push, pull, lateral, etc.)
    timestamp: float
    recovery_required: bool


class PushRecoveryController:
    """
    Push recovery controller using capture point dynamics.
    
    Detects external disturbances and generates recovery stepping
    motions to prevent falling.
    """
    
    def __init__(self, config: Optional[DisturbanceConfig] = None):
        self.config = config or DisturbanceConfig()
        
        # State
        self.recovery_state = RecoveryState.STABLE
        self.disturbance_history: List[DisturbanceInfo] = []
        self.recovery_steps_taken = 0
        self.recovery_target: Optional[np.ndarray] = None
        
        # Capture point dynamics
        self.omega = np.sqrt(self.config.gravity / self.config.com_height)
        
        logger.info("Push recovery controller initialized")
    
    def detect_disturbance(
        self,
        com_position: np.ndarray,
        com_velocity: np.ndarray,
        com_acceleration: np.ndarray,
        angular_velocity: np.ndarray,
        zmp: np.ndarray,
        support_polygon_center: np.ndarray,
        timestamp: float
    ) -> Optional[DisturbanceInfo]:
        """
        Detect if robot is experiencing a disturbance.
        
        Args:
            com_position: Center of mass position [x, y, z]
            com_velocity: COM velocity [vx, vy, vz]
            com_acceleration: COM acceleration [ax, ay, az]
            angular_velocity: Angular velocity [wx, wy, wz]
            zmp: Zero moment point [x, y, z]
            support_polygon_center: Center of support polygon [x, y]
            timestamp: Current time
            
        Returns:
            DisturbanceInfo if disturbance detected, None otherwise
        """
        # Check multiple indicators
        disturbance_indicators = []
        
        # 1. High COM velocity
        com_vel_magnitude = np.linalg.norm(com_velocity[:2])
        if com_vel_magnitude > self.config.com_velocity_threshold:
            disturbance_indicators.append(('velocity', com_vel_magnitude / (self.config.com_velocity_threshold * 2)))
        
        # 2. High COM acceleration
        com_acc_magnitude = np.linalg.norm(com_acceleration[:2])
        if com_acc_magnitude > self.config.com_acceleration_threshold:
            disturbance_indicators.append(('acceleration', com_acc_magnitude / (self.config.com_acceleration_threshold * 2)))
        
        # 3. High angular velocity
        ang_vel_magnitude = np.linalg.norm(angular_velocity[:2])
        if ang_vel_magnitude > self.config.angular_velocity_threshold:
            disturbance_indicators.append(('angular', ang_vel_magnitude / (self.config.angular_velocity_threshold * 2)))
        
        # 4. ZMP near edge of support
        zmp_offset = zmp[:2] - support_polygon_center
        zmp_distance = np.linalg.norm(zmp_offset)
        if zmp_distance > self.config.zmp_distance_threshold:
            disturbance_indicators.append(('zmp', zmp_distance / (self.config.zmp_distance_threshold * 2)))
        
        # If no indicators, no disturbance
        if not disturbance_indicators:
            if self.recovery_state == RecoveryState.DETECTING:
                self.recovery_state = RecoveryState.STABLE
            return None
        
        # Calculate overall disturbance magnitude
        magnitude = min(1.0, np.mean([score for _, score in disturbance_indicators]))
        
        # Determine disturbance direction
        # Use COM velocity as primary indicator
        direction = com_velocity[:2].copy()
        if np.linalg.norm(direction) < 0.01:
            # If no velocity, use ZMP offset
            direction = zmp_offset
        
        direction = direction / (np.linalg.norm(direction) + 1e-6)
        
        # Classify disturbance type
        if abs(direction[0]) > abs(direction[1]):
            dist_type = "forward_push" if direction[0] > 0 else "backward_push"
        else:
            dist_type = "left_push" if direction[1] > 0 else "right_push"
        
        # Determine if recovery is required
        recovery_required = magnitude > 0.5
        
        disturbance = DisturbanceInfo(
            magnitude=magnitude,
            direction=direction,
            type=dist_type,
            timestamp=timestamp,
            recovery_required=recovery_required
        )
        
        # Update state
        if recovery_required:
            if self.recovery_state == RecoveryState.STABLE:
                self.recovery_state = RecoveryState.DETECTING
                logger.warning(f"Disturbance detected: {dist_type}, magnitude: {magnitude:.2f}")
        
        self.disturbance_history.append(disturbance)
        
        # Keep only recent history
        if len(self.disturbance_history) > 100:
            self.disturbance_history.pop(0)
        
        return disturbance
    
    def compute_capture_point(
        self,
        com_position: np.ndarray,
        com_velocity: np.ndarray
    ) -> np.ndarray:
        """
        Compute the capture point (instantaneous capture point).
        
        The capture point is the point where the robot must step to
        in order to come to a complete stop.
        
        Args:
            com_position: COM position [x, y, z]
            com_velocity: COM velocity [vx, vy, vz]
            
        Returns:
            Capture point position [x, y]
        """
        # ICP = COM_xy + COM_velocity_xy / omega
        capture_point = com_position[:2] + com_velocity[:2] / self.omega
        return capture_point
    
    def plan_recovery_step(
        self,
        disturbance: DisturbanceInfo,
        com_position: np.ndarray,
        com_velocity: np.ndarray,
        current_support_foot: str,  # "left" or "right"
        current_foot_positions: Tuple[np.ndarray, np.ndarray]  # (left, right)
    ) -> Tuple[np.ndarray, str]:
        """
        Plan a recovery step to counteract disturbance.
        
        Args:
            disturbance: Detected disturbance information
            com_position: Current COM position
            com_velocity: Current COM velocity
            current_support_foot: Which foot is currently supporting
            current_foot_positions: (left_foot_pos, right_foot_pos)
            
        Returns:
            (target_foot_position, stepping_foot) - where to step and which foot
        """
        # Compute capture point
        capture_point = self.compute_capture_point(com_position, com_velocity)
        
        # Determine which foot should step
        left_pos, right_pos = current_foot_positions
        
        if current_support_foot == "left":
            stepping_foot = "right"
            support_pos = left_pos[:2]
        else:
            stepping_foot = "left"
            support_pos = right_pos[:2]
        
        # Calculate target step location
        # Step towards the capture point, scaled by disturbance magnitude
        step_vector = (capture_point - support_pos) * self.config.recovery_step_length_factor
        
        # Limit step length
        max_step_length = 0.4  # meters
        step_length = np.linalg.norm(step_vector)
        if step_length > max_step_length:
            step_vector = step_vector / step_length * max_step_length
        
        # Add some lateral offset for stability
        lateral_offset = 0.08 if stepping_foot == "left" else -0.08
        
        target_position = np.array([
            support_pos[0] + step_vector[0],
            support_pos[1] + step_vector[1] + lateral_offset,
            0.0
        ])
        
        logger.info(f"Recovery step planned: {stepping_foot} foot to {target_position[:2]}")
        
        return target_position, stepping_foot
    
    def should_trigger_recovery(
        self,
        stability_margin: float
    ) -> bool:
        """
        Determine if recovery stepping should be triggered.
        
        Args:
            stability_margin: Current stability margin
            
        Returns:
            True if recovery should be triggered
        """
        # Check if we have recent disturbances
        if not self.disturbance_history:
            return False
        
        recent_disturbance = self.disturbance_history[-1]
        
        # Trigger if:
        # 1. Disturbance magnitude is high AND
        # 2. Stability margin is low OR recovery already in progress
        
        if recent_disturbance.magnitude > 0.6:
            if stability_margin < self.config.min_stability_margin:
                return True
            
            if self.recovery_state in [RecoveryState.RECOVERING, RecoveryState.CRITICAL]:
                return True
        
        return False
    
    def execute_recovery_step(
        self,
        target_position: np.ndarray,
        stepping_foot: str,
        current_time: float
    ) -> dict:
        """
        Generate recovery step trajectory.
        
        Args:
            target_position: Where to place the stepping foot
            stepping_foot: Which foot is stepping ("left" or "right")
            current_time: Current simulation time
            
        Returns:
            Recovery step trajectory information
        """
        if self.recovery_state != RecoveryState.RECOVERING:
            self.recovery_state = RecoveryState.RECOVERING
            self.recovery_steps_taken += 1
            logger.info(f"Executing recovery step {self.recovery_steps_taken}")
        
        return {
            'target_position': target_position,
            'stepping_foot': stepping_foot,
            'step_duration': self.config.recovery_step_speed,
            'step_height': 0.08,  # Higher step for faster recovery
            'urgency': 'high'
        }
    
    def assess_recovery_success(
        self,
        com_position: np.ndarray,
        com_velocity: np.ndarray,
        stability_margin: float
    ) -> bool:
        """
        Check if recovery was successful.
        
        Args:
            com_position: Current COM position
            com_velocity: Current COM velocity
            stability_margin: Current stability margin
            
        Returns:
            True if robot has successfully recovered
        """
        # Check if robot is now stable
        com_vel_magnitude = np.linalg.norm(com_velocity[:2])
        
        if (com_vel_magnitude < self.config.com_velocity_threshold * 0.3 and
            stability_margin > self.config.min_stability_margin * 1.5):
            
            logger.info(f"Recovery successful after {self.recovery_steps_taken} steps")
            self.recovery_state = RecoveryState.STABLE
            self.recovery_steps_taken = 0
            return True
        
        # Check if we've taken too many recovery steps
        if self.recovery_steps_taken >= self.config.max_recovery_steps:
            logger.critical("Maximum recovery steps reached - stability not achieved")
            self.recovery_state = RecoveryState.CRITICAL
            return False
        
        return False
    
    def get_state(self) -> dict:
        """Get current recovery state"""
        return {
            'state': self.recovery_state,
            'steps_taken': self.recovery_steps_taken,
            'recent_disturbances': len([d for d in self.disturbance_history 
                                       if d.recovery_required])
        }
    
    def reset(self):
        """Reset recovery controller"""
        self.recovery_state = RecoveryState.STABLE
        self.recovery_steps_taken = 0
        self.recovery_target = None
        self.disturbance_history.clear()
        logger.info("Push recovery controller reset")


if __name__ == "__main__":
    # Test push recovery
    logging.basicConfig(level=logging.INFO)
    
    config = DisturbanceConfig()
    controller = PushRecoveryController(config)
    
    print("Testing push recovery system...")
    
    # Simulate a forward push
    com_pos = np.array([0.0, 0.0, 0.9])
    com_vel = np.array([0.6, 0.0, 0.0])  # Fast forward velocity
    com_acc = np.array([3.0, 0.0, 0.0])  # High acceleration
    ang_vel = np.zeros(3)
    zmp = np.array([0.08, 0.0, 0.0])  # ZMP shifted forward
    support_center = np.zeros(2)
    
    # Detect disturbance
    disturbance = controller.detect_disturbance(
        com_pos, com_vel, com_acc, ang_vel, zmp, support_center, 0.0
    )
    
    if disturbance:
        print(f"\nDisturbance detected!")
        print(f"  Type: {disturbance.type}")
        print(f"  Magnitude: {disturbance.magnitude:.2f}")
        print(f"  Direction: {disturbance.direction}")
        print(f"  Recovery required: {disturbance.recovery_required}")
        
        # Plan recovery step
        left_foot = np.array([0.0, 0.1, 0.0])
        right_foot = np.array([0.0, -0.1, 0.0])
        
        target, foot = controller.plan_recovery_step(
            disturbance,
            com_pos,
            com_vel,
            "left",
            (left_foot, right_foot)
        )
        
        print(f"\nRecovery step:")
        print(f"  Foot: {foot}")
        print(f"  Target: {target}")
    
    print("\nPush recovery test complete!")
