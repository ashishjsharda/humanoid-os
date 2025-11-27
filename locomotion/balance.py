"""
Bipedal Balance Controller

Implements Zero Moment Point (ZMP) based balance control for humanoid robots.
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class BalanceConfig:
    """Configuration for balance controller"""
    # Physical parameters
    robot_mass: float = 50.0  # kg
    robot_height: float = 1.5  # meters (approximate COM height)
    foot_length: float = 0.25  # meters
    foot_width: float = 0.15  # meters
    
    # Control gains
    kp_position: np.ndarray = None  # Position gain
    kd_position: np.ndarray = None  # Velocity gain
    kp_orientation: float = 20.0
    kd_orientation: float = 5.0
    
    # Limits
    max_com_offset: float = 0.1  # meters
    max_ankle_torque: float = 100.0  # Nm
    
    def __post_init__(self):
        if self.kp_position is None:
            self.kp_position = np.array([100.0, 100.0, 50.0])
        if self.kd_position is None:
            self.kd_position = np.array([20.0, 20.0, 10.0])


@dataclass
class BalanceState:
    """Current balance state"""
    # Center of Mass (COM) state
    com_position: np.ndarray  # [x, y, z]
    com_velocity: np.ndarray  # [vx, vy, vz]
    com_acceleration: np.ndarray  # [ax, ay, az]
    
    # Body orientation (RPY - Roll, Pitch, Yaw)
    orientation: np.ndarray  # [roll, pitch, yaw]
    angular_velocity: np.ndarray  # [wx, wy, wz]
    
    # Zero Moment Point
    zmp: np.ndarray  # [x, y]
    
    # Support polygon (foot contact)
    left_foot_contact: bool = True
    right_foot_contact: bool = True
    
    @property
    def is_balanced(self) -> bool:
        """Check if robot is balanced (ZMP inside support polygon)"""
        # Simplified check - should be more sophisticated in production
        if not (self.left_foot_contact or self.right_foot_contact):
            return False
        
        # Check if ZMP is within reasonable bounds
        max_offset = 0.08  # 8cm from center
        return np.linalg.norm(self.zmp[:2]) < max_offset
    
    @property
    def stability_margin(self) -> float:
        """Calculate distance from ZMP to support polygon edge"""
        # Simplified - distance to center
        return max(0.0, 0.08 - np.linalg.norm(self.zmp[:2]))


class BalanceController:
    """
    ZMP-based balance controller for bipedal robots.
    
    Maintains upright stance and prevents falling by controlling
    the Zero Moment Point within the support polygon.
    """
    
    def __init__(self, config: Optional[BalanceConfig] = None):
        self.config = config or BalanceConfig()
        self.gravity = np.array([0.0, 0.0, -9.81])
        
        # Target state (upright, stationary)
        self.target_com_height = self.config.robot_height * 0.6  # Approx COM height
        self.target_orientation = np.zeros(3)  # Level orientation
        
        logger.info("Balance controller initialized")
    
    def compute_zmp(self, state: BalanceState) -> np.ndarray:
        """
        Compute Zero Moment Point from current state.
        
        The ZMP is the point on the ground where the total moment
        of ground reaction forces equals zero.
        """
        # Simplified ZMP calculation using COM dynamics
        # ZMP = COM_xy - (COM_z / g) * COM_acceleration_xy
        
        com_height = state.com_position[2]
        if com_height <= 0.01:
            com_height = 0.01  # Avoid division by zero
        
        zmp_x = state.com_position[0] - (com_height / abs(self.gravity[2])) * state.com_acceleration[0]
        zmp_y = state.com_position[1] - (com_height / abs(self.gravity[2])) * state.com_acceleration[1]
        
        return np.array([zmp_x, zmp_y, 0.0])
    
    def compute_control(
        self, 
        state: BalanceState, 
        dt: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute control torques to maintain balance.
        
        Args:
            state: Current balance state
            dt: Time step
            
        Returns:
            (ankle_torques, hip_torques) - Control torques for legs
        """
        # Update ZMP in state
        state.zmp = self.compute_zmp(state)
        
        # COM position control
        com_error = np.array([
            state.com_position[0],  # Should be near 0
            state.com_position[1],  # Should be near 0
            state.com_position[2] - self.target_com_height
        ])
        
        com_vel_error = state.com_velocity
        
        # PD control for COM
        com_force = (
            -self.config.kp_position * com_error -
            self.config.kd_position * com_vel_error
        )
        
        # Orientation control (keep upright)
        orientation_error = state.orientation - self.target_orientation
        angular_vel_error = state.angular_velocity
        
        # PD control for orientation
        orientation_torque = (
            -self.config.kp_orientation * orientation_error -
            self.config.kd_orientation * angular_vel_error
        )
        
        # Convert forces/torques to joint torques
        # Simplified model: ankle torques for ZMP control, hip for orientation
        
        # Ankle torques (control ZMP position)
        ankle_torques = np.array([
            orientation_torque[0],  # Roll
            orientation_torque[1],  # Pitch
        ])
        
        # Clip to limits
        ankle_torques = np.clip(
            ankle_torques,
            -self.config.max_ankle_torque,
            self.config.max_ankle_torque
        )
        
        # Hip torques (maintain posture)
        hip_torques = com_force[:2] * 0.5  # Simplified mapping
        
        return ankle_torques, hip_torques
    
    def is_stable(self, state: BalanceState) -> bool:
        """Check if current state is stable"""
        return state.is_balanced and state.stability_margin > 0.02
    
    def predict_fall(self, state: BalanceState, horizon: float = 0.5) -> bool:
        """
        Predict if robot will fall within the given time horizon.
        
        Args:
            state: Current state
            horizon: Time horizon for prediction (seconds)
            
        Returns:
            True if fall is predicted
        """
        # Simple prediction based on COM velocity and ZMP
        if not state.is_balanced:
            return True
        
        # Predict future COM position
        future_com = state.com_position[:2] + state.com_velocity[:2] * horizon
        
        # Check if predicted position is too far from support
        max_stable_offset = 0.05  # 5cm
        return np.linalg.norm(future_com) > max_stable_offset


def create_balance_state_from_simulation(
    position: np.ndarray,
    velocity: np.ndarray,
    orientation: np.ndarray,
    angular_velocity: np.ndarray,
    left_contact: bool = True,
    right_contact: bool = True
) -> BalanceState:
    """
    Helper function to create BalanceState from simulation data.
    """
    # Estimate acceleration from velocity (would use filtered derivative in practice)
    acceleration = np.zeros(3)
    
    return BalanceState(
        com_position=position,
        com_velocity=velocity,
        com_acceleration=acceleration,
        orientation=orientation,
        angular_velocity=angular_velocity,
        zmp=np.zeros(3),  # Will be computed
        left_foot_contact=left_contact,
        right_foot_contact=right_contact
    )


if __name__ == "__main__":
    # Simple test
    logging.basicConfig(level=logging.INFO)
    
    config = BalanceConfig()
    controller = BalanceController(config)
    
    # Create test state (slightly off-balance)
    state = BalanceState(
        com_position=np.array([0.02, 0.0, 0.9]),  # 2cm forward, 90cm high
        com_velocity=np.array([0.1, 0.0, 0.0]),   # Moving forward
        com_acceleration=np.zeros(3),
        orientation=np.array([0.0, 0.05, 0.0]),   # 5° pitch
        angular_velocity=np.zeros(3),
        zmp=np.zeros(3)
    )
    
    # Compute control
    ankle_torques, hip_torques = controller.compute_control(state, 0.001)
    
    print(f"Balance state: {state.is_balanced}")
    print(f"Stability margin: {state.stability_margin:.3f}m")
    print(f"ZMP: {state.zmp}")
    print(f"Ankle torques: {ankle_torques}")
    print(f"Hip torques: {hip_torques}")
    print(f"Fall predicted: {controller.predict_fall(state)}")
