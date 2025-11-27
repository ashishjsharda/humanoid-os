"""
Unit tests for balance controller
"""

import pytest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from locomotion.balance import (
    BalanceController,
    BalanceConfig,
    BalanceState,
    create_balance_state_from_simulation
)


class TestBalanceConfig:
    """Test BalanceConfig dataclass"""
    
    def test_default_config(self):
        """Test default configuration values"""
        config = BalanceConfig()
        assert config.robot_mass == 50.0
        assert config.robot_height == 1.5
        assert config.kp_position is not None
        assert config.kd_position is not None
        
    def test_custom_config(self):
        """Test custom configuration"""
        config = BalanceConfig(
            robot_mass=60.0,
            robot_height=1.8,
            max_ankle_torque=150.0
        )
        assert config.robot_mass == 60.0
        assert config.robot_height == 1.8
        assert config.max_ankle_torque == 150.0


class TestBalanceState:
    """Test BalanceState dataclass"""
    
    def test_balanced_state(self):
        """Test detection of balanced state"""
        state = BalanceState(
            com_position=np.array([0.0, 0.0, 0.9]),
            com_velocity=np.zeros(3),
            com_acceleration=np.zeros(3),
            orientation=np.zeros(3),
            angular_velocity=np.zeros(3),
            zmp=np.array([0.01, 0.01, 0.0]),  # Small offset
            left_foot_contact=True,
            right_foot_contact=True
        )
        assert state.is_balanced
        assert state.stability_margin > 0
    
    def test_unbalanced_state(self):
        """Test detection of unbalanced state"""
        state = BalanceState(
            com_position=np.array([0.0, 0.0, 0.9]),
            com_velocity=np.zeros(3),
            com_acceleration=np.zeros(3),
            orientation=np.zeros(3),
            angular_velocity=np.zeros(3),
            zmp=np.array([0.15, 0.0, 0.0]),  # Large offset
            left_foot_contact=True,
            right_foot_contact=True
        )
        assert not state.is_balanced
    
    def test_no_contact_unbalanced(self):
        """Test that no ground contact means unbalanced"""
        state = BalanceState(
            com_position=np.array([0.0, 0.0, 0.9]),
            com_velocity=np.zeros(3),
            com_acceleration=np.zeros(3),
            orientation=np.zeros(3),
            angular_velocity=np.zeros(3),
            zmp=np.zeros(3),
            left_foot_contact=False,
            right_foot_contact=False
        )
        assert not state.is_balanced


class TestBalanceController:
    """Test BalanceController class"""
    
    def test_initialization(self):
        """Test controller initialization"""
        controller = BalanceController()
        assert controller.config is not None
        assert controller.gravity[2] == -9.81
        
    def test_zmp_computation(self):
        """Test ZMP calculation"""
        controller = BalanceController()
        
        state = BalanceState(
            com_position=np.array([0.0, 0.0, 0.9]),
            com_velocity=np.zeros(3),
            com_acceleration=np.array([1.0, 0.0, 0.0]),  # Accelerating forward
            orientation=np.zeros(3),
            angular_velocity=np.zeros(3),
            zmp=np.zeros(3),
            left_foot_contact=True,
            right_foot_contact=True
        )
        
        zmp = controller.compute_zmp(state)
        
        # ZMP should be behind COM when accelerating forward
        assert zmp[0] < state.com_position[0]
        assert zmp[2] == 0.0  # ZMP is on ground
    
    def test_control_output_shape(self):
        """Test that control outputs have correct shape"""
        controller = BalanceController()
        
        state = BalanceState(
            com_position=np.array([0.02, 0.0, 0.9]),
            com_velocity=np.array([0.1, 0.0, 0.0]),
            com_acceleration=np.zeros(3),
            orientation=np.array([0.0, 0.05, 0.0]),  # 5° pitch
            angular_velocity=np.zeros(3),
            zmp=np.zeros(3),
            left_foot_contact=True,
            right_foot_contact=True
        )
        
        ankle_torques, hip_torques = controller.compute_control(state, 0.001)
        
        assert ankle_torques.shape == (2,)
        assert hip_torques.shape == (2,)
    
    def test_torque_limits(self):
        """Test that torques respect limits"""
        config = BalanceConfig(max_ankle_torque=50.0)
        controller = BalanceController(config)
        
        # Create highly unstable state
        state = BalanceState(
            com_position=np.array([0.1, 0.1, 0.9]),
            com_velocity=np.array([1.0, 1.0, 0.0]),
            com_acceleration=np.zeros(3),
            orientation=np.array([0.3, 0.3, 0.0]),  # Large tilt
            angular_velocity=np.array([1.0, 1.0, 0.0]),
            zmp=np.zeros(3),
            left_foot_contact=True,
            right_foot_contact=True
        )
        
        ankle_torques, hip_torques = controller.compute_control(state, 0.001)
        
        # Check torques are within limits
        assert np.all(np.abs(ankle_torques) <= config.max_ankle_torque)
    
    def test_stability_check(self):
        """Test stability checking"""
        controller = BalanceController()
        
        # Stable state
        stable_state = BalanceState(
            com_position=np.array([0.0, 0.0, 0.9]),
            com_velocity=np.zeros(3),
            com_acceleration=np.zeros(3),
            orientation=np.zeros(3),
            angular_velocity=np.zeros(3),
            zmp=np.array([0.01, 0.01, 0.0]),
            left_foot_contact=True,
            right_foot_contact=True
        )
        
        assert controller.is_stable(stable_state)
        
        # Unstable state
        unstable_state = BalanceState(
            com_position=np.array([0.0, 0.0, 0.9]),
            com_velocity=np.zeros(3),
            com_acceleration=np.zeros(3),
            orientation=np.zeros(3),
            angular_velocity=np.zeros(3),
            zmp=np.array([0.1, 0.0, 0.0]),
            left_foot_contact=True,
            right_foot_contact=True
        )
        
        assert not controller.is_stable(unstable_state)
    
    def test_fall_prediction(self):
        """Test fall prediction"""
        controller = BalanceController()
        
        # State that will fall
        falling_state = BalanceState(
            com_position=np.array([0.02, 0.0, 0.9]),
            com_velocity=np.array([0.5, 0.0, 0.0]),  # Moving fast forward
            com_acceleration=np.zeros(3),
            orientation=np.zeros(3),
            angular_velocity=np.zeros(3),
            zmp=np.zeros(3),
            left_foot_contact=True,
            right_foot_contact=True
        )
        
        will_fall = controller.predict_fall(falling_state, horizon=0.5)
        assert will_fall
        
        # Stable state
        stable_state = BalanceState(
            com_position=np.array([0.0, 0.0, 0.9]),
            com_velocity=np.array([0.01, 0.0, 0.0]),  # Slow movement
            com_acceleration=np.zeros(3),
            orientation=np.zeros(3),
            angular_velocity=np.zeros(3),
            zmp=np.zeros(3),
            left_foot_contact=True,
            right_foot_contact=True
        )
        
        will_fall = controller.predict_fall(stable_state, horizon=0.5)
        assert not will_fall


class TestHelperFunctions:
    """Test helper functions"""
    
    def test_create_balance_state_from_simulation(self):
        """Test creation of balance state from sim data"""
        position = np.array([0.1, 0.2, 0.9])
        velocity = np.array([0.5, 0.0, 0.0])
        orientation = np.array([0.0, 0.1, 0.0])
        angular_velocity = np.array([0.0, 0.5, 0.0])
        
        state = create_balance_state_from_simulation(
            position, velocity, orientation, angular_velocity
        )
        
        assert np.allclose(state.com_position, position)
        assert np.allclose(state.com_velocity, velocity)
        assert np.allclose(state.orientation, orientation)
        assert np.allclose(state.angular_velocity, angular_velocity)
        assert state.left_foot_contact
        assert state.right_foot_contact


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
