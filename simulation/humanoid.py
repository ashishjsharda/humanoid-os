"""
Humanoid Robot Simulation using PyBullet

Provides a simulated humanoid robot for testing the control system.
"""

import numpy as np
import pybullet as p
import pybullet_data
from typing import Optional, List, Dict, Tuple
from dataclasses import dataclass
import logging
import time

logger = logging.getLogger(__name__)


@dataclass
class JointConfig:
    """Configuration for a single joint"""
    name: str
    index: int
    type: int  # PyBullet joint type
    lower_limit: float
    upper_limit: float
    max_force: float
    max_velocity: float


class HumanoidRobot:
    """
    Simulated humanoid robot in PyBullet.
    
    Uses a simplified humanoid model with:
    - Torso
    - Two legs (hip, knee, ankle joints)
    - Two arms (shoulder, elbow joints)
    - Head
    """
    
    def __init__(
        self,
        use_gui: bool = True,
        dt: float = 0.001,
        use_default_model: bool = True
    ):
        """
        Initialize robot simulation.
        
        Args:
            use_gui: Whether to show GUI
            dt: Simulation timestep
            use_default_model: Use PyBullet's humanoid URDF
        """
        self.dt = dt
        self.use_gui = use_gui
        
        # Connect to PyBullet
        if use_gui:
            self.physics_client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        else:
            self.physics_client = p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(dt)
        
        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Load humanoid robot
        if use_default_model:
            # Use PyBullet's humanoid model
            start_pos = [0, 0, 1.0]
            start_orientation = p.getQuaternionFromEuler([0, 0, 0])
            self.robot_id = p.loadURDF(
                "humanoid/humanoid.urdf",
                start_pos,
                start_orientation,
                useFixedBase=False,
                flags=p.URDF_USE_SELF_COLLISION
            )
        else:
            # Would load custom URDF here
            raise NotImplementedError("Custom humanoid model not yet implemented")
        
        # Get joint information
        self.num_joints = p.getNumJoints(self.robot_id)
        self.joints: Dict[str, JointConfig] = {}
        self._parse_joints()
        
        # Define key joint groups
        self.leg_joints = [
            name for name in self.joints.keys()
            if any(x in name.lower() for x in ['hip', 'knee', 'ankle', 'thigh', 'shin'])
        ]
        
        self.arm_joints = [
            name for name in self.joints.keys()
            if any(x in name.lower() for x in ['shoulder', 'elbow', 'wrist'])
        ]
        
        # Enable torque control mode
        self._setup_control_mode()
        
        # State tracking
        self.step_count = 0
        
        logger.info(f"Humanoid robot initialized with {self.num_joints} joints")
        logger.info(f"Leg joints: {self.leg_joints}")
    
    def _parse_joints(self):
        """Parse joint information from loaded robot"""
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]
            
            # Only track revolute and prismatic joints
            if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                self.joints[joint_name] = JointConfig(
                    name=joint_name,
                    index=i,
                    type=joint_type,
                    lower_limit=joint_info[8],
                    upper_limit=joint_info[9],
                    max_force=joint_info[10],
                    max_velocity=joint_info[11]
                )
    
    def _setup_control_mode(self):
        """Setup torque control for all joints"""
        for joint in self.joints.values():
            # Disable default motor control
            p.setJointMotorControl2(
                self.robot_id,
                joint.index,
                p.VELOCITY_CONTROL,
                force=0
            )
    
    def get_base_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Get robot base (torso) state.
        
        Returns:
            (position, orientation_euler, linear_velocity, angular_velocity)
        """
        pos, quat = p.getBasePositionAndOrientation(self.robot_id)
        vel, ang_vel = p.getBaseVelocity(self.robot_id)
        
        # Convert quaternion to euler
        euler = p.getEulerFromQuaternion(quat)
        
        return (
            np.array(pos),
            np.array(euler),
            np.array(vel),
            np.array(ang_vel)
        )
    
    def get_joint_states(self) -> Dict[str, Tuple[float, float, float]]:
        """
        Get current joint states.
        
        Returns:
            Dictionary mapping joint names to (position, velocity, torque)
        """
        states = {}
        for name, joint in self.joints.items():
            joint_state = p.getJointState(self.robot_id, joint.index)
            states[name] = (
                joint_state[0],  # position
                joint_state[1],  # velocity
                joint_state[3]   # applied torque
            )
        return states
    
    def get_foot_contacts(self) -> Tuple[bool, bool]:
        """
        Check if feet are in contact with ground.
        
        Returns:
            (left_foot_contact, right_foot_contact)
        """
        # Get contact points
        contact_points = p.getContactPoints(self.robot_id, self.plane_id)
        
        # Find foot links (simplified - would need proper link identification)
        left_contact = False
        right_contact = False
        
        for contact in contact_points:
            link_index = contact[3]
            # This is simplified - proper implementation would identify foot links
            # For now, assume links near the bottom are feet
            if link_index < self.num_joints / 2:
                left_contact = True
            else:
                right_contact = True
        
        return left_contact, right_contact
    
    def apply_joint_torques(self, torques: Dict[str, float]):
        """
        Apply torques to joints.
        
        Args:
            torques: Dictionary mapping joint names to desired torques
        """
        for joint_name, torque in torques.items():
            if joint_name in self.joints:
                joint = self.joints[joint_name]
                
                # Clip to max force
                torque = np.clip(torque, -joint.max_force, joint.max_force)
                
                # Apply torque
                p.setJointMotorControl2(
                    self.robot_id,
                    joint.index,
                    p.TORQUE_CONTROL,
                    force=torque
                )
    
    def set_joint_positions(
        self,
        positions: Dict[str, float],
        kp: float = 100.0,
        kd: float = 10.0
    ):
        """
        Set desired joint positions using PD control.
        
        Args:
            positions: Dictionary mapping joint names to desired positions
            kp: Position gain
            kd: Velocity gain
        """
        current_states = self.get_joint_states()
        torques = {}
        
        for joint_name, target_pos in positions.items():
            if joint_name in current_states:
                current_pos, current_vel, _ = current_states[joint_name]
                
                # PD control
                error = target_pos - current_pos
                torque = kp * error - kd * current_vel
                
                torques[joint_name] = torque
        
        self.apply_joint_torques(torques)
    
    def reset_pose(self, position: Optional[np.ndarray] = None):
        """
        Reset robot to default standing pose.
        
        Args:
            position: Optional starting position [x, y, z]
        """
        if position is None:
            position = np.array([0.0, 0.0, 1.0])
        
        p.resetBasePositionAndOrientation(
            self.robot_id,
            position,
            p.getQuaternionFromEuler([0, 0, 0])
        )
        
        # Reset joint positions to neutral standing pose
        for joint in self.joints.values():
            p.resetJointState(self.robot_id, joint.index, 0.0)
        
        self.step_count = 0
        logger.info("Robot pose reset")
    
    def step_simulation(self):
        """Advance simulation by one timestep"""
        p.stepSimulation()
        self.step_count += 1
    
    def add_debug_lines(self):
        """Add debug visualization (COM, ZMP, etc.)"""
        if not self.use_gui:
            return
        
        # Draw COM
        pos, _, _, _ = self.get_base_state()
        p.addUserDebugLine(
            pos,
            pos - np.array([0, 0, 0.2]),
            [1, 0, 0],
            2,
            lifeTime=0.1
        )
    
    def disconnect(self):
        """Disconnect from PyBullet"""
        p.disconnect(self.physics_client)
        logger.info("Simulation disconnected")


if __name__ == "__main__":
    # Simple test
    logging.basicConfig(level=logging.INFO)
    
    print("Initializing humanoid robot simulation...")
    robot = HumanoidRobot(use_gui=True, dt=0.001)
    
    print("\nRobot initialized!")
    print(f"Total joints: {robot.num_joints}")
    print(f"Controllable joints: {len(robot.joints)}")
    print(f"Leg joints: {robot.leg_joints}")
    print(f"Arm joints: {robot.arm_joints}")
    
    # Run simulation for a few seconds
    print("\nRunning simulation for 5 seconds...")
    print("The robot should maintain balance in standing pose.")
    
    start_time = time.time()
    while time.time() - start_time < 5.0:
        # Get state
        pos, euler, vel, ang_vel = robot.get_base_state()
        left_contact, right_contact = robot.get_foot_contacts()
        
        # Simple PD control to maintain upright pose
        # (In practice, this would be handled by the control loop)
        target_positions = {joint_name: 0.0 for joint_name in robot.joints.keys()}
        robot.set_joint_positions(target_positions, kp=50.0, kd=5.0)
        
        robot.step_simulation()
        robot.add_debug_lines()
        
        time.sleep(robot.dt)
    
    print("\nFinal state:")
    pos, euler, vel, ang_vel = robot.get_base_state()
    print(f"  Position: {pos}")
    print(f"  Orientation (deg): {np.degrees(euler)}")
    print(f"  Height: {pos[2]:.3f}m")
    
    robot.disconnect()
    print("\nTest complete!")
