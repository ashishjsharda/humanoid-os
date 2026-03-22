"""Tests for Hardware Abstraction Layer."""

import numpy as np
import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hal.hardware_interface import (
    HardwareInterface,
    JointState,
    JointCommand,
    IMUData,
    FootContact,
    ControlMode,
)
from hal.real_robot_hal import RealRobotHAL


# ---------------------------------------------------------------------------
# Dataclass sanity
# ---------------------------------------------------------------------------

class TestHALDataclasses:
    def test_joint_state_fields(self):
        js = JointState(name="hip_left", position=0.5, velocity=0.1, torque=2.0)
        assert js.name == "hip_left"
        assert js.position == 0.5
        assert js.temperature == 0.0  # default

    def test_joint_command_defaults(self):
        cmd = JointCommand(name="knee_right")
        assert cmd.mode == ControlMode.TORQUE
        assert cmd.value == 0.0
        assert cmd.kp == 100.0

    def test_imu_data_fields(self):
        imu = IMUData(
            orientation=np.array([0.0, 0.1, 0.0]),
            angular_velocity=np.zeros(3),
            linear_acceleration=np.array([0.0, 0.0, 9.81]),
        )
        assert imu.orientation.shape == (3,)
        assert imu.timestamp == 0.0

    def test_foot_contact_defaults(self):
        fc = FootContact()
        assert fc.left  is False
        assert fc.right is False
        assert fc.left_force == 0.0


# ---------------------------------------------------------------------------
# HardwareInterface ABC enforcement
# ---------------------------------------------------------------------------

class TestHardwareInterfaceABC:
    def test_cannot_instantiate_abstract(self):
        with pytest.raises(TypeError):
            HardwareInterface()


# ---------------------------------------------------------------------------
# RealRobotHAL (stub)
# ---------------------------------------------------------------------------

class TestRealRobotHAL:
    def _hal(self, names=None):
        return RealRobotHAL(joint_names=names or ["j0", "j1", "j2"])

    def test_initialize_returns_true(self):
        hal = self._hal()
        assert hal.initialize() is True
        assert hal.is_connected() is True

    def test_shutdown_disconnects(self):
        hal = self._hal()
        hal.initialize()
        hal.shutdown()
        assert hal.is_connected() is False

    def test_read_joint_states_returns_all(self):
        names = ["shoulder", "elbow", "wrist"]
        hal   = self._hal(names)
        hal.initialize()
        states = hal.read_joint_states()
        assert set(states.keys()) == set(names)
        for s in states.values():
            assert isinstance(s, JointState)

    def test_read_imu_returns_imudata(self):
        hal = self._hal()
        hal.initialize()
        imu = hal.read_imu()
        assert isinstance(imu, IMUData)
        assert imu.linear_acceleration[2] == pytest.approx(9.81)

    def test_read_foot_contacts_returns_footcontact(self):
        hal = self._hal()
        hal.initialize()
        fc = hal.read_foot_contacts()
        assert isinstance(fc, FootContact)

    def test_write_torque_command(self):
        hal = self._hal(["j0", "j1"])
        hal.initialize()
        cmds = [
            JointCommand(name="j0", mode=ControlMode.TORQUE, value=5.0),
            JointCommand(name="j1", mode=ControlMode.TORQUE, value=-3.0),
        ]
        hal.write_joint_commands(cmds)  # should not raise

    def test_write_position_command_updates_shadow(self):
        hal = self._hal(["j0"])
        hal.initialize()
        cmds = [JointCommand(name="j0", mode=ControlMode.POSITION, value=0.785)]
        hal.write_joint_commands(cmds)
        assert hal._last_position["j0"] == pytest.approx(0.785)

    def test_emergency_stop_prevents_commands(self):
        hal = self._hal(["j0"])
        hal.initialize()
        hal.emergency_stop()
        assert hal.is_emergency_stopped() is True
        # Commands after e-stop should be silently ignored
        hal.write_joint_commands([JointCommand(name="j0", value=100.0)])
        assert hal._last_position.get("j0", 0.0) == pytest.approx(0.0)

    def test_command_torque_helper(self):
        hal = self._hal(["a", "b"])
        hal.initialize()
        hal.command_torque({"a": 10.0, "b": -5.0})  # should not raise

    def test_command_position_helper(self):
        hal = self._hal(["a"])
        hal.initialize()
        hal.command_position({"a": 0.5}, kp=120.0, kd=12.0)
        assert hal._last_position["a"] == pytest.approx(0.5)

    def test_zero_all_torques(self):
        hal = self._hal(["a", "b"])
        hal.initialize()
        hal.zero_all_torques()  # should not raise

    def test_repr(self):
        hal = self._hal()
        r = repr(hal)
        assert "RealRobotHAL" in r


# ---------------------------------------------------------------------------
# ROS 2 Bridge (stub mode)
# ---------------------------------------------------------------------------

class TestROS2BridgeStub:
    def test_import(self):
        from ros2.ros2_bridge import ROS2Bridge, ROS2Config
        cfg    = ROS2Config()
        bridge = ROS2Bridge(cfg)
        assert bridge is not None

    def test_start_stop_stub(self):
        from ros2.ros2_bridge import ROS2Bridge, ROS2Config
        bridge = ROS2Bridge(ROS2Config())
        bridge.start()
        assert bridge._node is not None
        bridge.stop()

    def test_get_velocity_command_default_zeros(self):
        from ros2.ros2_bridge import ROS2Bridge
        bridge = ROS2Bridge()
        bridge.start()
        cmd = bridge.get_velocity_command()
        assert np.allclose(cmd, 0.0)
        bridge.stop()

    def test_is_emergency_stop_default_false(self):
        from ros2.ros2_bridge import ROS2Bridge
        bridge = ROS2Bridge()
        bridge.start()
        assert bridge.is_emergency_stop_requested() is False
        bridge.stop()

    def test_publish_methods_dont_raise_in_stub(self):
        from ros2.ros2_bridge import ROS2Bridge
        bridge = ROS2Bridge()
        bridge.start()
        # These should silently do nothing in stub mode
        bridge.publish_joint_states({})
        bridge.publish_imu(IMUData(
            orientation=np.zeros(3),
            angular_velocity=np.zeros(3),
            linear_acceleration=np.zeros(3),
        ))
        bridge.publish_odometry(np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))
        bridge.publish_robot_state({"state": "idle"})
        bridge.stop()

    def test_euler_to_quat_identity(self):
        from ros2.ros2_bridge import ROS2Bridge
        q = ROS2Bridge._euler_to_quat(0.0, 0.0, 0.0)
        assert q[3] == pytest.approx(1.0, abs=1e-6)  # w=1 for identity

    def test_euler_to_quat_unit_norm(self):
        from ros2.ros2_bridge import ROS2Bridge
        for angles in [(0.1, 0.2, 0.3), (-0.5, 0.0, 1.2), (np.pi, 0.0, 0.0)]:
            q = ROS2Bridge._euler_to_quat(*angles)
            assert np.linalg.norm(q) == pytest.approx(1.0, abs=1e-6)
