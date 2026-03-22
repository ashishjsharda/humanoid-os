"""
Hardware Abstraction Layer (HAL) for HumanoidOS

Provides a unified interface so control code is hardware-agnostic.
Swap SimulationHAL for RealRobotHAL without touching any controller.
"""

from hal.hardware_interface import (
    HardwareInterface,
    JointState,
    JointCommand,
    IMUData,
    FootContact,
    ControlMode,
)
from hal.simulation_hal import SimulationHAL
from hal.real_robot_hal import RealRobotHAL

__all__ = [
    "HardwareInterface",
    "JointState",
    "JointCommand",
    "IMUData",
    "FootContact",
    "ControlMode",
    "SimulationHAL",
    "RealRobotHAL",
]
