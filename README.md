# HumanoidOS

Open-source operating system for humanoid robots.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.9+](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/)


https://github.com/user-attachments/assets/32c3d915-4a07-40b2-9bd8-c78669f05bd6
> *Validation test of the Zero-G Kinematics Engine and Joint Control Loop.*

## Overview

HumanoidOS provides a complete software stack for bipedal humanoid robots - from real-time control to autonomous locomotion. Built for researchers, hobbyists, and companies who need a solid foundation without reinventing the wheel.

## Features

- Real-time control loop (1kHz)
- ZMP-based balance controller
- Multiple gait patterns (walk, run, sidestep, turn)
- Automatic push recovery using capture point dynamics
- PyBullet physics simulation
- Comprehensive test suite (81 tests)
- **Terrain adaptation** — slope, stairs, and uneven ground with foot placement & body lean compensation
- **Arm manipulation & IK** — 7-DOF Jacobian IK solver, smooth trajectory interpolation, balance-assist swinging
- **Vision-based navigation** — depth raycasting, occupancy grid, A\* path planning, pure-pursuit controller
- **Hardware abstraction layer** — unified interface supporting simulation, real-robot serial/CAN, and extensible backends
- **ROS 2 integration** — bridge for joint states, IMU, odometry, cmd_vel, and emergency stop (gracefully stubs when ROS 2 is absent)

## Quick Start

```bash
git clone https://github.com/ashishjsharda/humanoid-os.git
cd humanoid-os
pip install -r requirements.txt

# Run all tests
python -m pytest tests/ -v

# Walk demo
python examples/walking_demo.py

# Terrain demo (slope / stairs / uneven)
python examples/terrain_demo.py --terrain stairs

# Arm manipulation demo
python examples/arm_manipulation_demo.py

# Vision-based navigation demo
python examples/navigation_demo.py
```

## Usage

```python
# Gait generation
from locomotion.advanced_gaits import AdvancedGaitGenerator, GaitType

gait = AdvancedGaitGenerator()
gait.set_gait(GaitType.FORWARD_WALK, speed=0.5)
for _ in range(1000):
    state = gait.update(dt=0.01)

# Arm IK
from kinematics.inverse_kinematics import JacobianIKSolver, make_7dof_arm_chain
solver = JacobianIKSolver(make_7dof_arm_chain())
result = solver.solve(target=np.array([0.35, -0.10, 0.80]))

# Navigation
from navigation.path_planner import PathPlanner, NavigationController, NavigationGoal
nav = NavigationController()
nav.set_goal(NavigationGoal(position=np.array([4.0, 0.0])))

# Hardware abstraction
from hal.simulation_hal import SimulationHAL
hal = SimulationHAL(robot)
hal.initialize()
states = hal.read_joint_states()

# ROS 2 bridge (stubs gracefully if rclpy not installed)
from ros2.ros2_bridge import ROS2Bridge
bridge = ROS2Bridge()
bridge.start()
```

## Architecture

```
humanoid-os/
├── core/              # Control loop and state management
├── locomotion/        # Balance, gaits, push recovery, terrain adaptation, arm controller
├── kinematics/        # Inverse kinematics (Jacobian / DH / PyBullet)
├── navigation/        # A* path planner, pure-pursuit navigation controller
├── sensors/           # Vision system (depth raycasting, occupancy grid)
├── hal/               # Hardware abstraction layer (simulation + real-robot stubs)
├── ros2/              # ROS 2 bridge (publishes joints, IMU, odom; subscribes cmd_vel)
├── simulation/        # PyBullet integration, terrain generation
├── examples/          # Demo applications
└── tests/             # Test suite (81 tests, no PyBullet required)
```

## Examples

| Demo | Command | Description |
|------|---------|-------------|
| Walking | `python examples/walking_demo.py` | Basic bipedal locomotion |
| Terrain | `python examples/terrain_demo.py --terrain slope` | Walk on slopes, stairs, or uneven ground |
| Arm | `python examples/arm_manipulation_demo.py` | Reach, wave, and balance-assist arm motions |
| Navigation | `python examples/navigation_demo.py` | Vision-based obstacle avoidance and A* navigation |

Add `--no-gui` to any demo to run headless.

## Performance

- Control frequency: 1000 Hz
- Walking speed: 0.1 - 1.5 m/s
- Recovery response: <200ms
- IK solver: <5ms per solve (7-DOF Jacobian, 200 iterations)
- Vision update: 200ms cycle (24×16 ray grid)
- Test suite: 81 tests, ~1s

## Roadmap

- [x] Terrain adaptation (Slope/Stairs)
- [x] Arm manipulation & IK
- [x] Vision-based navigation
- [x] Hardware abstraction layer
- [x] ROS 2 integration

## Contributing

Contributions welcome! Open an issue or submit a PR.

## License

MIT License - see [LICENSE](LICENSE)

## Background

Built after working on robotics projects at Apple, Formant, and various startups. Got tired of rebuilding the same control systems over and over, so decided to open-source a solid foundation.

## Contact

- Issues: [GitHub Issues](https://github.com/ashishjsharda/humanoid-os/issues)
- Twitter: [@ashishjsharda](https://twitter.com/ashishjsharda)
