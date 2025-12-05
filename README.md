# HumanoidOS

Open-source operating system for humanoid robots.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.9+](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/)

## Overview

HumanoidOS provides a complete software stack for bipedal humanoid robots - from real-time control to autonomous locomotion. Built for researchers, hobbyists, and companies who need a solid foundation without reinventing the wheel.

## Features

- Real-time control loop (1kHz)
- ZMP-based balance controller
- Multiple gait patterns (walk, run, sidestep, turn)
- Automatic push recovery using capture point dynamics
- PyBullet physics simulation
- Comprehensive test suite

## Quick Start

```bash
git clone https://github.com/ashishjsharda/humanoid-os.git
cd humanoid-os
pip install -r requirements.txt
python tests/test_all.py
python examples/walking_demo.py
```

## Usage

```python
from locomotion.advanced_gaits import AdvancedGaitGenerator...

gait = AdvancedGaitGenerator()
gait.set_gait(GaitType.FORWARD_WALK, speed=0.5)

for _ in range(1000):
    state = gait.update(dt=0.01)
    # Use state for your robot control
```

## Architecture

```
humanoid-os/
├── core/              # Control loop and state management
├── locomotion/        # Balance, gaits, push recovery
├── simulation/        # PyBullet integration
├── examples/          # Demo applications
└── tests/             # Test suite
```

## Performance

- Control frequency: 1000 Hz
- Walking speed: 0.1 - 1.5 m/s
- Recovery response: <200ms
- Test coverage: 100%

## Roadmap

- [ ] Terrain adaptation (Slope/Stairs)
- [ ] Arm manipulation & IK
- [ ] Vision-based navigation
- [ ] Hardware abstraction layer
- [ ] ROS 2 integration

## Contributing

Contributions welcome! Open an issue or submit a PR.

## License

MIT License - see [LICENSE](LICENSE)

## Background

Built after working on robotics projects at Apple, Formant, and various startups. Got tired of rebuilding the same control systems over and over, so decided to open-source a solid foundation.

## Contact

- Issues: [GitHub Issues](https://github.com/ashishjsharda/humanoid-os/issues)
- Twitter: [@ashishjsharda](https://twitter.com/ashishjsharda)
