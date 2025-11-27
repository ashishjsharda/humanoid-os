# Changelog

All notable changes to HumanoidOS will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2024-12-02

### Added
- Initial release of HumanoidOS
- Core control loop running at 1kHz
- Zero Moment Point (ZMP) based balance controller
- Bipedal gait generator with cycloid foot trajectories
- PyBullet simulation integration
- Real-time metrics and telemetry
- State machine for system management
- Emergency stop and safety monitoring
- Balance control demo
- Walking demo
- Comprehensive documentation
- Unit tests for core components
- CI/CD pipeline with GitHub Actions
- MIT license

### Features
- **Control Loop**: High-frequency (1kHz) deterministic control cycle
- **Balance Control**: PD-based balance maintenance using ZMP
- **Gait Generation**: Walking trajectory generation with configurable parameters
- **Simulation**: PyBullet-based physics simulation
- **Safety**: Emergency stop, fall detection, timing violation monitoring
- **Developer API**: Clean Python API for extending functionality
- **Examples**: Working demos for balance and walking

### Documentation
- Architecture overview
- Getting started guide
- API documentation
- Contributing guidelines
- Installation instructions

### Testing
- Unit tests for balance controller
- Unit tests for control loop
- Integration test framework
- CI/CD with GitHub Actions

### Known Limitations
- Simplified joint-to-torque mapping
- No inverse kinematics yet
- No perception system yet
- Simulation only (no hardware support)
- Basic foot contact detection

### Roadmap
See README.md for full roadmap.

---

## [Unreleased]

### Planned for v0.2.0
- Inverse kinematics solver
- Improved gait stability
- Terrain adaptation
- Hardware abstraction layer
- ROS 2 bridge
- Enhanced visualization tools

### Planned for v0.3.0
- Perception system (computer vision)
- Manipulation planning
- Whole-body motion control
- Learning-based controllers
- Multi-robot coordination

---

*For more details, see the [commit history](https://github.com/ashishsharda/humanoid-os/commits/main).*
