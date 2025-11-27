# Getting Started with HumanoidOS

This guide will help you get up and running with HumanoidOS in minutes.

## Prerequisites

- Python 3.8 or higher
- pip package manager
- (Optional) GPU for faster simulation

## Installation

### Quick Install

```bash
# Clone the repository
git clone https://github.com/ashishsharda/humanoid-os.git
cd humanoid-os

# Install dependencies
pip install -r requirements.txt
```

### Verify Installation

```bash
# Run a simple test
python -c "import pybullet; import numpy; print('Installation successful!')"
```

## Your First Demo

### 1. Balance Control

The simplest demo shows a humanoid robot maintaining balance:

```bash
python examples/balance_demo.py
```

You should see:
- A PyBullet GUI window with a humanoid robot
- The robot standing upright and maintaining balance
- Console output showing control metrics

**Try it with disturbance:**
```bash
python examples/balance_demo.py --disturbance
```

The robot will be pushed at 2 seconds and recover balance!

### 2. Walking Demo

See the robot walk:

```bash
python examples/walking_demo.py --duration 30
```

The robot will:
1. Stand and stabilize for 2 seconds
2. Begin walking forward
3. Continue walking for 28 seconds

### 3. Headless Mode

Run without GUI for faster simulation:

```bash
python examples/balance_demo.py --no-gui --duration 5
```

## Understanding the Demos

### Balance Demo

**What's happening:**
1. Control loop runs at 1000 Hz (1ms cycles)
2. Sensors read robot state (position, orientation, velocities)
3. Balance controller computes corrective torques
4. Torques applied to ankle and hip joints
5. Physics simulation steps forward

**Key concepts:**
- **ZMP (Zero Moment Point):** Point where ground reaction moments sum to zero
- **COM (Center of Mass):** Robot's center of gravity
- **PD Control:** Proportional-Derivative feedback control

### Walking Demo

**What's happening:**
1. Gait generator creates foot trajectories
2. Balance controller maintains stability
3. Feet follow cycloid trajectories
4. COM shifts laterally for weight transfer
5. Continuous stepping motion

**Gait phases:**
- **Double Support:** Both feet on ground
- **Single Support:** One foot in air, other supports
- **Swing Phase:** Moving foot follows trajectory

## Basic Usage

### Creating a Simple Controller

```python
from core.control_loop import ControlLoop, ControlConfig
from simulation.humanoid import HumanoidRobot

# Initialize components
config = ControlConfig(frequency_hz=1000.0)
loop = ControlLoop(config)
robot = HumanoidRobot(use_gui=True)

# Define control function
def my_controller(dt, state):
    # Your control logic here
    positions, _, _, _ = robot.get_base_state()
    print(f"Robot at {positions}")
    return {"status": "ok"}

# Register and run
loop.register_subsystem("my_control", my_controller)
loop.run(duration=5.0)  # Run for 5 seconds
```

### Reading Sensor Data

```python
# Get robot state
position, orientation, velocity, angular_velocity = robot.get_base_state()

# Get joint states
joint_states = robot.get_joint_states()
for name, (pos, vel, torque) in joint_states.items():
    print(f"{name}: position={pos:.3f}, velocity={vel:.3f}")

# Check foot contacts
left_contact, right_contact = robot.get_foot_contacts()
```

### Applying Control

```python
# Apply joint torques directly
torques = {
    'hip_flexion_r': 10.0,
    'knee_r': -5.0,
    'ankle_r': 2.0
}
robot.apply_joint_torques(torques)

# Or use position control (PD control)
positions = {
    'hip_flexion_r': 0.1,  # radians
    'knee_r': -0.2
}
robot.set_joint_positions(positions, kp=100.0, kd=10.0)
```

## Project Structure

```
humanoid-os/
├── core/               # Core control loop and state management
│   ├── control_loop.py
│   └── state_machine.py (future)
├── locomotion/         # Walking and balance
│   ├── balance.py      # Balance controller
│   └── gait.py         # Gait generator
├── kinematics/         # IK/FK solvers (future)
├── simulation/         # Physics simulation
│   └── humanoid.py     # PyBullet integration
├── sensors/            # Sensor fusion (future)
├── api/                # Public API (future)
├── examples/           # Demo applications
│   ├── balance_demo.py
│   └── walking_demo.py
├── tests/              # Unit tests
└── docs/               # Documentation
    └── ARCHITECTURE.md
```

## Next Steps

### Beginner
1. ✅ Run the demos
2. Modify demo parameters (step length, speed, etc.)
3. Add simple logging or visualization
4. Try different robot poses

### Intermediate
1. Create custom controller
2. Implement new gait pattern
3. Add sensor processing
4. Integrate with ROS

### Advanced
1. Implement IK solver
2. Add perception system
3. Create learning-based controller
4. Port to real hardware

## Common Issues

### PyBullet GUI Not Appearing

**Solution:**
```bash
# Install display dependencies (Linux)
sudo apt-get install xvfb

# Or run without GUI
python examples/balance_demo.py --no-gui
```

### Slow Simulation

**Try:**
- Run without GUI (`--no-gui`)
- Reduce control frequency
- Close other applications
- Use smaller simulation timestep

### Robot Falls Immediately

**Check:**
- Initial pose is stable
- Control gains are appropriate
- Simulation timestep matches control loop
- Ground friction is sufficient

## Configuration

### Control Loop

```python
from core.control_loop import ControlConfig

config = ControlConfig(
    frequency_hz=1000.0,        # Control frequency
    max_cycle_time_ms=5.0,      # Max allowed cycle time
    enable_safety_checks=True,  # Enable safety monitoring
    enable_telemetry=True       # Enable metrics tracking
)
```

### Balance Controller

```python
from locomotion.balance import BalanceConfig
import numpy as np

config = BalanceConfig(
    robot_mass=50.0,                    # kg
    robot_height=1.5,                   # meters
    kp_position=np.array([100, 100, 50]), # Position gains
    kd_position=np.array([20, 20, 10]),   # Velocity gains
    kp_orientation=20.0,                # Orientation gain
    max_ankle_torque=100.0              # Nm
)
```

### Gait Generator

```python
from locomotion.gait import GaitConfig

config = GaitConfig(
    step_length=0.2,            # meters
    step_width=0.15,            # meters
    step_height=0.05,           # meters
    step_duration=0.6,          # seconds
    desired_velocity=0.3,       # m/s
    double_support_ratio=0.2    # 20% of cycle
)
```

## Learning Resources

### Understanding Humanoid Robotics
- [ZMP Tutorial](https://www.youtube.com/watch?v=...) - Zero Moment Point basics
- [Bipedal Walking](https://ocw.mit.edu/...) - MIT OpenCourseWare
- [Modern Robotics](http://hades.mech.northwestern.edu/index.php/Modern_Robotics) - Textbook

### Python & Robotics
- [NumPy Tutorial](https://numpy.org/doc/stable/user/quickstart.html)
- [PyBullet Quickstart](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/)
- [Real-Time Systems](https://en.wikipedia.org/wiki/Real-time_computing)

### Contributing
- See [CONTRIBUTING.md](CONTRIBUTING.md) for development guide
- Check [ARCHITECTURE.md](docs/ARCHITECTURE.md) for system design

## Getting Help

- **Documentation:** Check `docs/` directory
- **Examples:** Look at `examples/` for working code
- **Issues:** Search [GitHub Issues](https://github.com/ashishsharda/humanoid-os/issues)
- **Discussions:** Ask in [GitHub Discussions](https://github.com/ashishsharda/humanoid-os/discussions)

## Example Projects

### Make the Robot Dance
```python
# examples/dance_demo.py (your creation!)
# Implement rhythmic joint movements
```

### Add Obstacle Avoidance
```python
# Use foot contact sensors to detect obstacles
# Modify gait to step over/around them
```

### Create Custom Behaviors
```python
# Combine locomotion primitives
# Add state machines for complex behaviors
```

## Tips for Success

1. **Start Simple:** Get balance working before attempting walking
2. **Tune Gradually:** Adjust one parameter at a time
3. **Log Everything:** Add logging to understand behavior
4. **Visualize:** Use PyBullet's debug lines to see trajectories
5. **Read Code:** The best documentation is in the source code

## What's Next?

Now that you're set up, you can:
- Explore the codebase
- Modify existing demos
- Create new controllers
- Contribute improvements
- Build awesome robots! 🤖

Happy coding!

---

*Need help? Open an issue or start a discussion on GitHub!*
