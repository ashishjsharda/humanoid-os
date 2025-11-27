# HumanoidOS Architecture

## Overview

HumanoidOS is designed as a modular, real-time operating system for humanoid robots. The architecture follows a layered approach with clear separation of concerns.

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Application Layer                     │
│  (User applications, behaviors, task planning)           │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────┐
│                      API Layer                           │
│  (Public Python API, ROS bridges, network interfaces)    │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────┐
│                   Control Layer                          │
│  • Locomotion (Balance, Gait, ZMP)                       │
│  • Manipulation (IK/FK, trajectory planning)             │
│  • Perception (Vision, sensor fusion)                    │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────┐
│                    Core Layer                            │
│  • Real-time control loop (1kHz)                         │
│  • State machine                                         │
│  • Safety system                                         │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────┐
│                 Hardware/Simulation Layer                │
│  • PyBullet simulation                                   │
│  • Hardware drivers (future)                             │
│  • Sensor interfaces                                     │
└─────────────────────────────────────────────────────────┘
```

## Core Components

### 1. Control Loop (`core/control_loop.py`)

The heart of HumanoidOS - a high-frequency control loop that orchestrates all subsystems.

**Features:**
- Runs at 1kHz (configurable)
- Subsystem registration and management
- Pre/post-step callbacks for sensor reading and actuation
- Real-time metrics tracking
- Emergency stop capability
- State machine integration

**Design Philosophy:**
- Predictable, deterministic execution
- Minimal latency
- Graceful degradation on timing violations
- Comprehensive error handling

### 2. Locomotion System (`locomotion/`)

#### Balance Controller (`balance.py`)
Implements Zero Moment Point (ZMP) based balance control.

**Key Concepts:**
- **ZMP (Zero Moment Point):** The point on the ground where the total moment of ground reaction forces equals zero. Must be within support polygon for stability.
- **COM (Center of Mass) Control:** PD control to maintain desired COM position and height
- **Stability Margin:** Distance from ZMP to edge of support polygon

**Control Strategy:**
```python
# Simplified control law:
ankle_torque = -Kp * orientation_error - Kd * angular_velocity
hip_torque = -Kp * com_error - Kd * com_velocity
```

#### Gait Generator (`gait.py`)
Generates walking trajectories for bipedal locomotion.

**Gait Phases:**
1. **Double Support:** Both feet on ground (20% of cycle)
2. **Single Support:** One foot in air (80% of cycle)
   - Left swing or right swing

**Trajectory Generation:**
- **Foot trajectories:** Cycloid curves for smooth motion
- **COM trajectories:** Sinusoidal lateral sway for weight shift
- **Timing:** Configurable step length, duration, height

**Mathematical Foundation:**
```
Foot height: z(t) = h * sin(π * t/T)
COM sway:    y(t) = A * sin(π * t/T)
Forward:     x(t) = x₀ + v * t
```

### 3. Kinematics System (`kinematics/`)

**Planned Features:**
- Forward kinematics (joint angles → end effector pose)
- Inverse kinematics (desired pose → joint angles)
- Jacobian computation
- Whole-body motion planning

**Libraries:**
- Primary: Pinocchio (fast rigid body dynamics)
- Fallback: Custom implementations

### 4. Simulation Layer (`simulation/`)

#### Humanoid Robot (`humanoid.py`)
PyBullet-based simulation of humanoid robot.

**Features:**
- Realistic physics simulation
- Joint torque control
- Contact detection
- Sensor simulation (IMU, joint encoders)
- Debug visualization

**Interface:**
```python
robot = HumanoidRobot(use_gui=True, dt=0.001)
pos, euler, vel, ang_vel = robot.get_base_state()
robot.apply_joint_torques(torques)
robot.step_simulation()
```

### 5. Sensor Fusion (`sensors/`)

**Planned Features:**
- IMU integration
- Joint encoder processing
- Vision processing
- Multi-sensor fusion (Kalman filtering)

## Data Flow

```
Sensors → State Estimation → Control → Actuation → Physics → Sensors
   ↑                                                              ↓
   └──────────────────────────────────────────────────────────────┘
```

### Control Cycle (1ms @ 1kHz)

1. **Pre-step (Sensing):**
   - Read IMU data
   - Read joint encoders
   - Estimate robot state (position, velocity, orientation)
   - Update contact detection

2. **Control Computation:**
   - Gait generation (if walking)
   - Balance control
   - Trajectory tracking
   - Joint-level control

3. **Post-step (Actuation):**
   - Send joint torques
   - Step simulation/hardware
   - Log telemetry
   - Update visualizations

## State Management

### System States
```
INITIALIZING → IDLE ⟷ STANDING ⟷ WALKING
                ↓         ↓         ↓
           EMERGENCY_STOP ← FALLING
                ↓
            SHUTDOWN
```

### State Transitions
- **IDLE → STANDING:** Balance controller activated
- **STANDING → WALKING:** Gait generator starts
- **WALKING → STANDING:** Gait completes, return to stance
- **ANY → EMERGENCY_STOP:** Safety violation detected
- **ANY → FALLING:** Balance lost beyond recovery

## Safety System

### Monitoring
- Control loop timing violations
- Balance stability margin
- Joint limits
- Velocity limits
- Torque limits
- Sensor failures

### Response
- Immediate emergency stop on critical failures
- Graceful degradation for non-critical issues
- Automatic fall detection and recovery attempts

## Performance Characteristics

### Real-time Requirements
- Control frequency: 1kHz (1ms cycle time)
- Maximum cycle time: 5ms (configurable)
- Sensor latency: <0.5ms
- Actuation latency: <1ms

### Computational Complexity
- Balance control: O(1) - constant time
- Gait generation: O(1) - analytical solutions
- IK/FK: O(n) - n = number of joints
- Full body planning: O(n²) - optimization-based

## Extension Points

### Adding New Subsystems
```python
def my_subsystem(dt: float, state: SystemState):
    # Your control logic here
    return control_output

control_loop.register_subsystem("my_system", my_subsystem)
```

### Adding Sensors
```python
def read_my_sensor(dt: float, state: SystemState):
    data = sensor.read()
    # Process data
    
control_loop.add_pre_step_callback(read_my_sensor)
```

### Custom Robot Models
```python
# Load custom URDF
robot = HumanoidRobot(use_default_model=False)
robot.load_urdf("my_robot.urdf")
```

## Future Roadmap

### Phase 2 (Post-Launch)
- [ ] Perception system (computer vision)
- [ ] Manipulation planning
- [ ] Whole-body motion planning
- [ ] Terrain adaptation

### Phase 3 (Advanced Features)
- [ ] Learning-based controllers
- [ ] Multi-robot coordination
- [ ] Natural language interface
- [ ] Cloud telemetry

### Phase 4 (Production)
- [ ] Hardware drivers for commercial humanoids
- [ ] Real-time Linux integration
- [ ] Safety certification
- [ ] ROS 2 full integration

## Performance Optimization

### Current Optimizations
- NumPy vectorization
- Minimal allocations in control loop
- Efficient state representation
- Cached computations

### Future Optimizations
- JIT compilation (Numba)
- GPU acceleration (CuPy)
- C++ critical path
- SIMD operations

## Testing Strategy

### Unit Tests
- Individual component functionality
- Mathematical correctness
- Edge cases and error handling

### Integration Tests
- Subsystem interactions
- End-to-end scenarios
- Performance benchmarks

### Simulation Tests
- Balance recovery
- Walking stability
- Disturbance rejection
- Long-duration runs

## References

### Key Papers
1. Vukobratović, M., & Borovac, B. (2004). "Zero-Moment Point — Thirty Five Years of its Life"
2. Kajita, S., et al. (2003). "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point"
3. Pratt, J., et al. (2006). "Capture Point: A Step toward Humanoid Push Recovery"

### Inspiration
- OpenRAVE
- ROS (Robot Operating System)
- Drake (MIT)
- Pinocchio (LAAS-CNRS)
- PyBullet Examples

---

*Last updated: November 2024*
