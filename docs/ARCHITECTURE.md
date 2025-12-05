# Architecture

## Overview

HumanoidOS is a modular control stack for bipedal humanoid robots. It's designed around a real-time control loop that coordinates locomotion, balance, and simulation.

## Core Components

### Control Loop (`core/control_loop.py`)

High-frequency (1kHz) control loop that orchestrates all subsystems.

```python
# Main cycle:
1. Read sensors (IMU, encoders)
2. Update state estimation
3. Run controllers (balance, gait)
4. Apply joint torques
5. Step simulation
```

### Balance Controller (`locomotion/balance.py`)

ZMP (Zero Moment Point) based balance control.

**Key idea:** Keep the ZMP (point where ground reaction forces balance) within the support polygon (area between feet). Uses PD control to maintain desired center of mass position and orientation.

### Gait Generator (`locomotion/gait.py`, `advanced_gaits.py`)

Generates walking trajectories using cycloid foot paths and sinusoidal COM motion.

**Available gaits:**
- Forward/backward walking
- Sidestepping
- Turning
- Running (with flight phase)

### Push Recovery (`locomotion/push_recovery.py`)

Detects external disturbances and plans recovery steps using capture point dynamics.

**How it works:**
1. Monitor COM velocity and acceleration
2. Calculate capture point (where robot will fall if no action taken)
3. Plan step to new support location
4. Execute recovery motion

### Simulation (`simulation/humanoid.py`)

PyBullet-based physics simulation for testing without hardware.

## Control Flow

```
Sensors → State Estimation → Controllers → Torques → Physics → Sensors
```

Each cycle runs in ~1ms (1000 Hz).

## Extension Points

### Adding Controllers

```python
def my_controller(dt, state):
    # Your control logic
    return output

control_loop.register_subsystem("my_controller", my_controller)
```

### Custom Gaits

```python
gait = AdvancedGaitGenerator()
gait.set_gait(GaitType.SIDESTEP_LEFT, speed=0.3)
```

## Design Principles

- **Modularity**: Each component is independent
- **Real-time**: Deterministic 1kHz execution
- **Safety**: Multiple layers of safety checks
- **Extensibility**: Easy to add new controllers

## Current Limitations

- Flat terrain only (no slopes/stairs yet)
- Simulation only (no hardware drivers yet)
- No vision system
- Basic foot contact detection

## Next Steps

See roadmap in README for planned features.

---

Built on ZMP balance theory and capture point dynamics from MIT/Boston Dynamics research.
