# Thalassic

> These docs are mostly AI written. Proceed at your own risk.

Thalassic is the ROS2 software stack for AVBotz's **Marlin V2** AUV (Autonomous Underwater Vehicle). It targets ROS2 Jazzy on Ubuntu and uses Stonefish for physics simulation.

## Docs

- [Architecture](architecture.md) — packages, nodes, topics, TF tree
- [Control System](control.md) — cascade PID, command interface, safety, coordinate frames, thruster allocation
- [Mission System](mission.md) — BehaviorTree.CPP mission XML, movement actions, and implementation boundaries
- [Vision Terms](vision.md) — detection, align, orient, and sweep definitions for mission XML
- [Setup & Build](setup.md) — installation, building, running

## Quick Start

```bash
# First-time setup
./install.sh

# Every new shell
# For bash
source setup.sh
# Or for zsh shell
source setup.zsh

# Build
python -m colcon build

# Run simulation
ros2 launch sub_bringup sim_launch.py

# Run pool test
ros2 launch sub_bringup pool_test_launch.py
```
