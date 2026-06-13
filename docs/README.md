# Thalassic

Thalassic is the ROS2 software stack for AVBotz's **Marlin V2** AUV (Autonomous Underwater Vehicle). It targets ROS2 Jazzy on Ubuntu and uses Stonefish for physics simulation.

## Docs

- [Architecture](architecture.md) — packages, nodes, topics, TF tree
- [Control System](control.md) — state feedback, safety, coordinate frames, constrained allocation
- [Low-Level Board](sub_low.md) — USB CDC ECM setup and `sub_low` lifecycle usage
- [Setup & Build](setup.md) — installation, building, running

## Quick Start

```bash
# First-time setup
./install.sh

# Every new shell
source setup.zsh

# Build
colcon build

# Run simulation
ros2 launch sub_bringup sim_launch.py
```
