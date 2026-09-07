# Thalassic

> These docs are mostly AI written. Proceed at your own risk.

Thalassic is the ROS2 software stack for AVBotz's **Marlin V2** AUV (Autonomous Underwater Vehicle). It targets ROS2 Jazzy (from RoboStack, via pixi) and uses Stonefish for physics simulation.

## Docs

- [Setup & Build](setup.md) — pixi workflow, DDS profiles, container, GPUs
- [Deployment](deployment.md) — Jetson provisioning and updates
- [Decisions](decisions.md) — rationale for the tooling choices
- [Architecture](architecture.md) — packages, nodes, topics, TF tree
- [Control System](control.md) — cascade PID, command interface, safety, coordinate frames, thruster allocation
- [Mission System](mission.md) — BehaviorTree.CPP mission XML, movement actions, and implementation boundaries
- [Vision Terms](vision.md) — detection, align, orient, and sweep definitions for mission XML
- [Simulation](simulation.md) — scenario generation, sim nodes, sensor bridges
- [Networking](networking.md) — vehicle addresses

## Quick Start

```bash
# First-time setup (installs pixi if needed, creates the environment, builds)
scripts/install.sh

# Build
pixi run build

# Run simulation
pixi run sim

# Run pool test (vehicle)
pixi run pool

# Interactive shell with ROS 2, the workspace, and the DDS profile set up
pixi shell
```
