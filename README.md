# Thalassic

ROS 2 Jazzy software stack for AVBotz's Marlin V3 AUV.

## Quick start

```bash
git clone --recurse-submodules https://github.com/avbotz/thalassic.git
cd thalassic
scripts/install.sh

pixi run sim                # Stonefish simulation
pixi shell                  # interactive shell with ROS 2 + workspace
```

Common tasks (`pixi task list` shows all):

| Command | Does |
|---|---|
| `pixi run build` | `colcon build` with the repo defaults (symlink + merge install) |
| `pixi run prune-symlinks` | removes the dangling symlinks that deleted files leave in `build/` and `install/` |
| `pixi run sim` | launch simulation |
| `pixi run pool` | launch with real hardware in pool |
| `pixi run test` | `colcon test` |
| `pixi run format` | format Python (ruff) and C/C++ (clang-format) |
| `pixi run lint` | check formatting and lint without rewriting |
| `pixi run clean` | delete `build/ install/ log/` |
| `pixi run reset-dds` | kill stray ROS processes and stale Fast DDS shared memory |

## Docs

- [Setup & Build](docs/setup.md)
- [Deployment](docs/deployment.md)
- [Decisions](docs/decisions.md)
- [Architecture](docs/architecture.md)
- [Control](docs/control.md)
- [Mission](docs/mission.md)
- [Vision](docs/vision.md)
- [Simulation](docs/simulation.md)
- [Recording](docs/recording.md)
- [Networking](docs/networking.md)
