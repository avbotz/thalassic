# Thalassic

ROS 2 Jazzy software stack for AVBotz's **Marlin V2** AUV. Runs on the vehicle's
Jetson AGX Orin and, with the Stonefish simulator, on developer machines.

Everything the workspace needs is declared in `pixi.toml` and pinned in
`pixi.lock`. See [docs/setup.md](docs/setup.md) for the workflow.

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
| `pixi run sim` | launch simulation |
| `pixi run pool` | launch with real hardware in pool |
| `pixi run test` | `colcon test` |
| `pixi run clean` | delete `build/ install/ log/` |
| `pixi run reset-dds` | kill stray ROS processes and stale Fast DDS shared memory |

## Docs

- [Setup & Build](docs/setup.md)irs
- [Deployment](docs/deployment.md) — Jetson provisioning
- [Decisions](docs/decisions.md)
- [Architecture](docs/architecture.md)
- [Control](docs/control.md)
- [Mission](docs/mission.md)
- [Vision](docs/vision.md)
- [Simulation](docs/simulation.md)
- [Networking](docs/networking.md)
