# Thalassic

ROS 2 Jazzy software stack for AVBotz's Marlin V3 AUV.

**The documentation is at https://sw.avbotz.com.** New to the team? [Start here](https://sw.avbotz.com/start-here/). Looking for details? See the [reference](https://sw.avbotz.com/thalassic/). The docs' source is in [avbotz/software-docs](https://github.com/avbotz/software-docs).

## Quick start

```bash
git clone --recurse-submodules https://github.com/avbotz/thalassic.git
cd thalassic
scripts/install.sh

pixi run up                 # Stonefish simulation
pixi shell                  # interactive shell with ROS 2 + workspace
```

Common tasks (`pixi task list` shows all):

| Command | Does |
|---|---|
| `pixi run build` | `colcon build` with the repo defaults (symlink + merge install) |
| `pixi run up` | launch simulation; launch arguments go after `--`, as in `pixi run up -- mission:=pid_tuning` |
| `pixi run -e jetson up pool_test` | launch with real hardware in pool (on the vehicle, in its own environment) |
| `pixi run preflight` | check the running stack before it goes in the water (`--sim` for the simulator) |
| `pixi run test` | every package's tests, including a headless-simulator run of the whole stack |
| `pixi run format` | format Python (ruff), C/C++ (clang-format) and shell (shfmt) |
| `pixi run lint` | check formatting and lint without rewriting |
| `pixi run clean` | delete `build/ install/ log/` |
| `pixi run reset` | kill stray ROS processes and stale Fast DDS shared memory |
