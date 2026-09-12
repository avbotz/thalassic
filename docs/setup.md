# Setup & Build

Thalassic uses [pixi](https://pixi.sh) with the [RoboStack](https://robostack.github.io) channel. One manifest (`pixi.toml`) and one lockfile (`pixi.lock`) describe ROS 2 Jazzy, compilers, native libraries, and Python packages for every platform we run on.

## Prerequisites

| Platform | Notes |
|---|---|
| Ubuntu / any Linux (x86_64) | Native. Stonefish needs OpenGL 4.3 (Mesa or NVIDIA drivers on the host). |
| Jetson AGX Orin (aarch64) | Native. See [deployment.md](deployment.md) for vehicle-only OS setup. |
| Windows / MacOS | Use Docker. |

The only host tools the workspace uses outside pixi are `lsb_release` and `dpkg`, which the FLIR driver unpacks its SDK with at build time (Linux only).

**Do not install or source an apt ROS 2 on a machine that uses this workspace.** This was previously done manually with `source setup.sh`, though it is no longer needed as pixi should setup the paths properly.

## First-time setup

```bash
git clone --recurse-submodules https://github.com/avbotz/thalassic.git
cd thalassic
scripts/install.sh
```

`install.sh` installs pixi to `~/.pixi/bin` if it is missing (add that directory to your `PATH`), fetches the submodules, runs `pixi install --frozen` to materialise `pixi.lock` into `.pixi/envs/default`, and runs the first `colcon build`. Expect several GB of downloads the first time; pixi caches packages in `~/.cache/rattler` so later workspaces are fast.

## Everyday workflow

Every `pixi run` and `pixi shell` activates the environment through `scripts/setup.sh`, which:

1. sources the `install/setup.*` overlay for your shell once the workspace has been built,
2. applies a [DDS profile](#dds-profiles),
3. points the FLIR driver at its GenTL producer once the driver has been built.

Do not source that file by hand: it needs the pixi environment around it (the ROS underlay's hooks expand `$CONDA_PREFIX`), so it refuses to run outside one. Use `pixi shell` or `pixi run` instead.

```bash
pixi run build                        # colcon build (colcon_defaults.yaml applies)
pixi run build --packages-select sub_control
pixi run sim                          # ros2 launch sub_bringup sim_launch.py
pixi run sim seed:=42 DX:=0.0 mission:=pid_tuning
pixi shell                            # then use ros2 / colcon / rviz2 directly
pixi run test
pixi run clean
```

`colcon_defaults.yaml` configures `--symlink-install` (Python edits take effect without rebuilding), `--merge-install` (a single `install/` tree), compile commands for clangd, and the RoboStack-recommended flags that make CMake find the environment's Python rather than the system one.

### Shell completion

`pixi shell` starts your own shell (`bash` or `zsh`) with the environment exported. For `colcon` and `ros2` tab completion inside it:

```bash
source $CONDA_PREFIX/share/colcon_argcomplete/hook/colcon-argcomplete.zsh   # or .bash
```

### Editors

`pyrightconfig.json` points at the environment's `site-packages`; `compile_commands.json` lands in `build/`. VS Code users can install the `prefix-dev.pixi-vscode` extension or select `.pixi/envs/default/bin/python` as the interpreter.

## Formatting and linting

```bash
pixi run format     # rewrite every file this repo owns
pixi run lint       # check only; non-zero exit if anything is off
```

Two tools, one per language, both pinned in `pixi.lock` so a rebuild never reformats the tree under someone:

| Language | Tool | Config |
|---|---|---|
| Python | `ruff` (formatter + linter) | `ruff.toml` — 100 columns, `py312` |
| C / C++ | `clang-format` | `.clang-format` — 4-space indent, 120 columns |

`scripts/format.sh` builds its file list with `git ls-files`, which lists submodule directories but not their contents, so upstream code under `src/stonefish_ros2`, `src/sub_drivers/flir_camera_driver` and `src/sub_drivers/waterlinked_dvl` is never reformatted.

Run `pixi run format` before committing. If the formatter and a hand-aligned block disagree and the block is right — a lookup table, say — wrap it in `// clang-format off` / `// clang-format on`.

Packages no longer carry `ament_lint_auto` blocks, so `colcon test` (`pixi run test`) runs tests rather than a second, contradictory style check. See [Decision 9](decisions.md).

## DDS profiles

ROS 2 discovery is configured per machine role by `THALASSIC_DDS_PROFILE`, read by `scripts/setup.sh`:

| Profile | When | Effect |
|---|---|---|
| `dev` (default) | laptops, CI, containers | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; `ROS_DOMAIN_ID` derived from your Unix user id (override with `THALASSIC_ROS_DOMAIN_ID`) |
| `vehicle` | the Jetson, and a laptop plugged into the sub | `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`; fixed `ROS_DOMAIN_ID=42`; optional `THALASSIC_ROS_STATIC_PEERS` for links that block multicast |

Both profiles pin `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` so every machine runs the same middleware.

Localhost-only discovery means two developers on the same Wi-Fi never see each other's `/marlin_v3` graph. Foxglove is unaffected: it connects to the bridge over a websocket, not DDS. On a shared machine, each Unix user gets a different domain so their simulations stay separate.

To talk to the sub from a laptop:

```bash
THALASSIC_DDS_PROFILE=vehicle pixi shell
ros2 topic list
```

The Jetson selects the `vehicle` profile for every login shell through `/etc/profile.d/thalassic.sh` (installed by `deploy/jetson/setup.sh`).

If nodes stop discovering each other after a crash, Fast DDS may have left shared-memory segments behind: `pixi run reset-dds`.

## Stonefish

The simulator library is built inside the workspace by the `stonefish_vendor` package from the git submodule at `src/sub_sim/stonefish_vendor/stonefish` (AVBotz fork, branch `fixes-merged`). It installs to `install/opt/stonefish_vendor` and is found by `stonefish_ros2` through an ament environment hook.

If you have previously installed Stonefish globally, it is recommended to remove it.

To test another Stonefish version:

```bash
git -C src/sub_sim/stonefish_vendor/stonefish fetch
git -C src/sub_sim/stonefish_vendor/stonefish checkout <branch-or-commit>
pixi run build --packages-select stonefish_vendor stonefish_ros2
```

Commit the submodule pointer when the version should become the team default.

## GPU inference

The default environment installs CPU builds of PyTorch and ONNX Runtime; they are enough for the simulator and for the color-correction node's CPU fallback.

- **Jetson:** TensorRT and the CUDA runtime bindings come from JetPack, not from pixi. `deploy/jetson/link_jetpack_python.sh` exposes them to the environment (see [deployment.md](deployment.md)). `sub_vision` then uses `<task>.engine` files as before.
- **x86 with NVIDIA:** not wired up yet. conda-forge ships CUDA builds of `pytorch` and `onnxruntime`; enabling them means declaring a CUDA-capable platform in `pixi.toml` and rebuilding. Open item.

## Container

`docker/Dockerfile` and `.devcontainer/devcontainer.json` provide a Linux userland. Useful if running MacOS / Windows.

Open the folder in VS Code and choose *Reopen in Container*, or:

```bash
docker build -f docker/Dockerfile -t thalassic .
docker run --rm -it --net=host --ipc=host --device=/dev/dri \
  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PWD:/home/ubuntu/thalassic" thalassic
```

## Updating dependencies

```bash
pixi add ros-jazzy-<package>          # adds to pixi.toml and re-locks
pixi update                           # refresh every pin in pixi.lock
```

Commit `pixi.toml` and `pixi.lock` together. `pixi install --frozen` (used by `install.sh` and the Jetson) refuses to run if the two disagree, which is the point: the lockfile is the source of truth.

Packages not available from RoboStack are built from source as submodules under `src/`: `stonefish_ros2`, `waterlinked_dvl`, `flir_camera_driver` (downloads the Spinnaker SDK during its build; no separate SDK install), and Stonefish itself.

## Send commands manually

```bash
# Hold position 1 m forward, 0 m lateral, 0.5 m down (FLU: dive = negative z)
ros2 topic pub /marlin_v3/pos_setpoint sub_control_interfaces/msg/Setpoint \
  '{velocity: false, altitude: false, setpoint: {x: 1.0, y: 0.0, z: -0.5}}'

# Command a yaw of 0.5 rad (relative to startup heading)
ros2 topic pub /marlin_v3/att_setpoint sub_control_interfaces/msg/Setpoint \
  '{velocity: false, setpoint: {roll: 0.0, pitch: 0.0, yaw: 0.5}}'

# Command forward velocity of 0.3 m/s directly (bypass the position loop)
ros2 topic pub /marlin_v3/pos_setpoint sub_control_interfaces/msg/Setpoint \
  '{velocity: true, altitude: false, setpoint: {x: 0.3, y: 0.0, z: 0.0}}'
```

## Useful commands

```bash
ros2 topic list
ros2 topic echo /marlin_v3/odometry/filtered
ros2 topic echo /marlin_v3/control/error
ros2 run tf2_tools view_frames
ros2 topic echo /marlin_v3/sim/thruster_setpoints
```

## Launch arguments

Accepted by both `sim_launch.py` and `pool_test_launch.py`:

| Argument | Default | Description |
|---|---|---|
| `robot_name` | `marlin_v3` | Vehicle to bring up; selects `config/<robot_name>.yaml` |
| `mission` | `""` | Mission entrypoint to run; empty skips `sub_mission` |
| `role` | `SURVEY` | Vision model role for mission: `SURVEY` or `SEARCH` |
| `gains` | per stack | PID gains file in `sub_bringup/config` — `control_gains.yaml` on the vehicle, `control_gains_sim.yaml` in the simulator |
| `foxglove_port` | `8765` | Websocket port for the Foxglove bridge |

Simulation only:

| Argument | Default | Description |
|---|---|---|
| `seed` | `""` (random) | Integer seed for scenario randomization |
| `DX` | `0.25` | Max X position fuzz applied to task objects (m) |
| `DY` | `0.25` | Max Y position fuzz (m) |
| `DZ` | `0.10` | Max Z position fuzz (m) |
| `DYAW` | `0.10` | Max yaw fuzz applied to task objects (rad) |
| `labeling` | `false` | Write labelled training images from the segmentation camera |

`ros2 launch sub_bringup sim_launch.py --show-args` prints the live list.

## Launch files

Four files, layered so the two stacks cannot drift apart:

| File | Runs |
|---|---|
| `common_launch.py` | Everything both stacks run identically: the transforms, the EKF, `sub_control`, the mission executive, the Foxglove bridge |
| `description_launch.py` | The vehicle's static transforms, from `config/<robot_name>.yaml`. Included by `common_launch.py` |
| `pool_test_launch.py` | `common_launch.py` + the hardware drivers, the two cameras, and one `sub_vision` |
| `sim_launch.py` | `common_launch.py` + Stonefish, the sensor bridges, `robot_state_publisher`, two `sub_vision` nodes and the annotation visualizer |

The two top-level files differ only in where sensor data comes from, so everything downstream of it is in `common_launch.py` and adding a node there reaches both. The only thing they configure differently is `gains`.

`common_launch.py` is also launchable on its own, against a stack whose drivers are already running — restarting the controller and the mission executive without re-enumerating the cameras:

```bash
ros2 launch sub_bringup common_launch.py mission:=pool_a
```

Launch configurations are inherited by an include, so an argument set on the command line or defaulted by the parent reaches the nodes inside without being threaded through by hand.

## The vehicle description

Anything that is a property of the physical hull — mounting offsets, thruster geometry, the serial number of the down camera, the udev names of the serial devices — lives in `src/sub_bringup/config/<robot_name>.yaml`, not in a launch file. `description_launch.py` turns its transforms into static transform publishers; `sim_launch.py` and `pool_test_launch.py` read the rest.

Node *tuning* (PID gains, EKF configuration, camera exposure) is not vehicle identity and stays in its own config file next to it.

`sub_mission` started by hand needs the stack's namespace:

```bash
ros2 run sub_mission restart --ros-args -r __ns:=/marlin_v3 -p mission:=pid_tuning
```

## Workspace layout

```
thalassic/
├── pixi.toml / pixi.lock   # the environment: ROS 2, compilers, libraries, Python
├── colcon_defaults.yaml    # colcon flags
├── colcon.meta             # build order and flags for submodules we cannot edit
├── scripts/
│   ├── install.sh          # first-time setup
│   └── setup.sh            # pixi activation: workspace overlay, DDS profile, GenTL path
├── deploy/jetson/          # vehicle-only OS configuration
├── deploy/udev/            # device rules (installed on the Jetson)
├── src/                    # ROS 2 packages + submodules
├── build/ install/ log/    # colcon output (gitignored)
└── docs/
```
