# Setup & Build

Thalassic uses [pixi](https://pixi.sh) with the [RoboStack](https://robostack.github.io) channel. One manifest (`pixi.toml`) and one lockfile (`pixi.lock`) describe ROS 2 Jazzy, compilers, native libraries, and Python packages for every platform we run on.

## Prerequisites

| Platform | Notes |
|---|---|
| Ubuntu / any Linux (x86_64) | Native. Stonefish needs OpenGL 4.3 (Mesa or NVIDIA drivers on the host). |
| Jetson AGX Orin (aarch64) | Native. See [deployment.md](deployment.md) for vehicle-only OS setup. |
| Windows / MacOS | Not supported (there is no container). Use a Linux machine or VM. |

The only host tools the workspace uses outside pixi are `lsb_release` and `dpkg`, which the FLIR driver unpacks its SDK with at build time (Linux only).

**Do not install or source an apt ROS 2 on a machine that uses this workspace.** This was previously done manually with `source setup.sh`, though it is no longer needed as pixi should setup the paths properly.

## First-time setup

```bash
git clone --recurse-submodules https://github.com/avbotz/thalassic.git
cd thalassic
scripts/install.sh
```

`install.sh` installs pixi to `~/.pixi/bin` if it is missing (add that directory to your `PATH`), fetches the submodules, runs `pixi install --frozen` to materialise `pixi.lock` into `.pixi/envs/default`, runs the first `colcon build`, and downloads the model weights ([vision.md](vision.md#model-weights)). Expect several GB of downloads the first time; pixi caches packages in `~/.cache/rattler` so later workspaces are fast.

## Everyday workflow

Every `pixi run` and `pixi shell` activates the environment through `scripts/setup.sh`, which:

1. sources the `install/setup.*` overlay for your shell once the workspace has been built,
1. applies a [DDS profile](#dds-profiles),
1. points the FLIR driver at its GenTL producer once the driver has been built,
1. sets `SDL_VIDEODRIVER=x11`, so Stonefish's window keeps its title bar under Wayland.

Do not source that file by hand: it needs the pixi environment around it (the ROS underlay's hooks expand `$CONDA_PREFIX`), so it refuses to run outside one. Use `pixi shell` or `pixi run` instead.

```bash
pixi run build                        # colcon build (colcon_defaults.yaml applies)
pixi run build --packages-select sub_control
pixi run fetch-weights                # after a pull that changed weights/manifest.toml
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

`scripts/format.sh` builds its file list with `git ls-files`, which lists submodule directories but not their contents, so upstream code under `src/sub_sim/stonefish_ros2`, `src/sub_sim/stonefish_vendor/stonefish`, `src/sub_drivers/flir_camera_driver` and `src/sub_drivers/waterlinked_dvl` is never reformatted.

Run `pixi run format` before committing. If the formatter and a hand-aligned block disagree and the block is right — a lookup table, say — wrap it in `// clang-format off` / `// clang-format on`.

Packages no longer carry `ament_lint_auto` blocks, so `colcon test` (`pixi run test`) runs tests rather than a second, contradictory style check. See [Decision 9](decisions.md).

## Tests and CI

`pixi run test` runs every package's tests and exits non-zero if any fails; `pixi run colcon test-result --verbose` then shows which. They include unit tests of the controller, the mission nodes, the serial drivers (against a pretend board on a pseudo-terminal), scenario generation, and one run of the whole stack against the headless simulator ([simulation.md](simulation.md#regression-test)), which takes a minute or two. `colcon test --packages-select <pkg>` runs one package's.

pytest is pinned below 9 in `pixi.toml`: Jazzy's `launch_testing` plugins do not load under pytest 9.

`.github/workflows/ci.yml` runs `pixi run lint`, `pixi run build` and `pixi run test` on every pull request and every push to `main`, from `pixi.lock` (`--frozen`), on linux-64. Make its `build-and-test` check required for `main` in the repository settings (Branches, branch protection), together with one approving review, so nothing merges red or unread.

## DDS profiles

ROS 2 discovery is configured per machine role by `THALASSIC_DDS_PROFILE`, read by `scripts/setup.sh`:

| Profile | When | Effect |
|---|---|---|
| `dev` (default) | laptops and CI | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |
| `vehicle` | the Jetson, and a laptop plugged into the sub | `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` |

Both profiles pin `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` so every machine runs the same middleware.

Localhost-only discovery means two developers on the same Wi-Fi never see each other's `/marlin_v3` graph. Foxglove is unaffected: it connects to the bridge over a websocket, not DDS. On a shared machine, each Unix user gets a different domain so their simulations stay separate.

To talk to the sub from a laptop:

```bash
THALASSIC_DDS_PROFILE=vehicle pixi shell
ros2 topic list
```

The Jetson selects the `vehicle` profile for every login shell through `/etc/profile.d/thalassic.sh` (installed by `deploy/jetson/setup.sh`).

If nodes stop discovering each other after a crash, Fast DDS may have left shared-memory segments behind: `pixi run reset-dds`. A crashed simulator leaves 64 MB of them (its segment holds the camera images; see [simulation.md](simulation.md#performance)).

## Stonefish

The simulator library is built inside the workspace by the `stonefish_vendor` package from the git submodule at `src/sub_sim/stonefish_vendor/stonefish` (AVBotz fork, branch `claude-extra-features`). It installs to `install/opt/stonefish_vendor` and registers a forwarding `StonefishConfig.cmake` under `install/share/Stonefish/cmake`, so `stonefish_ros2` finds it through the workspace prefix with no environment hook.

If you have previously installed Stonefish globally, it is recommended to remove it.

To test another Stonefish version:

```bash
git -C src/sub_sim/stonefish_vendor/stonefish fetch
git -C src/sub_sim/stonefish_vendor/stonefish checkout <branch-or-commit>
pixi run build --packages-select stonefish_vendor stonefish_ros2
```

Commit the submodule pointer when the version should become the team default.

## GPU inference

The default environment installs CPU builds of ONNX Runtime and, on `linux-64`, of PyTorch; they are enough for the simulator.

- **Jetson:** the `linux-aarch64` environment carries JetPack 7.2.1's TensorRT (10.16.2, built for the Orin, from the isaac-forge channel) and the CUDA 13.2 runtime and bindings from conda-forge; JetPack only has to provide the driver. `sub_vision` runs TensorRT engines that `pixi run build-engines` builds from the ONNX models and checks against ONNX Runtime ([deployment.md](deployment.md#the-gpu-stack)).
- **x86 with an NVIDIA GPU:** on a machine whose driver supports CUDA 12 or newer, `pixi install` picks the `linux-64-cuda` entry of `pixi.toml` and the environment's ONNX Runtime has the CUDA provider; `device: auto` uses it. Nothing to opt into, and nothing changes for CI or machines without NVIDIA hardware, which get the plain `linux-64` entry. PyTorch stays the CPU build everywhere. The two entries are solved separately, so a developer switching machines rebuilds the workspace rather than copying `install/` across.

## Updating dependencies

```bash
pixi add ros-jazzy-<package>          # adds to pixi.toml and re-locks
pixi update                           # refresh every pin in pixi.lock
```

Commit `pixi.toml` and `pixi.lock` together. `install.sh` and the Jetson use `--frozen`, which installs `pixi.lock` as it is, without checking it against `pixi.toml`: the lockfile is the source of truth, and a manifest change that was not re-locked does not reach them.

Packages not available from RoboStack are built from source as submodules under `src/`: `stonefish_ros2`, `waterlinked_dvl`, `flir_camera_driver` (downloads the Spinnaker SDK during its build; no separate SDK install), and Stonefish itself.

## Send commands manually

`motion_setpoint` sets a mode per axis `[x, y, z, roll, pitch, yaw]` (0 keep, 1 position, 2 velocity, 3 hold, 4 effort); see [control.md](control.md#command-interface). Positions are in odom (x, y and yaw zeroed where the kill switch was released; z is depth, negative down).

```bash
# Go to 1 m forward of the arm point, 0.5 m deep (depth is absolute), level, heading 0
ros2 topic pub --once /marlin_v3/motion_setpoint sub_control_interfaces/msg/MotionSetpoint \
  '{mode: [1, 1, 1, 1, 1, 1], position: [1.0, 0.0, -0.5, 0.0, 0.0, 0.0]}'

# Turn to a yaw of 0.5 rad, leaving everything else as it is
ros2 topic pub --once /marlin_v3/motion_setpoint sub_control_interfaces/msg/MotionSetpoint \
  '{mode: [0, 0, 0, 0, 0, 1], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.5]}'

# Surge at 0.3 m/s while still holding depth and heading; stops 1 s after the stream does
ros2 topic pub -r 5 /marlin_v3/motion_setpoint sub_control_interfaces/msg/MotionSetpoint \
  '{mode: [2, 2, 0, 0, 0, 0], velocity: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0], timeout: 1.0}'

# Stop and hold wherever the vehicle comes to rest
ros2 topic pub --once /marlin_v3/motion_setpoint sub_control_interfaces/msg/MotionSetpoint \
  '{mode: [3, 3, 3, 3, 3, 3]}'
```

## Useful commands

```bash
ros2 topic list
ros2 topic echo /marlin_v3/odometry/filtered
ros2 topic echo /marlin_v3/control/error
ros2 topic echo /marlin_v3/control/status     # modes, reference, wrench, thrusters, watchdogs
ros2 run tf2_tools view_frames
ros2 topic echo /marlin_v3/sim/thruster_states
```

## Launch arguments

Accepted by both `sim_launch.py` and `pool_test_launch.py`:

| Argument | Default | Description |
|---|---|---|
| `robot_name` | `marlin_v3` | Vehicle to bring up; selects `config/vehicles/<robot_name>.yaml` |
| `mission` | `""` | Mission entrypoint to run; empty skips `sub_mission` |
| `role` | `SURVEY` | Vision model role for mission: `SURVEY` or `SEARCH` |
| `groot_port` | `5555` | Port of the mission's Groot2 live view, which takes the next port too; `0` turns it off ([mission.md](mission.md#live-monitoring-groot2)) |
| `restart` | per stack | Run the mission under `sub_mission`'s restart supervisor, which reruns it from the beginning at each kill-switch release ([mission.md](mission.md#restart-supervisor)) — `true` on the vehicle, `false` in the simulator |
| `gains` | per stack | Control profile (gains, vehicle model, trajectory limits) in `sub_bringup/config` — `control/gains.yaml` on the vehicle, `control/gains_sim.yaml` in the simulator |
| `foxglove_port` | `8765` | Websocket port for the Foxglove bridge |
| `foxglove_compression` | per stack | Deflate the bridge's messages — `true` on the vehicle, for a laptop on its network; `false` in the simulator, where Foxglove connects over localhost and compression only adds latency |
| `use_sim_time` | per stack | Run every node on the simulator's `/clock` — `false` on the vehicle, `true` in the simulator |

`record`, `bag_dir` and `jpeg_quality` are in [recording.md](recording.md).

Simulation only:

| Argument | Default | Description |
|---|---|---|
| `seed` | `""` (random) | Integer seed for scenario randomization |
| `DX` | `0.25` | Max X position fuzz applied to task objects (m) |
| `DY` | `0.25` | Max Y position fuzz (m) |
| `DZ` | `0.10` | Max Z position fuzz (m) |
| `DYAW` | `0.10` | Max yaw fuzz applied to task objects (rad) |
| `segmentation_camera` | `false` | Add the segmentation camera, whose images the bag records as training labels for remora ([simulation.md](simulation.md#cameras)) |
| `depth_camera` | `false` | Add the simulated depth camera |
| `vision_depth` | `false` | Run Depth Anything on the front camera beside the detector and publish its maps (`sub_vision/depth`, `sub_vision/depth/color`); each run delays that frame's detections, about 0.4 s on an iGPU |

`course`, `current`, `sensor_noise`, `window_width`, `window_height` and `rendering_quality` are in [simulation.md](simulation.md#simulation-settings).

`role`, `course` and `rendering_quality` refuse values outside their choices. Boolean arguments take `true`/`false` or `1`/`0`.

`ros2 launch sub_bringup sim_launch.py --show-args` prints the live list.

## Launch files

Four files, layered so the two stacks cannot drift apart:

| File | Runs |
|---|---|
| `common_launch.py` | Everything both stacks run identically: the transforms, the EKF, `sub_control`, the mission executive, the Foxglove bridge |
| `description_launch.py` | The vehicle's static transforms, from `config/vehicles/<robot_name>.yaml`. Included by `common_launch.py` |
| `pool_test_launch.py` | `common_launch.py` + the hardware drivers, the two cameras, and a `sub_vision` node and an annotation visualizer per camera |
| `sim_launch.py` | `common_launch.py` + Stonefish, the sensor bridges, `robot_state_publisher`, and a `sub_vision` node and an annotation visualizer per camera |

The two top-level files differ only in where sensor data comes from, so everything downstream of it is in `common_launch.py` and adding a node there reaches both. The only things they configure differently are the defaults of `gains`, `restart`, `foxglove_compression` and `use_sim_time`.

`common_launch.py` is also launchable on its own, against a stack whose drivers are already running — restarting the controller and the mission executive without re-enumerating the cameras:

```bash
ros2 launch sub_bringup common_launch.py mission:=pool_a restart:=true
```

On its own its defaults are the vehicle's `gains` with `restart:=false`, so pass what the top-level file would: against the simulator, `gains:=control/gains_sim.yaml use_sim_time:=true foxglove_compression:=false`. Its `record` does nothing; the top-level files start the recorder.

Launch configurations are inherited by an include, so an argument set on the command line or defaulted by the parent reaches the nodes inside without being threaded through by hand.

## The vehicle description

Anything that is a property of the physical hull — mounting offsets, thruster geometry, the serial number of the down camera, the udev names of the serial devices — lives in `src/sub_bringup/config/vehicles/<robot_name>.yaml`, not in a launch file. `description_launch.py` publishes its transforms from one `static_transforms` node; `sim_launch.py` and `pool_test_launch.py` read the rest.

Node *tuning* (control gains, EKF configuration, camera exposure) is not vehicle identity and stays in its own config file next to it.

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
│   ├── setup.sh            # pixi activation: workspace overlay, DDS profile, GenTL path
│   ├── format.sh           # pixi run format / lint
│   └── prune-symlinks.sh   # pixi run prune-symlinks
├── deploy/jetson/          # vehicle-only OS configuration
├── deploy/udev/            # device rules (installed on the Jetson)
├── src/                    # ROS 2 packages + submodules
├── build/ install/ log/    # colcon output (gitignored)
└── docs/
```
