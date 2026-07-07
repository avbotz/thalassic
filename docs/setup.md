# Setup & Build

## Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy
- Stonefish (physics simulator, custom fork)
- GPU with OpenGL support (for Stonefish rendering; `stonefish_simulator_nogpu` exists as a fallback)

## Fresh Install

Run once on a new machine. This installs all apt dependencies, builds Stonefish from source, and runs `colcon build`.

```bash
./install.sh
```

The script clones Stonefish from `https://github.com/kethan1/stonefish` (branch `fixes-merged`) and builds it into `/usr/local`. If Stonefish is already installed, skip the relevant section manually.

## Every New Shell

```bash
source setup.zsh
```

This sources both `/opt/ros/jazzy/setup.bash` and the workspace `install/setup.sh`. Always do this before any `ros2` or `colcon` command.

## Build

```bash
# Source first, then:
colcon build
```

`colcon_defaults.yaml` is pre-configured with:
- `--symlink-install` — Python files are symlinked so edits take effect without rebuilding
- `--merge-install` — all packages install into a single `install/` tree
- `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` — generates `compile_commands.json` for clangd

To rebuild a single package:

```bash
colcon build --packages-select sub_control
```

## Run the Simulation

```bash
ros2 launch sub_bringup sim_launch.py
```

Optional arguments:

| Argument | Default | Description |
|---|---|---|
| `seed` | `""` (random) | Integer seed for scenario randomization |
| `DX` | `0.25` | Max X position fuzz applied to task objects (m) |
| `DY` | `0.25` | Max Y position fuzz (m) |
| `DZ` | `0.10` | Max Z position fuzz (m) |
| `DYAW` | `0.10` | Max yaw fuzz applied to task objects (rad) |
| `mission` | `""` | Mission entrypoint to run; empty skips `sub_mission` |
| `role` | `SURVEY` | Vision model role for mission: `SURVEY` or `SEARCH` |

Example with a fixed seed:

```bash
ros2 launch sub_bringup sim_launch.py seed:=42 DX:=0.0 DY:=0.0
```

## Send Commands Manually

```bash
# Hold position 1 m forward, 0 m lateral, 0.5 m down (FLU: dive = negative z)
ros2 topic pub /marlin_v2/pos_setpoint sub_control_interfaces/msg/Setpoint \
  '{velocity: false, altitude: false, setpoint: {x: 1.0, y: 0.0, z: -0.5}}'

# Command a yaw of 0.5 rad (relative to startup heading)
ros2 topic pub /marlin_v2/att_setpoint sub_control_interfaces/msg/Setpoint \
  '{velocity: false, setpoint: {roll: 0.0, pitch: 0.0, yaw: 0.5}}'

# Command forward velocity of 0.3 m/s directly (bypass the position loop)
ros2 topic pub /marlin_v2/pos_setpoint sub_control_interfaces/msg/Setpoint \
  '{velocity: true, altitude: false, setpoint: {x: 0.3, y: 0.0, z: 0.0}}'
```

## Useful Commands

```bash
# List all topics
ros2 topic list

# Monitor filtered odometry
ros2 topic echo /marlin_v2/odometry/filtered

# Monitor control errors
ros2 topic echo /marlin_v2/control/error

# View TF tree
ros2 run tf2_tools view_frames

# Check thruster outputs
ros2 topic echo /marlin_v2/sim/thruster_setpoints
```

## Workspace Layout

```
thalassic/
├── src/              # ROS2 packages + submodules
├── build/            # colcon build artifacts (gitignored)
├── install/          # installed packages (gitignored)
├── log/              # colcon logs (gitignored)
├── docs/             # this documentation
├── colcon_defaults.yaml
├── setup.sh / setup.zsh   # shell environment setup
└── install.sh        # one-time machine setup
```
