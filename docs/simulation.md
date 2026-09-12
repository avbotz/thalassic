# Simulation

## Overview

The simulator is [Stonefish](https://github.com/patrykcieslak/stonefish), a physics engine built for underwater vehicles. `stonefish_ros2` provides the ROS2 bridge. A custom fork (`avbotz/stonefish`, branch `fixes-merged`) is required; it is built inside the workspace by the `stonefish_vendor` package from the submodule at `src/sub_sim/stonefish_vendor/stonefish`, so no global install is needed (see [setup.md](setup.md#stonefish) for switching versions).

Scenarios are described with Stonefish `.scn` XML files. Thalassic generates them at launch time from Jinja2 templates so task-object positions can be randomized.

## Scenario Generation Pipeline

`sim_launch.py` runs this pipeline before starting any nodes:

1. **Render robot layout** — `generate_robot.py` renders `data/robots/marlin_v3/layout.scn.j2` into a temporary `.scn` file. The template uses a `thruster.j2` macro to stamp out all 8 thruster definitions.

2. **Convert to URDF** — `robot_scenario_to_urdf.py` parses the rendered robot `.scn` and emits a URDF for `robot_state_publisher`. This is used by RViz and `tf2` for visualization.

3. **Render pool scenario** — `randomize_locs.py` renders `scenarios/woollett.scn.j2` with the `DX/DY/DZ/DYAW` fuzz arguments, producing the final simulation scenario that Stonefish loads.

## Scenario Template Variables

The Jinja2 pool scenario (`woollett.scn.j2`) has access to these helpers:

| Variable / Function | Description |
|---|---|
| `fuzz(base, mag)` | `base ± uniform(mag)` |
| `fuzz_z(base, mag)` | Same but clamped to ≥ 0 |
| `rand(a, b)` | `uniform(a, b)` |
| `choose(options)` | Pick one item at random |
| `DX`, `DY`, `DZ`, `DYAW` | Per-axis fuzz magnitudes passed from launch args |
| `ROBOT_SCENARIO_PATH` | Path to the rendered robot `.scn` to `<include>` |
| `PI` | `math.pi` |

Use `seed` in the launch argument to get reproducible scenarios.

## Simulation Nodes

The following nodes are launched in simulation mode (in addition to the real-hardware nodes):

The sim sensor bridges run together inside a single composable-node container (`sim_sensors_container`):

| Node | Package | Purpose |
|---|---|---|
| `stonefish_simulator` | `stonefish_ros2` | Physics engine + sensor simulation |
| `robot_state_publisher` | `robot_state_publisher` | Publishes URDF and TF from joint states |
| `sim_dvl_remapper` | `sub_sim_sensors` | `stonefish_ros2/DVL` → `odometry/dvl` (velocity only) |
| `sim_imu_remapper` | `sub_sim_sensors` | Stonefish NED IMU → ENU `imu/data` |
| `sim_thruster_republisher` | `sub_sim_sensors` | 8 thruster topics → `sim/thruster_setpoints` array |
| `sim_kill_switch` | `sub_sim_sensors` | `sim/kill_switch` → `kill_switch` after startup delay |
| `sim_torpedo_launcher` | `sub_sim_sensors` | Torpedo launch service |
| `sim_dropper` | `sub_sim_sensors` | Dropper service |

## Sensor Simulation

### DVL

Stonefish publishes `stonefish_ros2/DVL` on `sim/dvl`. `sim_dvl_remapper` converts it directly to a velocity-only `nav_msgs/Odometry` on `odometry/dvl` — the same topic the hardware WaterLinked driver publishes — which the EKF fuses.

### Thrusters

`sub_control` publishes 8 normalized `Float64` commands on `control/thruster_0` through `control/thruster_7`. `sim_thruster_republisher` combines them into a `Float64MultiArray` on `sim/thruster_setpoints`, scaling each command by 400 (T200 count), and publishes the array consumed by Stonefish.

## Simulation Settings

Fixed in `sim_launch.py`:

| Setting | Value |
|---|---|
| `simulation_rate` | 500 Hz |
| `window_res_x` × `window_res_y` | 1900 × 1000 |
| `rendering_quality` | medium |

## Task Models

3D models for competition tasks live in `src/sub_sim/sub_sim/data/models/`:

| Task | Objects |
|---|---|
| Gate | gate (SURVEY / SEARCH role variants) |
| Slalom | slalom poles |
| Bin | bin with images |
| Torpedoes | board (SURVEY / SEARCH role variants) |
| Octagon | octagon, table, bottles, ladles |
| Path | orange path markers |
| Pools | woollett, natatorium |

The Woollett pool model (`data/models/pools/woollett/`) is the default scenario.

## Robot Model

The Marlin V3 robot definition lives in `src/sub_sim/sub_sim/data/robots/marlin_v3/`:

- `layout.scn.j2` — Jinja2 template for the full robot Stonefish scenario (links, joints, sensors, actuators)
- `thruster.j2` — macro for a single thruster definition
- `frame/`, `dropper/`, `grabber/`, `torpedoes/`, etc. — mesh files (`.obj`)

The eight thruster poses in `layout.scn.j2` are the same numbers as the `thrusters:` block of `src/sub_bringup/config/marlin_v3.yaml` (the TF tree) and the allocation matrix in `sub_control/src/utils.cpp`. Stonefish parses the scenario before any node starts and the allocator's geometry is a compile-time constant, so neither can read the vehicle description; a change to the hull is a change to all three. See [Decision 7](decisions.md).
