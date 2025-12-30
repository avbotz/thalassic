# Simulation

## Overview

The simulator is [Stonefish](https://github.com/patrykcieslak/stonefish), a physics engine built for underwater vehicles. `stonefish_ros2` provides the ROS2 bridge. A custom fork (`kethan1/stonefish`, branch `fixes-merged`) is required.

Scenarios are described with Stonefish `.scn` XML files. Thalassic generates them at launch time from Jinja2 templates so task-object positions can be randomized.

## Scenario Generation Pipeline

`sim_launch.py` runs this pipeline before starting any nodes:

1. **Render robot layout** — `generate_robot.py` renders `data/robots/marlin_v2/layout.scn.j2` into a temporary `.scn` file. The template uses a `thruster.j2` macro to stamp out all 8 thruster definitions.

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

| Node | Package | Purpose |
|---|---|---|
| `stonefish_simulator` | `stonefish_ros2` | Physics engine + sensor simulation |
| `robot_state_publisher` | `robot_state_publisher` | Publishes URDF and TF from joint states |
| `sim_dvl_remapper` | `sub_sim_sensors` | `stonefish_ros2/DVL` → `marine_acoustic_msgs/Dvl` |
| `thruster_republishers` | `sub_sim_sensors` | 8 individual thruster topics → `Float64MultiArray` |

## Sensor Simulation

### DVL

Stonefish publishes `stonefish_ros2/DVL` on `sim/dvl`. `sim_dvl_remapper` converts it to `marine_acoustic_msgs/Dvl` on `dvl`, then `dvl_odom_remapper` extracts velocity and altitude for the EKF and `sub_control`.

### Thrusters

`sub_control` publishes `sim/thruster_setpoints` as a `Float64MultiArray` with 8 values (each ×400). Stonefish reads this topic directly.

The `thruster_republishers` node is a legacy bridge for when thruster commands are published as 8 individual `Float64` topics at `/marlin_v2/control/thruster_i`. It is still launched but sub_control no longer uses that path.

## Simulation Settings

Hardcoded in `sim_launch.py`:

| Setting | Value |
|---|---|
| `simulation_rate` | 300 Hz |
| `window_res_x` × `window_res_y` | 1900 × 1000 |
| `rendering_quality` | medium |
| `use_sim_time` | false |

## Task Models

3D models for competition tasks live in `src/sub_sim/sub_sim/data/models/`:

| Task | Objects |
|---|---|
| Gate | gate (sawfish / shark side images) |
| Slalom | slalom poles |
| Bin | bin with images |
| Torpedoes | board (sawfish and shark variants) |
| Octagon | octagon, table, bottles, ladles |
| Path | orange path markers |
| Pools | woollett, natatorium |

The Woollett pool model (`data/models/pools/woollett/`) is the default scenario.

## Robot Model

The Marlin V2 robot definition lives in `src/sub_sim/sub_sim/data/robots/marlin_v2/`:

- `layout.scn.j2` — Jinja2 template for the full robot Stonefish scenario (links, joints, sensors, actuators)
- `thruster.j2` — macro for a single thruster definition
- `frame/`, `dropper/`, `grabber/`, `torpedoes/`, etc. — mesh files (`.obj`)
