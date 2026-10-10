# sub_pid_tuner

A browser dashboard for tuning `sub_control` live. It edits any running node's parameters, graphs the controller's target, reference and measurement per axis, runs bounded tuning routines that send `MotionSetpoint`, and saves tuned values back to the gains file the controller was started with. [TUNING.md](TUNING.md) is the tuning procedure; [docs/control.md](../../docs/control.md) explains the controller.

## Running it

From the bringup launch files:

```bash
pixi run sim dashboard:=true    # http://127.0.0.1:8080
pixi run pool dashboard:=true   # http://<vehicle address>:8080
```

Or against a stack that is already running:

```bash
pixi run ros2 run sub_pid_tuner dashboard --ros-args -r __ns:=/marlin_v3 \
    -p profile:=src/sub_bringup/config/control/gains_sim.yaml -p use_sim_time:=true
```

| Parameter | Default | Meaning |
|---|---|---|
| `controller` | `sub_control` | The controller node, relative to the dashboard's namespace |
| `profile` | `""` | Gains file that **Save profile** writes; empty turns saving off |
| `host` | `127.0.0.1` | Address to serve on; `0.0.0.0` for other machines |
| `port` | `8080` | |

## Panels

Tiles can be dragged and resized; the layout and graph selection are kept in the browser.

- **Configuration**: pick any node, filter its parameters, edit and **Apply** them (one row, or every staged edit with `Ctrl+S`). Yellow rows are staged, blue rows differ from the gains file. `sub_control`'s parameters are grouped as in [docs/control.md](../../docs/control.md#configuration); short arrays get a field per element (`kp ki kd`, `v max a max j max`, x to yaw), and each `gains.<axis>` row has an **ω** field that fills in a critically damped `[3ω², ω³, 3ω]`.
- **Graph**: any live signal, or for one axis the presets *target / reference / measured*, *tracking error*, *velocity*, *wrench / disturbance* and *integral*. Fixed or custom Y range, zoom, pause, CSV export.
- **Control**: tuning routines (below), **Hold here** (every axis stops smoothly and holds, level), **Zero pose** (`sub_control`'s `reset_pose`), and a manual `MotionSetpoint` with a mode and value per axis.
- **Path**: the routine's waypoints, the controller's reference and the measured position, from above (XY), the side (XZ) or the front (YZ), with the heading.
- **Telemetry**: each axis's mode, the watchdog flags, thruster saturation and health, and every signal's latest value.

## Signals

Per axis `x`, `y`, `z`, `roll`, `pitch`, `yaw`, in the frames of `ControllerStatus`: translation in odom ENU (z up, so 1 m deep is z = −1) and rotation as ZYX Euler angles, yaw counter-clockwise from above.

| Signal | |
|---|---|
| `<axis>.target` | Where the axis is headed (its `POSITION` target, else the reference) |
| `<axis>.reference`, `.reference_velocity`, `.reference_acceleration` | The smooth trajectory the controller tracks |
| `<axis>.measured`, `.velocity` | From `odometry/filtered`, its body twist turned into odom and Euler rates; z is DVL altitude while holding altitude |
| `<axis>.tracking_error` | Reference minus measured: what the feedback acts on |
| `<axis>.wrench_command`, `.wrench_achieved` | Body FLU [N, N·m]: asked for, and what the thrusters produce of it |
| `<axis>.integral`, `.disturbance` | Body FLU integral action [m/s², rad/s²] and disturbance observer estimate [N, N·m] |
| `thruster_<i>.command`, `.force` | Normalized command and force [N] |
| `altitude`, `state_age` | DVL altitude [m]; age of the controller's odometry [s] |

## Routines

All start from where `sub_control` is holding (each `POSITION` axis's target, else the measured pose). The controller shapes every target into its trajectory, so a routine only says where to go and for how long. Positions and velocities are along odom axes; zero the pose first so that x is forward and y left.

| Routine | Sends |
|---|---|
| Step | `POSITION` start + step, start, start − step, start on one axis, holding each |
| Cruise | `VELOCITY` at the speed, `HOLD`, then the same the other way, on x, y, z or yaw; streamed with a 0.5 s timeout |
| Hold | `POSITION` at the start on every axis, for a disturbance test |
| Square | `POSITION` through the corners of a square in odom x/y, forward then left |

A routine starts only with live `odometry/filtered` and `control/status`, the vehicle unkilled and not in failsafe, and no other publisher on `motion_setpoint` or `cmd_vel` (a mission, teleop, `identify_model`) and no second controller or EKF. It aborts when any of that stops being true. **Stop**, **Hold here**, an abort, closing the browser tab that started it, or stopping the dashboard all leave every axis stopping smoothly and holding, level; a kill aborts without sending anything. Routines that move z keep the target at least 0.3 m deep. Manual `VELOCITY` and `EFFORT` commands need a timeout of at most 30 s.

## Saving

- **Apply** sets the staged parameters of each node atomically and reads them back; a row clears only once the readback matches.
- **Save profile** (`Ctrl+Shift+S`) applies any staged edits, reads `sub_control`'s parameters, and writes each that the gains file sets and that changed into the file, in place: comments, layout and every other value stay as they are. `thruster_health` and `failed_thrusters` are fault state and are never saved. It refuses if the file changed since the dashboard read it; **Reload** rereads it.
- With the workspace's `--symlink-install`, the installed gains file links to `src/sub_bringup/config/control/`, so the change lands in the source tree. Review it with `git diff`.
