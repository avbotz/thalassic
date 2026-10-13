# Architecture

## Package Overview

```
src/
├── sub_bringup/                 # Launch files and config
├── sub_control/
│   ├── sub_control/             # Control algorithm, arming and watchdogs
│   └── sub_control_interfaces/  # ROS2 messages for control
├── sub_mission/                 # BehaviorTree.CPP mission execution
├── sub_drivers/
│   ├── flir_camera_driver/      # FLIR / Spinnaker camera driver (submodule)
│   ├── sub_driver_interfaces/   # Dropper, torpedo and (sim only) grabber service definitions
│   ├── sub_serial_drivers/      # Serial-device nodes (maritime low-level, IMU)
│   └── waterlinked_dvl/         # WaterLinked DVL driver (submodule)
├── sub_sim/
│   ├── sub_sim/                 # Scenario generation, robot description
│   ├── sub_sim_sensors/         # Sim sensor bridges (DVL, IMU, depth, thrusters, actuators)
│   ├── stonefish_vendor/        # Builds the Stonefish library from its submodule
│   └── stonefish_ros2/          # Stonefish simulator ROS2 bridge (submodule)
└── sub_vision/
    ├── sub_vision/              # Detection node and post-processors
    └── sub_vision_interfaces/   # Detection messages, load_model service
```

## Nodes

### `sub_control` (`sub_control/sub_control`)

The main control node, at 50 Hz: a jerk-limited trajectory generator, a model-based control law (feedforward from the vehicle model plus full-state feedback, the structure NUS Bumblebee uses) and a box-constrained QP thrust allocator that works around failed thrusters and saturation. It also owns arming and the watchdogs that decide which axes have feedback and whether the thrusters may run at all. See [control.md](control.md).

| | Topic | Type |
|---|---|---|
| Sub | `motion_setpoint` | `sub_control_interfaces/MotionSetpoint` (mode per axis, with an `id` it acknowledges) |
| Sub | `cmd_vel` | `geometry_msgs/Twist` (stops if the stream does) |
| Sub | `odometry/filtered` | `nav_msgs/Odometry` |
| Sub | `imu/data` | `sensor_msgs/Imu` (rate feedback) |
| Sub | `odometry/dvl`, `odometry/depth` | `nav_msgs/Odometry` (watchdogs only) |
| Sub | `altitude` | `sensor_msgs/Range` (DVL altitude) |
| Sub | `kill_switch` | `std_msgs/Bool` |
| Sub | `/tf`, `/tf_static` | `tf2_msgs/TFMessage` (the IMU's mounting) |
| Pub | `control/thrusters` | `sub_control_interfaces/ThrusterCommand` (all eight, stamped, normalized, capped at ±0.6) |
| Pub | `control/error` | `sub_control_interfaces/Error` (with the id of the last command taken up) |
| Pub | `control/status` | `sub_control_interfaces/ControllerStatus` |
| Pub | `control/armed` | `std_msgs/Bool`, latched |
| Pub | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` (arming, sensors, thrusters) |
| Client | `set_pose` | `robot_localization/SetPose` |
| Service | `~/reset_pose` | `std_srvs/Trigger` ([re-zeroes the pose](control.md#re-zeroing-the-pose)) |

The thruster commands go out every cycle, `control/error` and `control/status` every cycle except while the node waits for the EKF to confirm the pose reset after a kill-switch release. Gains, the vehicle model and trajectory limits come from the hardware or simulation control profile; the thruster layout from the vehicle description. Parameters are listed in [control.md](control.md#configuration).

Driving, tuning and watching the vehicle from a laptop is done in Foxglove, through the bridge, with the panels in [avbotz/foxglove-extensions](https://github.com/avbotz/foxglove-extensions): teleop (6-DOF Teleop, Motion Setpoint), tuning (Controller Gains, Tuning Routines, Controller Signals) and vision (Detections, drawn on the frames they were computed from).

### `sub_mission` (`sub_mission/mission`)

Runs the selected mission as BehaviorTree.CPP XML. See [mission.md](mission.md) for behavior tree structure, movement nodes, and implementation boundaries. On the vehicle it runs under `sub_mission/restart`, which stops it when the kill switch is pulled and starts it again on release (`restart` launch argument).

| | Topic | Type |
|---|---|---|
| Sub | `kill_switch` | `std_msgs/Bool` |
| Sub | `control/armed` | `std_msgs/Bool` (the run starts only once sub_control is armed) |
| Sub | `control/error` | `sub_control_interfaces/Error` |
| Sub | `odometry/filtered` | `nav_msgs/Odometry` |
| Sub | `vision/detections`, `vision/detections_down` | `sub_vision_interfaces/DetectionArray` |
| Sub | `/tf_static` | `tf2_msgs/TFMessage` (the vehicle's mounts, for `NavigateToTransform`) |
| Pub | `motion_setpoint` | `sub_control_interfaces/MotionSetpoint` |
| Client | `sub_vision/load_model`, `sub_vision_down/load_model` | `sub_vision_interfaces/LoadModel` |
| Client | `set_dropper` | `sub_driver_interfaces/SetDropper` |
| Client | `launch_torpedo` | `sub_driver_interfaces/LaunchTorpedo` |

### `waterlinked_dvl_driver` (`waterlinked_dvl`) — hardware only

Driver for the WaterLinked A50 DVL (the team's fork, at commit `7163fbb` or later). Its `~/odom` output is remapped to `odometry/dvl` (velocity-only `nav_msgs/Odometry`) for the EKF: one message per bottom-track velocity report the DVL marks valid, stamped with the report's `time_of_validity` (the DVL's clock, synced to the Jetson's by chrony); invalid and water-track reports are dropped. Its `~/altitude` (`sensor_msgs/Range`) is remapped to `altitude` for altitude hold. An older driver (before `7163fbb`) published invalid velocities, published each one twice and had no altitude.

### `sub_low` / `naviguider_imu_driver` (`sub_serial_drivers`) — hardware only

`sub_low` talks to the [maritime](https://github.com/avbotz/maritime) firmware over `/dev/maritime`: `control/thrusters` out, as one `m` line per cycle once the board's heartbeat says it speaks protocol 2 (eight `p` lines before then, so older firmware keeps working); `kill_switch` and `odometry/depth` in; and the `launch_torpedo` and `set_dropper` services, which each pick one of two (`torpedo_id`, `dropper_id`: 0 or 1). It stops every thruster if `sub_control` goes quiet for `command_timeout` (0.5 s), ignores a thruster command older than that, and reopens the port if the link drops (stopping the thrusters first). The board has its own 0.5 s command watchdog and a hardware watchdog, so the thrusters stop even if `sub_low` or the USB link dies ([control.md](control.md#safety)). `naviguider_imu_driver` reads the NaviGuider IMU over `/dev/naviguider_imu` and publishes ENU `imu/data` with `frame_id` `marlin_v3/imu_link`, but only while its gyro and orientation are both fresh, so a sensor that stops makes `imu/data` stop instead of repeating; it too reopens the port if the link drops. Both publish `/diagnostics`.

### `sim_dvl_remapper` (`sub_sim_sensors`) — sim only

Converts Stonefish's proprietary DVL message into the velocity-only `nav_msgs/Odometry` the EKF expects and the altitude, matching the hardware DVL topics. Readings without bottom lock are dropped, as the driver drops invalid ones. Like the real A50, each reading arrives `latency` (0.15 s) after its stamp, and `dropout_probability` (0 by default) of them can be dropped; both are settable at runtime.

| | Topic | Type |
|---|---|---|
| Sub | `sim/dvl` | `stonefish_ros2/DVL` |
| Pub | `odometry/dvl` | `nav_msgs/Odometry` (velocity only) |
| Pub | `altitude` | `sensor_msgs/Range` |

### `sim_thrusters` (`sub_sim_sensors`) — sim only

Stands in for `sub_low`, the firmware and the ESCs: passes `control/thrusters` to Stonefish (`sim/thruster_setpoints`) through the ESC's response lag, stops every thruster when the commands stop or go stale for `command_timeout` (0.5 s), and injects faults (`effectiveness`). See [simulation.md](simulation.md#thrusters).

### `sim_imu_remapper` (`sub_sim_sensors`) — sim only

Re-expresses Stonefish's NED IMU stream as ENU and stamps it with `imu_link`.

| | Topic | Type |
|---|---|---|
| Sub | `sim/imu` | `sensor_msgs/Imu` (NED) |
| Pub | `imu/data` | `sensor_msgs/Imu` (ENU) |

### `sim_pressure_to_depth` (`sub_sim_sensors`) — sim only

Converts Stonefish's gauge pressure into depth as `sub_low` reports it on the vehicle: ENU z in `odom` (negative underwater), child frame `pressure_link`.

| | Topic | Type |
|---|---|---|
| Sub | `sim/pressure` | `sensor_msgs/FluidPressure` |
| Pub | `odometry/depth` | `nav_msgs/Odometry` (z only) |

### `sim_kill_switch` (`sub_sim_sensors`) — sim only

Starts killed, releases the kill switch after a startup delay (`off_delay`, 6 s in `sim_launch.py`), then passes `sim/kill_switch` through to `kill_switch`, delaying releases by the ESCs' start-up time (`esc_startup`, 3 s).

### `sim_torpedo_launcher` / `sim_dropper` / `sim_grabber` (`sub_sim_sensors`) — sim only

The actuators in simulation: `launch_torpedo` and `set_dropper`, as `sub_low` serves them on the vehicle, and `set_grabber` (`sub_driver_interfaces/SetGrabber`), which closes the grabber's jaws, holding whatever is between them, or opens them to let go. The vehicle has no grabber service yet.

### `ekf_filter_node` (`robot_localization`)

Fuses the IMU, the DVL and the pressure sensor into a filtered odometry estimate, with one configuration (`config/ekf.yaml`) for the vehicle and the simulator.

- `imu0`: `imu/data` — yaw, relative to where it started, and yaw rate
- `imu1`: `imu/data` — roll and pitch, absolute, and their rates
- `odom0`: `odometry/dvl` — linear velocity, gated at 5 standard deviations
- `odom1`: `odometry/depth` — z, from the pressure sensor, gated at 5 standard deviations
- Output: `odometry/filtered` (and the `odom` → `base_link` transform), and `/diagnostics`

`smooth_lagged_data` with a 1 s history fuses the DVL's late velocities at their stamps instead of as if they were current. The IMU is not gated: it is the only attitude source, and a filter that rejected it could not recover.

### `diagnostic_aggregator` (`diagnostic_aggregator`)

Groups every node's `/diagnostics` into `/diagnostics_agg` (Control, Estimation, Drivers, Vision; `config/diagnostics.yaml`), and marks an item that stops reporting as stale, so a dead driver shows up even though it can no longer say so. Foxglove's diagnostics panel and `ros2 run sub_bringup preflight` read it.

## TF Tree

```
map
└── marlin_v3/odom          (static, identity)
    └── marlin_v3/base_link  (published by EKF; ENU/FLU control frame)
        ├── marlin_v3/imu_link            (relative to base_link)
        └── marlin_v3/base_link_ned       (roll=π, yaw=−π/2 — static mounting frame)
            ├── marlin_v3/pressure_link
            ├── marlin_v3/front_camera
            ├── marlin_v3/down_camera
            ├── marlin_v3/dvl_link
            ├── marlin_v3/dropper_link
            ├── marlin_v3/left_grabber_link
            ├── marlin_v3/right_grabber_link
            ├── marlin_v3/torpedo_0_link
            ├── marlin_v3/torpedo_1_link
            ├── marlin_v3/thruster_0_link
            ├── ...
            └── marlin_v3/thruster_7_link
```

Control runs in `base_link` (ENU/FLU). `base_link_ned` is a static mounting frame — most sensor and thruster offsets in `src/sub_bringup/config/vehicles/marlin_v3.yaml` are defined relative to it, except `imu_link`, which hangs off `base_link`. `description_launch.py` publishes that file's transforms on `/tf_static` from one `static_transforms` node.

## Data Flow (Simulation)

```
Stonefish
  ├─ sim/dvl ──────► sim_dvl_remapper ──────► odometry/dvl ──────────────────────► EKF
  ├─ sim/imu ──────► sim_imu_remapper ──────► imu/data ──────────────────────────► EKF
  ├─ sim/pressure ─► sim_pressure_to_depth ─► odometry/depth ────────────────────► EKF
  └─ sim/thruster_setpoints ◄── sim_thrusters ◄── control/thrusters ◄────────────── sub_control
                                                      odometry/filtered (EKF) ────────┤
                                                      motion_setpoint (sub_mission) ──┘
Operator (ros2 topic pub, Foxglove)
  └─ sim/kill_switch ─► sim_kill_switch ────► kill_switch ───────────────────────► sub_control
```

On hardware the same graph applies, with `waterlinked_dvl_driver` publishing `odometry/dvl` and `altitude` and `naviguider_imu_driver` publishing `imu/data` in place of the sim remappers, and `sub_low` publishing `odometry/depth` and `kill_switch` and driving the thrusters. There is no `robot_state_publisher` (the simulator's URDF is its only user).
