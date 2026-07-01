# Architecture

## Package Overview

```
src/
├── sub_bringup/                 # Launch files and config
├── sub_control/
│   ├── sub_control/             # Control algorithm
│   └── sub_control_interfaces/  # ROS2 messages for control
├── sub_drivers/
│   ├── flir_camera_driver/      # FLIR / Spinnaker camera driver (submodule)
│   ├── sub_driver_interfaces/   # Dropper / torpedo service definitions
│   ├── sub_serial_drivers/      # Serial-device nodes (Pico low-level, IMU)
│   └── waterlinked_dvl/         # WaterLinked DVL driver (submodule)
├── sub_sim/
│   ├── sub_sim/                 # Scenario generation, robot description
│   └── sub_sim_sensors/         # Sim sensor bridges (DVL, thrusters)
├── sub_vision/                  # Vision processing nodes, WIP
└── stonefish_ros2/              # Stonefish simulator ROS2 bridge (submodule)
```

## Nodes

### `sub_control` (`sub_control/sub_control`)

The main control node. Runs a 6-DOF cascade PID with bounded pseudo-inverse
thruster allocation at 50 Hz. See [control.md](control.md) for details.

| | Topic | Type |
|---|---|---|
| Sub | `pos_setpoint` | `sub_control_interfaces/Setpoint` |
| Sub | `att_setpoint` | `sub_control_interfaces/Setpoint` |
| Sub | `cmd_vel` | `geometry_msgs/Twist` |
| Sub | `odometry/filtered` | `nav_msgs/Odometry` |
| Sub | `altitude` | `std_msgs/Float64` |
| Sub | `kill_switch` | `std_msgs/Bool` |
| Pub | `control/thruster_0` … `thruster_7` | `std_msgs/Float64` (normalized −1…1) |
| Pub | `control/error` | `sub_control_interfaces/Error` |
| Client | `set_pose` | `robot_localization/SetPose` |

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `control_rate_hz` | `50.0` | Control loop frequency |
| `power_limit` | `0.6` | Normalized thrust cap in `[0, 1]`; bounds the allocator |
| `robot_name` | `""` | Namespace prefix for the `set_pose` frame id |
| `pos_pid.{x,y,z}` | — | Position → velocity PID `[kp, ki, kd, max]` |
| `vel_pid.{x,y,z}` | — | Velocity → force PID `[kp, ki, kd, max]` |
| `att_pid.{x,y,z}` | — | Attitude → rate PID `[kp, ki, kd, max]` |
| `ang_pid.{x,y,z}` | — | Angular-rate → torque PID `[kp, ki, kd, max]` |

PID gain arrays are loaded from the hardware or simulation control profile at
startup. See [control.md](control.md).

### `waterlinked_dvl_driver` (`waterlinked_dvl`) — hardware only

Driver for the WaterLinked A50 DVL. Its `~/odom` output is remapped to
`odometry/dvl` (velocity-only `nav_msgs/Odometry`) for the EKF.

### `sub_low` / `naviguider_imu_driver` (`sub_serial_drivers`) — hardware only

`sub_low` talks to the RPi Pico over `/dev/pico` (thruster output, kill switch).
`naviguider_imu_driver` reads the NaviGuider IMU over `/dev/naviguider_imu` and
publishes ENU `imu/data` with `frame_id` `marlin_v2/imu_link`.

### `sim_dvl_remapper` (`sub_sim_sensors`) — sim only

Converts Stonefish's proprietary DVL message into the velocity-only
`nav_msgs/Odometry` the EKF expects, matching the hardware DVL topic.

| | Topic | Type |
|---|---|---|
| Sub | `sim/dvl` | `stonefish_ros2/DVL` |
| Pub | `odometry/dvl` | `nav_msgs/Odometry` (velocity only) |

### `sim_imu_remapper` (`sub_sim_sensors`) — sim only

Re-expresses Stonefish's NED IMU stream as ENU and stamps it with `imu_link`.

| | Topic | Type |
|---|---|---|
| Sub | (Stonefish IMU) | `sensor_msgs/Imu` (NED) |
| Pub | `imu/data` | `sensor_msgs/Imu` (ENU) |

### `sim_thruster_republisher` (`sub_sim_sensors`) — sim only

Collects the 8 individual thruster topics into a single `Float64MultiArray` for
Stonefish, scaling each normalized command by 400 (T200 count).

| | Topic | Type |
|---|---|---|
| Sub | `control/thruster_0` … `thruster_7` | `std_msgs/Float64` |
| Pub | `sim/thruster_setpoints` | `std_msgs/Float64MultiArray` |

### `sim_kill_switch` (`sub_sim_sensors`) — sim only

Starts killed, releases the kill switch after a startup delay (`off_delay`, 8 s),
then passes `sim/kill_switch` through to `kill_switch`. `sim_torpedo_launcher`
and `sim_dropper` provide the corresponding actuator services in simulation.

### `ekf_filter_node` (`robot_localization`)

Fuses IMU orientation and DVL velocity into a filtered odometry estimate.

- `imu0`: `imu/data` — orientation + angular velocity
- `odom0`: `odometry/dvl` — linear velocity only
- Output: `odometry/filtered` (and the `odom` → `base_link` transform)

## TF Tree

```
map
└── marlin_v2/odom          (static, identity)
    └── marlin_v2/base_link  (published by EKF; ENU/FLU control frame)
        ├── marlin_v2/imu_link            (relative to base_link)
        └── marlin_v2/base_link_ned       (roll=π, yaw=−π/2 — static mounting frame)
            ├── marlin_v2/front_camera
            ├── marlin_v2/dvl_link
            ├── marlin_v2/dropper_link
            ├── marlin_v2/left_grabber_link
            ├── marlin_v2/right_grabber_link
            ├── marlin_v2/thruster_0_link
            ├── ...
            └── marlin_v2/thruster_7_link
```

Control runs in `base_link` (ENU/FLU). `base_link_ned` is a static mounting frame
— most sensor and thruster offsets in
`src/sub_bringup/launch/marlin_v2_launch.py` are defined relative to it, except
`imu_link`, which hangs off `base_link`.

## Data Flow (Simulation)

```
Stonefish
  ├─ sim/dvl ──────► sim_dvl_remapper ──► odometry/dvl ──────────────────────────► EKF
  ├─ (IMU, NED) ───► sim_imu_remapper ──► imu/data ───────────────────────────────► EKF
  ├─ sim/kill_switch ► sim_kill_switch ─► kill_switch ──────────────────────────► sub_control
  └─ sim/thruster_setpoints ◄── sim_thruster_republisher ◄── control/thruster_i ◄── sub_control
                                                                                      ▲
                                                      odometry/filtered (EKF) ───────┘
```

On hardware the same graph applies, with `waterlinked_dvl_driver` publishing
`odometry/dvl` and `naviguider_imu_driver` publishing `imu/data` in place of the
sim remappers, and `sub_low` driving the thrusters and kill switch.
