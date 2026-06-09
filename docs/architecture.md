# Architecture

## Package Overview

```
src/
├── sub_bringup/            # Launch files and config
├── sub_drivers/
│   ├── sub_control/        # Cascade PID + thruster mixing
│   ├── sub_control_interfaces/  # Custom ROS2 messages
│   ├── sub_drivers_mappings/    # DVL → odometry bridge
│   └── waterlinked_dvl/    # WaterLinked DVL driver (submodule)
├── sub_sim/
│   ├── sub_sim/            # Scenario generation, robot description
│   └── sub_sim_sensors/    # Sim sensor bridges (DVL, thrusters)
└── stonefish_ros2/         # Stonefish simulator ROS2 bridge
```

## Nodes

### `sub_control` (`sub_drivers/sub_control`)

The main control node. Runs a cascade PID loop at 50 Hz and publishes one normalized command per thruster. See [control.md](control.md) for the loop and allocation details.

| | Topic | Type |
|---|---|---|
| Sub | `pos_setpoint` | `sub_control_interfaces/Setpoint` |
| Sub | `att_setpoint` | `sub_control_interfaces/Setpoint` |
| Sub | `odometry/filtered` | `nav_msgs/Odometry` |
| Sub | `dvl_altitude` | `std_msgs/Float64` |
| Pub | `control/thruster_0` … `thruster_7` | `std_msgs/Float64` (normalized −1…1) |
| Pub | `/control/pos/x`, `/y`, `/z` | `std_msgs/Float64` |
| Pub | `/control/vel/x`, `/y`, `/z` | `std_msgs/Float64` |
| Pub | `/control/ang/x`, `/y`, `/z` | `std_msgs/Float64` |
| Pub | `/control/angvel/x`, `/y`, `/z` | `std_msgs/Float64` |

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `control_frame` | `base_link` | Body frame for TF lookups (sim sets `marlin_v2/base_link`) |
| `world_frame` | `odom` | World frame for TF lookups (sim sets `map`) |
| `odom_topic` | `odometry/filtered` | Source of velocity feedback |
| `control_rate_hz` | `50.0` | Control loop frequency |
| `thruster_max_force` | `35.0` | Per-thruster force cap [N] for allocation saturation |

The cascade gains (`pos_kp`, `vel_kp/ki/kd`, `att_kp`, `ang_kp/ki/kd`) and rate limits (`max_speed`, `max_ang_rate`) are also parameters and are **live-tunable** on a running node — see [control.md](control.md#gains-ros-parameters--live-tunable).

### `dvl_odom_remapper` (`sub_drivers_mappings`)

Converts `marine_acoustic_msgs/Dvl` into `nav_msgs/Odometry` (velocity only) and extracts altitude.

| | Topic | Type |
|---|---|---|
| Sub | `dvl` | `marine_acoustic_msgs/Dvl` |
| Pub | `dvl_odom` | `nav_msgs/Odometry` |
| Pub | `dvl_altitude` | `std_msgs/Float64` |

### `sim_dvl_remapper` (`sub_sim_sensors`) — sim only

Bridges Stonefish's proprietary DVL message to the standard `marine_acoustic_msgs/Dvl`.

| | Topic | Type |
|---|---|---|
| Sub | `sim/dvl` | `stonefish_ros2/DVL` |
| Pub | `dvl` | `marine_acoustic_msgs/Dvl` |

### `thruster_republishers` (`sub_sim_sensors`) — sim only

Collects 8 individual thruster topics into a single `Float64MultiArray` for Stonefish.

| | Topic | Type |
|---|---|---|
| Sub | `/marlin_v2/control/thruster_0` … `thruster_7` | `std_msgs/Float64` |
| Pub | `/marlin_v2/sim/thruster_setpoints` | `std_msgs/Float64MultiArray` |

### `ekf_filter_node` (`robot_localization`)

Fuses IMU orientation and DVL velocity into a filtered odometry estimate.

- IMU: angular velocity + linear acceleration
- DVL odom: linear velocity only
- Output: `odometry/filtered`

## TF Tree

```
map
└── marlin_v2/odom          (static, identity)
    └── marlin_v2/base_link  (published by EKF)
        └── marlin_v2/base_link_ned  (roll=π — FLU→FRD; this is the control frame)
            ├── marlin_v2/front_camera
            ├── marlin_v2/dvl_link
            ├── marlin_v2/imu_link
            ├── marlin_v2/dropper_link
            ├── marlin_v2/left_grabber_link
            ├── marlin_v2/right_grabber_link
            ├── marlin_v2/thruster_0_link
            ├── ...
            └── marlin_v2/thruster_7_link
```

All sensor offsets in `src/sub_bringup/launch/marlin_v2_launch.py` are defined relative to `base_link_ned`.

## Data Flow (Simulation)

```
Stonefish
  ├─ sim/dvl ──► sim_dvl_remapper ──► dvl ──► dvl_odom_remapper ──► dvl_odom ──► EKF
  ├─ /marlin_v2/imu ────────────────────────────────────────────────────────────► EKF
  └─ sim/thruster_setpoints ◄── thruster_republishers ◄── control/thruster_i ◄── sub_control
                                                                                    ▲
                                                    odometry/filtered (EKF) ───────┘
```
