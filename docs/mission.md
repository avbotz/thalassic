# Mission System

`sub_mission` runs selected missions as BehaviorTree.CPP XML. Packaged mission
entrypoints live in `src/sub_mission/resources/missions/`; reusable task trees
live in `src/sub_mission/resources/trees/`. C++ nodes implement reusable
movement primitives and blackbox placeholders for features that do not have a
current ROS API yet.

## Mission Selection

Mission selection uses the `mission` ROS parameter. The value may be the basename
of a packaged file under `resources/missions/` without `.xml`, or an explicit XML
path. The `role` ROS parameter selects the vision model family and must be
`SURVEY` or `SEARCH`; it defaults to `SURVEY`.

```bash
ros2 launch sub_bringup sim_launch.py mission:=pool_a
ros2 run sub_mission mission --ros-args -p mission:=pid_tuning -p role:=SEARCH
```

The current packaged mission entrypoints are `pool_a`, `pool_b`, `pool_c`,
`pool_d`, `prelim`, `pool_test`, `vision_test`, and `pid_tuning`.

Vision BT XML uses logical task names such as `task="gate"`. At runtime,
`sub_mission` maps those to role-specific sub_vision model names:
`gate_survey` for `SURVEY`, `gate_search` for `SEARCH`.

## Behavior Tree XML

Task XML should contain mission logic: ordering, fallback/retry structure,
pool-specific branches, and calls to reusable actions. C++ should contain the
ROS-facing mechanics: publishing commands, waiting for control error, and
wrapping any nontrivial state.

Movement-related XML nodes currently implemented in C++:

| BT node | Behavior |
|---|---|
| `PosSetpoint` | Position or depth command that waits for the controller to reach tolerance |
| `VelocitySetpoint` | Body-frame linear velocity command |
| `AltitudeSetpoint` | DVL altitude command that waits for the controller to reach tolerance |
| `AttSetpoint` | Attitude command that waits for the controller to reach tolerance |
| `AngularVelocitySetpoint` | Body-frame angular velocity command |
| `MoveRelative` | Relative x/y/z movement using the current commanded yaw |
| `AddAttSetpoint` | Relative roll/pitch/yaw target |
| `Spin` | Angular-velocity spin until measured yaw travel reaches the target |
| `WaitUntilHit` | Wait for velocity feedback to drop after a velocity command |
| `SweepCheck` | Sweep the old yaw pattern and align to the first valid front-camera detection |
| `ForwardSweepAlign` | Load a vision model, sweep yaw, move forward between sweeps, and align to the first valid detection |
| `ForwardAlign` | Move forward while continuously yaw/depth-aligning to a front-camera detection |
| `SurfaceAtOctagon`, `SurfaceAndKill` | Surface and stop commanded motion |

Built-in BehaviorTree.CPP nodes such as `Sleep`, `Repeat`, `Fallback`,
`Sequence`, `Inverter`, and decorators must not be registered in the
`TreeNodesModel`. Groot 2 can use them from its built-in palette.

When editing XML in Groot 2, keep `Project.btproj`, `resources/missions/*.xml`,
and `resources/trees/*.xml` synchronized with custom nodes only. Ports in XML
must exactly match the C++ `providedPorts()` model; BehaviorTree.CPP rejects
unknown ports at runtime.

## Control Interface

Mission does not write to the low-level board directly. Movement commands are
published to `sub_control`:

| Topic | Type | Use |
|---|---|---|
| `pos_setpoint` | `sub_control_interfaces/Setpoint` | Position/depth/altitude target, or body-frame linear velocity |
| `att_setpoint` | `sub_control_interfaces/Setpoint` | Absolute attitude target, or body-frame angular velocity |
| `control/error` | `sub_control_interfaces/Error` | Position, velocity, attitude, and angular-rate tracking error |
| `kill_switch` | `std_msgs/Bool` | Low-level kill state, published by `sub_low` |

The control topics use REP-103 FLU at the ROS boundary: x forward, y left, z
up. Existing mission XML was migrated from the old convention: x forward, y
right, z down/depth. The mission C++ preserves current XML behavior by
converting signs before publishing to `sub_control`, and converting
`control/error` back before evaluating movement tolerances.

For altitude commands, `AltitudeSetpoint` publishes `pos_setpoint` with
`altitude=true`; the z value is interpreted as height above the bottom from DVL
altitude feedback.

## Completion Semantics

Position and attitude commands return `RUNNING` until fresh `control/error`
feedback is within fixed C++ tolerances. The tolerance is intentionally not an
XML port, because the old movement layer owned those constants and mission XML
should stay focused on task logic.

`VelocitySetpoint` and `AngularVelocitySetpoint` return `SUCCESS` after
publishing because they are open-loop commands by nature. Use `WaitUntilHit` or
a later position/attitude hold when the sequence needs a completion condition.

`Spin` is a special case: it commands angular velocity, integrates measured yaw
rate from `control/error`, then publishes an attitude hold at the final yaw.

## Blackbox Actions

Blackbox actions are placeholders for behavior that needs an API that is not
available in the current codebase, or behavior intentionally deferred from this
migration.

Examples:

| Area | Why it remains blackboxed |
|---|---|
| Task-specific search strategy | Generic vision load/wait/align nodes exist, but higher-level search tactics are still placeholders |
| Grabber/dropper/torpedo actions | These require driver/service integration rather than direct serial writes |
| Exact abort/power-off behavior | The old mission sent raw `p 0` through `control_write`; the current stack has no mission-safe raw write API |
| Resetting yaw origin | The old code used raw `x` write semantics; the current controller captures origin on kill-switch release |

When replacing a blackbox, prefer a typed ROS topic, service, action, or
parameter API. Do not reintroduce direct serial/raw board writes in mission.

## Abort Behavior

Old `AbortMission` came from the failure branch of `coin_flip()`: when the gate
could not be found, the old mission sent `p 0` through `control_write` to set
power to zero.

That exact behavior cannot be reproduced until the current stack exposes a
supported mission-level power-disable or abort API. A partial replacement can
publish zero velocity commands and return `FAILURE` to stop the tree. If
controller-level power disable is desired, implement it explicitly through a
supported `sub_control` parameter or service rather than a raw board command.
