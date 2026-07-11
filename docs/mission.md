# Mission System

`sub_mission` runs selected missions as BehaviorTree.CPP XML. Packaged mission
entrypoints live in `src/sub_mission/resources/missions/`; reusable task trees
live in `src/sub_mission/resources/trees/`. C++ nodes implement reusable
movement primitives and typed interfaces for mission-specific features.

## Mission Selection

Mission selection uses the `mission` ROS parameter. The value may be the basename
of a packaged file under `resources/missions/` without `.xml`, or an explicit XML
path. The `role` ROS parameter must be `SURVEY` or `SEARCH`; it defaults to
`SURVEY`.

```bash
ros2 launch sub_bringup sim_launch.py mission:=pool_a
ros2 run sub_mission mission --ros-args -r __ns:=/marlin_v2 -p mission:=pid_tuning -p role:=SEARCH
```

The launch files put `sub_mission` in the `robot_name` namespace automatically
(`/marlin_v2` by default). If you start `mission` manually with `ros2 run`, pass
the same namespace yourself. Otherwise it publishes root-level topics such as
`/pos_setpoint`, while `sub_control` listens on `/marlin_v2/pos_setpoint`.

The current packaged mission entrypoints are `pool_a`, `pool_b`, `pool_c`,
`pool_d`, `prelim`, `pool_test`, `vision_test`, and `pid_tuning`.

Vision BT XML uses task-agnostic model names such as `task="gate"`.
`sub_mission` sends those names unchanged in `LoadModel` requests to
`sub_vision`.

## Restart Supervisor

`sub_mission/restart` is a small supervisor for competition runs. It does not
execute the behavior tree itself. Instead, it waits until the low-level board is
alive, spawns `sub_mission/mission` in a child process, and kills that child
when the sub is killed. When the kill switch is released again, `restart` starts
a fresh mission process from the beginning.

Run it with the same mission parameters as `mission`:

```bash
ros2 run sub_mission restart --ros-args -r __ns:=/marlin_v2 -p mission:=pool_a -p role:=SURVEY
```

When `restart` is namespaced, it forwards that namespace along with `mission`
and `role` to each spawned mission process:

```bash
ros2 run sub_mission mission --ros-args -r __ns:=/marlin_v2 -p mission:=<value> -p role:=<value>
```

Use `restart` when you want diver-controlled full-run reset behavior: kill the
sub, place it back at the start, release the kill switch, and the mission starts
again from step one. Use `mission` directly for bench tests, one-off XML smoke
tests, or debugging where automatic restart would hide the original failure.

Current limitations:

| Behavior | Detail |
|---|---|
| Restart granularity | Restarts the whole selected mission, not the current subtree |
| State persistence | Blackboard values come from XML parameters/scripts each run; runtime BT state is discarded |
| Stop behavior | Sends `SIGINT` to the spawned mission process group, waits for it to exit, then escalates to `SIGTERM`/`SIGKILL` if needed |
| Parameter forwarding | Only `mission` and `role` are forwarded by `restart.cpp` today |
| Namespace | Manual `ros2 run` commands must use the same namespace as the rest of the stack, normally `/marlin_v2` |

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
| `ForwardSweepAlign` | Sweep yaw, move forward between sweeps, and align to the first valid detection |
| `ForwardAlign` | Move forward while continuously yaw/depth-aligning to a front-camera detection |
| `OrientToDetectionAtDist` | Use vision orientation metadata to square up to an object while holding distance |
| `DownForwardAlign`, `DownForwardSweepAlign` | Move forward while centering a down-camera detection with x/y position offsets |
| `DownAlignToDetection` | Center a down-camera detection, optionally hold distance/depth, and yaw to orientation metadata |

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

## Action Boundaries

The packaged trees contain no blackboxed actions. Mission sequencing and
recovery logic belong in XML, while reusable movement, vision, and actuator
mechanics belong in C++. Hardware commands must use typed ROS topics, services,
or actions; do not introduce logging-only placeholders or direct serial writes.

## Abort Behavior

Coin flip does not need a separate abort leaf. If all three gate-search attempts
fail, `CoinFlipMission` returns `FAILURE` naturally. `CompetitionRun`
intentionally does not wrap `CoinFlipMission` in `ForceSuccess`, so that failure
stops the full run. Mission code does not send raw serial power commands.
