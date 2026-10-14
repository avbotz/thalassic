# Mission System

`sub_mission` runs selected missions as BehaviorTree.CPP XML. Packaged mission entrypoints live in `src/sub_mission/resources/missions/`; reusable task trees live in `src/sub_mission/resources/trees/`. Every file in both directories is registered before the selected mission is built, so a mission can run a task tree or another mission as a `SubTree`, as `hard_coded_competition` runs `hard_coded_gate`, `hard_coded_slalom` and `hard_coded_torp`. C++ nodes implement reusable movement primitives and typed interfaces for mission-specific features.

## Mission Selection

Mission selection uses the `mission` ROS parameter. The value may be the basename of a packaged file under `resources/missions/` without `.xml`, or an explicit XML path. The `role` ROS parameter must be `SURVEY` or `SEARCH`; it defaults to `SURVEY`.

```bash
ros2 launch sub_bringup sim_launch.py mission:=pool_a
ros2 run sub_mission mission --ros-args -r __ns:=/marlin_v3 -p mission:=pid_tuning -p role:=SEARCH
```

The launch files put `sub_mission` in the `robot_name` namespace automatically (`/marlin_v3` by default). If you start `mission` manually with `ros2 run`, pass the same namespace yourself. Otherwise it publishes root-level topics such as `/motion_setpoint`, while `sub_control` listens on `/marlin_v3/motion_setpoint`.

The current packaged mission entrypoints are `pool_a`, `pool_b`, `pool_c`, `pool_d`, `prelim`, `pool_test`, `vision_test` and `pid_tuning`; single tasks for testing, `bins_test`, `gate_test` (the coin flip and the gate), `gate_slalom_test`, `slalom_three_test` and `slalom_perception_hold_test` (the slalom model loaded, holding still); and `hard_coded_competition`, `hard_coded_gate`, `hard_coded_slalom`, `hard_coded_torp` and `hard_coded_octagon`, the same tasks dead-reckoned without vision.

The role starts as the `role` parameter. `GateMission` replaces it with the role shown on the gate's left opening, the one the vehicle passes through (`SelectGateRole`), and the bins tree drops in the fire bins (class `0`) for `SURVEY` or the blood bins (class `1`) for `SEARCH`. `bins_test` has no gate, so it runs on the parameter.

Vision BT XML uses task-agnostic model names such as `task="gate"`. `sub_mission` sends those names unchanged in `LoadModel` requests to `sub_vision`.

## Restart Supervisor

`sub_mission/restart` is a small supervisor for competition runs. It does not execute the behavior tree itself. Instead, it waits for the kill switch to be released, spawns `sub_mission/mission` in a child process, and stops that child when the sub is killed. After the next kill and release, however the last run ended, `restart` starts a fresh mission process from the beginning.

Run it with the same mission parameters as `mission`:

```bash
ros2 run sub_mission restart --ros-args -r __ns:=/marlin_v3 -p mission:=pool_a -p role:=SURVEY
```

It starts each mission process named `sub_mission`, as the launch files name it, in `restart`'s namespace (when it has one) and with its `mission`, `role`, `groot_port` and `use_sim_time`:

```bash
ros2 run sub_mission mission --ros-args -r __node:=sub_mission -r __ns:=/marlin_v3 -p mission:=<value> -p role:=<value> -p groot_port:=<value> -p use_sim_time:=<value>
```

`restart` exits at startup (FATAL) on an empty `mission`, a bad `role` or `groot_port`, or a missing mission file. The tree itself is loaded only by each child, after the release, so an XML mistake surfaces there.

Use `restart` when you want diver-controlled full-run reset behavior: kill the sub, place it back at the start, release the kill switch, and the mission starts again from step one. Use `mission` directly for bench tests, one-off XML smoke tests, or debugging where automatic restart would hide the original failure.

Current limitations:

| Behavior | Detail |
|---|---|
| Restart granularity | Restarts the whole selected mission, not the current subtree |
| State persistence | Blackboard values come from XML parameters/scripts each run; runtime BT state is discarded |
| Stop behavior | Any kill, even one shorter than its 0.5 s poll, stops the run: `SIGINT` to the mission's process group, up to 3 s for the whole group to exit, then `SIGTERM` and `SIGKILL` if needed. Each child's exit status or signal is logged |
| Parameter forwarding | Only `mission`, `role`, `groot_port` and `use_sim_time` (and the namespace) are forwarded by `restart.cpp`; a new mission parameter must be added there |
| Namespace | Manual `ros2 run` commands must use the same namespace as the rest of the stack, normally `/marlin_v3` |

## Live Monitoring (Groot2)

`mission` serves its tree to Groot2's Monitor mode on TCP port `groot_port` (5555 by default) and the port after it (5556). In Groot2, switch to Monitor, enter the address of the machine running `mission` and port 5555, and connect: `localhost` in simulation, the Jetson's `192.168.5.245` ([networking.md](networking.md)) on the vehicle. Groot2 then shows each node's status as the tree ticks; nodes the tree has moved on from are drawn faded, in the status they ended with.

```bash
ros2 launch sub_bringup sim_launch.py mission:=pool_a                   # live view on 5555
ros2 launch sub_bringup sim_launch.py mission:=pool_a groot_port:=5565  # on 5565 (and 5566)
ros2 launch sub_bringup sim_launch.py mission:=pool_a groot_port:=0     # no live view
ros2 run sub_mission mission --ros-args -r __ns:=/marlin_v3 -p mission:=pool_a -p groot_port:=5565
```

| Behavior | Detail |
|---|---|
| When it is up | From when the tree loads, before the kill switch is released, so Groot2 can connect and wait for the start. Under `restart`, each run's process loads the tree at the release, so the view is down between runs: connect Groot2 again after each release |
| End of a run | The view stays up 2 s after the tree finishes or a kill ends the run, so Groot2 shows where it ended and with what; the process then exits and the view goes with it. `SIGINT` and `SIGTERM` skip the wait, so under `restart` a kill takes the view down within its 0.5 s poll |
| Port taken | `mission` logs a warning and runs without the view, e.g. when a mission process left over from an earlier launch holds the port. A `groot_port` above 65534 (the view needs the next port too) or below 0 is FATAL at startup |
| Groot2 edition | The free edition monitors trees of up to 20 nodes. `prelim` (7), `pid_tuning` (10), `pool_test` (13), `vision_test` (3), `slalom_perception_hold_test` (3) and the single `hard_coded_*` tasks (4 to 9) fit; the `pool_*` missions have 277 nodes, `bins_test` 157, `gate_slalom_test` 64, `slalom_three_test` 49, `gate_test` 37, and `hard_coded_competition` 30. Larger trees, the blackboard view and breakpoints need Groot2 PRO |
| Breakpoints (PRO) | A breakpoint pauses the tick while it holds the mission's state, so the mission only sees a kill or an interrupt once the breakpoint is released. `sub_control` drops commands while killed either way. Groot2Publisher releases every breakpoint when Groot2 has been silent for 5 s |

Groot2 is not part of the pixi environment: download it from [behaviortree.dev](https://www.behaviortree.dev/groot/). The tree it shows comes from `mission`, so it is the one actually running, not `Project.btproj`.

## Behavior Tree XML

Task XML should contain mission logic: ordering, fallback/retry structure, pool-specific branches, and calls to reusable actions. C++ should contain the ROS-facing mechanics: publishing commands, waiting for control error, and wrapping any nontrivial state.

Movement-related XML nodes currently implemented in C++:

| BT node | Behavior |
|---|---|
| `PosSetpoint` | Position or depth command that waits for the controller to reach tolerance |
| `VelocitySetpoint` | Linear velocity along the heading; unset axes keep holding |
| `AltitudeSetpoint` | DVL altitude command that waits for the controller to reach tolerance |
| `AttSetpoint` | Attitude command that waits for the controller to reach tolerance |
| `AngularVelocitySetpoint` | Angular velocity; unset axes keep holding |
| `MoveRelative` | Relative forward/left/up movement along the current commanded heading |
| `MoveRelativePos` | Relative movement along odom's x, y and z, whatever the heading |
| `NavigateToTransform` | Move so one vehicle frame takes another's position, e.g. the dropper onto where the down camera is (see below) |
| `TimedVelocity` | Linear velocity along the heading for a fixed time, then hold |
| `AddAttSetpoint` | Relative roll/pitch/yaw target |
| `Spin` | Turn through a yaw distance at a set rate, braking into the final heading |
| `Stop` | HOLD x, y, z and yaw where they come to rest, ending any velocity or spin still running |
| `SavePose` | Write the x, y and/or yaw targets to the blackboard (gate home, a task's entry heading) |
| `WaitUntilHit` | Wait for velocity feedback to drop after a velocity command |
| `SweepAngle` | Turn through a yaw sweep (ahead, 40° and 80° right, 80° and 40° left, ahead); at the first heading where `attempts` front-camera frames see the target, write the average of their yaws toward it, each taken from the heading it was seen on, to `yaw`; optionally only for a target on one `side` of `reference_yaw` |
| `SearchForDetection` | Move at a fixed velocity, holding the heading, until a detection; or sweep left, then right |
| `AlignToDetection` | Turn (and optionally change depth) until a front-camera detection's bearing is within `tolerance` |
| `ForwardContinuousAlign` | Move forward at a fixed velocity, steering toward a front-camera detection, until it is close, lost or passed, or `max_dist` or `timeout_msec` runs out |
| `ForwardFixedTransit` | Move forward at a fixed velocity through something ranged before (the gate, a slalom opening), by odometry |
| `LateralAlignToDetection` | Center a front-camera detection by moving sideways (and in z), keeping the heading; locks one odom target, moves at a fixed speed, or steps |
| `SelectGateRole` | Set the role from the gate's left opening |
| `SquareUpToGate` | Line up `standoff` in front of the gate's left opening, on its normal and facing through it, however the gate is turned (see below) |
| `OrientToDetectionAtDist` | Square up to an object's face (`heading_yaw_deg`) at a set distance, looking at its center |
| `OctagonSurfaceSweep` | Turn in steps until an image to surface at is in view, then rise |
| `DownForwardAlign`, `DownForwardSweepAlign` | Move forward while centering a down-camera detection; the sweep zigzags along a track until it sees one |
| `DownSweepAlign` | Search sideways over the down camera, right of the start and then left |
| `DownAlignToDetection`, `DownContinuousAlign` | Center a down-camera detection, optionally hold distance/depth, and yaw to orientation metadata (the second name is an alias) |

The nodes that steer on a front-camera bearing (`AlignToDetection`, `LateralAlignToDetection`, `SweepAngle`, `ForwardContinuousAlign`, `OrientToDetectionAtDist`, `SquareUpToGate`) fail on `camera="down"`, and so do the searches built on `SweepAngle`. The `Down*` nodes read the down camera and have no `camera` port. A camera other than `front` or `down` fails the node.

The searches the tasks share are composed from these in XML, in `trees/search.xml`, and run as SubTrees, e.g. `<SubTree ID="SweepCheck" task="octagon_table" class_id="" min_score="0.6"/>`:

| SubTree | Behavior |
|---|---|
| `SweepCheck` | `SweepAngle`, then `AttSetpoint` to the yaw it found |
| `ForwardSweepAlign` | `SweepAngle` up to `num_sweeps` times with a `forward_step` `MoveRelative` between sweeps (none after the last), then `AttSetpoint` to the yaw found. A forward move that does not arrive ends it with `FAILURE`; a sweep cut short by a yaw step that does not arrive counts as one that saw nothing |
| `DownPatternScan` | `WaitForDetection` on the down camera at the start and at five points around it, (forward, left) of the start in `movement_dist`: (−1, −1), (0, 2), (2, 0), (0, −2), (−1, 1). Each point is a `MoveRelative` from the one before, along the heading the scan started on; a move that does not arrive samples where it got to. `SUCCESS` where a frame shows the target, else back to the start (`SavePose`, `PosSetpoint`) and `FAILURE` |

A setting left out takes the default in the file's `TreeNodesModel`, which BT.CPP applies to a SubTree as well as Groot2 showing it; `task` and `class_id` have none, so a SubTree without them fails to load, and `class_id=""` takes any class. BT.CPP reads an attribute that looks like a number as one, and a tree loads only if that is the type of the port it reaches: a decimal setting needs its decimal point (`forward_step="1.0"`), and a numbered class comes from a `Script` (`fire_class := '0'`, then `class_id="{fire_class}"`), as in `torp.xml` and `bins.xml`.

Built-in BehaviorTree.CPP nodes such as `Sleep`, `Repeat`, `Fallback`, `Sequence`, `Inverter`, and decorators must not be registered in the `TreeNodesModel`. Groot 2 can use them from its built-in palette.

When editing XML in Groot 2, keep the `TreeNodesModel`s of `Project.btproj`, `resources/missions/*.xml`, and `resources/trees/*.xml` synchronized: custom nodes only, plus the SubTree models of `trees/search.xml`, whose defaults BT.CPP takes from that file. Ports in XML must exactly match the C++ `providedPorts()` model; BehaviorTree.CPP rejects unknown ports when it creates the tree, before the first tick.

## Control Interface

Mission does not write to the low-level board directly. Movement commands are published to `sub_control`:

| Topic | Type | Use |
|---|---|---|
| `motion_setpoint` | `sub_control_interfaces/MotionSetpoint` | Every movement command: a mode per axis (position, velocity, hold) and its target, with a fresh `id` each |
| `control/error` | `sub_control_interfaces/Error` | Position, velocity, attitude, and angular-rate tracking error, and the id of the last command `sub_control` took up |
| `odometry/filtered` | `nav_msgs/Odometry` | The measured pose, which vision alignment, and relative moves on an axis without a target, start from |
| `kill_switch` | `std_msgs/Bool` | Low-level kill state, published by `sub_low` (`sim_kill_switch` in simulation) |
| `control/armed` | `std_msgs/Bool` | Latched by `sub_control`: the run starts only once it is armed |

The commands go out through `SetpointPublisher` (`commands.hpp`), which gives each its `id`. `MissionNode` counts a `control/error` as an update (`control_error_updates`) only when it carries the id of the last command sent and `sub_control` did not reject it: an error computed before `sub_control` took the command up is about the one before, and would read as arrival. A rejected command is logged and its move times out. A velocity the mission sends has no timeout: it lasts until replaced, and `MissionNode` sends a stop when the run ends ([control.md](control.md#command-interface)).

### Frames

The mission works in `sub_control`'s frames (REP-103) throughout: the C++, the trees, port values and blackboard entries. `commands.hpp` copies targets into `MotionSetpoint` unchanged, and `odometry/filtered` and `control/error` are used as they arrive.

| Values | Frame |
|---|---|
| Positions: `PosSetpoint`, `SavePose`, `Transit`'s `z`, `OctagonSurfaceSweep`'s `surface_z` | `odom`, ENU. x and y are zero where the kill switch was released, x along the heading there; z is up, so 1 m deep is `z="-1.0"` |
| Yaws: `AttSetpoint`, `SavePose`, `SweepAngle`, the pool headings | `odom`, counter-clockwise from the heading at the release |
| Relative moves and rates: `MoveRelative`, `VelocitySetpoint`, `TimedVelocity`, `SearchForDetection`, `LateralAlignToDetection`'s target offsets, `AddAttSetpoint`, `AngularVelocitySetpoint`, `Spin`, `OctagonSurfaceSweep` | Body FLU along the heading: x forward, y left, z up; a positive yaw turns left |
| `MoveRelativePos` | odom's axes |
| `AltitudeSetpoint`'s `z` | Height above the bottom |
| Roll and pitch | FLU: positive roll lowers the right side, positive pitch the nose |
| `sub_vision` bearings | Camera-optical, positive right of / below the image center; `vision.cpp` converts them where it steers on them |
| `sub_vision` yaw metadata (`yaw_deg`, `heading_yaw_deg`) | Counter-clockwise from the heading, used as is |

A port named for depth (`AlignToDetection`'s `depth`, `depth_gain` and `max_depth_step`, `LateralAlignToDetection`'s `align_depth` and `max_depth_step`) names the vertical axis or a gain, never a signed value; every vertical value is a z.

Each command sets only the axes it names and leaves the others as they were (`commands.hpp`): `VelocitySetpoint x="0.25"` drives forward while depth and heading stay held, `PosSetpoint z="-1.5"` changes depth and keeps x, y and the heading, and `Spin` turns in yaw while roll and pitch stay level. Velocity ports left unset keep their axis; x and y go together, so setting one velocity sets the other to zero, and setting one position keeps the other where it is meant to be. All commands go out on the one topic, so `sub_control` applies them in the order they were sent. `sub_control` smooths every target into a trajectory, so a step in the XML is a smooth move on the vehicle.

The mission remembers the position and attitude targets it sent (`MissionNode::commanded_pos`, `commanded_att`), so relative commands (`MoveRelative`, `AddAttSetpoint`, the sweeps) add to the target, not to wherever the vehicle has drifted. An axis set to a velocity holds no target until it is given one again, and a relative command on it starts from the measured pose. Vision alignment always starts from the measured pose. Targets are cleared when the kill switch is released, as `sub_control` then holds wherever the vehicle is. A `sub_control` `reset_pose` mid-mission (the Tuning Routines panel's Zero pose) re-zeroes x, y and yaw, but the mission's remembered targets stay in the old frame.

For altitude commands, `AltitudeSetpoint` publishes a z position with `altitude=true`; the z value is interpreted as height above the bottom from DVL altitude feedback, and `sub_control` falls back to holding depth if the DVL loses the bottom. Later commands that do not set z keep the altitude hold; a `MoveRelative` with a z offset moves the altitude target (positive z raises it). `sub_control` rejects a whole command whose altitude target is below 0, or that asks for an altitude hold without DVL bottom lock, and keeps the old target; `control/error` reports the rejection, so the move cannot succeed on the old target's error and times out instead. `AltitudeSetpoint` fails up front on a z at or below 0, and `MoveRelative`, `MoveRelativePos` and `NavigateToTransform` on an altitude target below 0, rather than wait out that timeout.

### Transform Navigation

`NavigateToTransform` moves the vehicle so that `target_frame` takes the position `source_frame` has now: both are frames of the vehicle's description (`src/sub_bringup/config/vehicles/<robot_name>.yaml`), looked up in `base_link` on `/tf_static` under the node's namespace (`marlin_v3/front_camera`). It is a relative move by source minus target, with `horizontal_only` keeping z, and does not turn the vehicle. The torpedo tree moves `torpedo_0_link` onto where `front_camera` was when it was centered on the target, then `torpedo_1_link` onto a point 0.15 m left of it; the bins tree moves `dropper_link` onto where `down_camera` was when it was centered on a bin. A frame that is not published fails the node.

### Squaring Up to the Gate

The gate need not be square to the course: in the sim's area A/B1 it is turned 30° from the start heading. `GateMission` therefore crosses along the gate's normal, not along whatever heading the coin flip left:

1. `PosSetpoint z="-0.7"`: down to where the role panels hang a little above the camera (a no-op after the coin flip, which is already there).
1. `SearchForDetection` and `SelectGateRole` find the left opening and read its role.
1. `SquareUpToGate` lines up 2 m (`standoff`) in front of the left opening, on its normal and facing through it. With both panels in view, the gate post-processor ranges each from its icons' heights, which a turned gate does not shrink, and gives the turn that would square the camera to the gate (`heading_yaw_deg`). Each frame is placed in odom from where the camera was when the frame was taken (`MissionNode::horizontalPoseAt`, from the last 3 s of odometry, and the `front_camera` mount), and the gate is the median of the last 25 frames' facings and left-panel positions. While fewer than `min_facing_frames` frames have shown both panels, it turns to keep both in view and backs off to `look_distance` if the gate is closer. It then moves to the standoff point looking at the middle of the gate, so both panels stay in view and keep refining the estimate, turns square there, and finishes once within `position_tolerance` and `yaw_tolerance_deg` for `settle_msec`. If no frame ever shows both panels, after `facing_timeout_msec` it takes the gate to face back along the line of sight to the left panel, which is what keeping the heading did.
1. `PosSetpoint z="-1.2"`: below the panels and the divider, so only the posts are left to clear.
1. `MoveRelative x="3.0"`: straight on along the gate's normal, the 2 m standoff and 1 m past. The move starts from the standoff point and goes along the estimated facing, so it crosses the gate's line at the left opening even if the facing is off by a few degrees; only the crossing angle suffers.

## Completion Semantics

Position and attitude commands return `RUNNING` until fresh `control/error` feedback (carrying their command's id, see [Control Interface](#control-interface)) is within fixed C++ tolerances (`POSITION_TOLERANCE` 0.25 m, `ANGLE_TOLERANCE` 5°, in `commands.hpp`). The tolerance is intentionally not an XML port, because the old movement layer owned those constants and mission XML should stay focused on task logic.

None of them waits forever. `PosSetpoint`, `MoveRelative`, `MoveRelativePos`, `NavigateToTransform`, `AltitudeSetpoint`, `AttSetpoint`, `AddAttSetpoint` and `Spin` return `FAILURE` once `timeout_msec` passes. Left at 0 (the default), the timeout scales with the move: 20 s to settle plus the move at about a third of `sub_control`'s speed limits (0.15 m/s horizontal, 0.1 m/s vertical, 0.2 rad/s), e.g. 47 s for a 4 m hop and 36 s for a half turn; `AltitudeSetpoint`, which does not know the altitude it starts from, allows 41 s, for a whole 2.1 m water column. A move that cannot finish, whether from a current or a dead thruster, then fails and the tree decides what to do instead of stalling the run. A timed-out `Spin` holds the heading it reached. Every timeout, and `TimedVelocity`'s duration, runs on the wall clock (steady), not ROS time.

A port whose blackboard entry is missing (`x="{@gate_home_x}"` before anything set it) fails the node with an error naming the port, rather than commanding 0. This holds for every port of the nodes in `src/nodes/actions/`, `SearchForDetection` and `ForwardFixedTransit`, and for the detection filter (`camera`, `task`, `class_id`, `min_score`) of the other vision nodes but `LoadModel`; their remaining ports (gains, distances, timeouts) do not fail and keep the port's default.

`VelocitySetpoint` and `AngularVelocitySetpoint` return `SUCCESS` after publishing because they are open-loop commands by nature. Use `WaitUntilHit` or a later position/attitude hold when the sequence needs a completion condition. `WaitUntilHit` measures the speed against the commanded velocity the mission recorded, so it needs the vehicle up to speed first (e.g. a `Sleep` after the `VelocitySetpoint`): started below 0.03 m/s, it succeeds at once. No packaged tree uses it.

`Spin` is a special case: it turns at a yaw velocity, counting the measured yaw rate from `control/error` over ROS time (the simulator's clock in simulation), and from π/2 before the end holds the final heading instead, which `sub_control`'s trajectory brakes into. Stopping the velocity at the end instead would overshoot by the braking distance and pull back.

The vision nodes that move at a velocity (`SearchForDetection`, `ForwardContinuousAlign`, `ForwardFixedTransit`, `LateralAlignToDetection`) end with a HOLD on the axes they moved, whether they succeed, fail or are halted. `DownForwardAlign` and `DownForwardSweepAlign`, whose position targets run ahead of the vehicle, HOLD x and y when they run out of distance or time or are halted, and the sweep also when it finds a detection; `DownSweepAlign` holds x and y where a detection comes into view mid-move.

## Action Boundaries

The packaged trees contain no blackboxed actions. Mission sequencing and recovery logic belong in XML, while reusable movement, vision, and actuator mechanics belong in C++. Hardware commands must use typed ROS topics, services, or actions; do not introduce logging-only placeholders or direct serial writes.

## Abort Behavior

Coin flip does not need a separate abort leaf. It looks for the gate ahead, then a quarter turn right, a half turn and a quarter turn left: heads only puts the AUV parallel to the gate (handbook 3.2.1), so the gate can be on either side. If all four attempts fail, `CoinFlipMission` returns `FAILURE` naturally. `CompetitionRun` intentionally does not wrap `CoinFlipMission` in `ForceSuccess`, so that failure stops the full run. Every later task is bounded by a time budget and recovered on failure (see `src/sub_mission/README.md`). Mission code does not send raw serial power commands.

A run starts once the kill switch is released, `sub_control` is armed (`control/armed`: it holds off while the state estimate, the IMU or depth is missing, and logs which), and 7 s more have passed for the ESCs. Until then `mission` logs what it is waiting for.

If the kill switch engages during a run, `mission` halts the tree and exits rather than failing through every remaining task. A kill and release, however brief, ends the run: `sub_control` drops all commands while killed and re-zeroes the pose at the release, so nothing the tree was doing still applies. Under `restart`, releasing the switch starts the mission again from the beginning.

Vision alignment never steers above z = −0.5, 0.5 m deep (`MAX_VISION_Z` in `vision.cpp`): a range missing from a detection, or one held from a target the pool is too shallow for, must not drive the sub through the surface. Surfacing is always an explicit `PosSetpoint` in the tree, as at the octagon. `OctagonSurfaceSweep`'s `surface_z` is the tree's own value too, so it is not bounded.

When the tree finishes, with any result, `mission` sends a `Stop` (a HOLD on x, y, z and yaw), so a velocity or spin the tree left running does not carry on without it, and the vehicle holds where it is, submerged: breaching the surface outside the octagon ends a RoboSub run. `SIGINT` or `SIGTERM` (Ctrl-C, launch shutdown, `restart` stopping it) ends the run the same way, between ticks: the tree is halted and the `Stop` sent. `mission` exits non-zero only when the mission cannot be loaded or the node fails at startup (a bad `role` or `groot_port`).
