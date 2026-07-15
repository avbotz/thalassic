# sub_mission

Behavior-tree mission executive. A mission is selected with a single ROS
parameter, `mission`, that names a **BehaviorTree.CPP v4 XML file** — the same
format Groot2 edits and Nav2 uses for its behaviors. This replaces the old
pile of boolean task-selection parameters.

## Coordinate frames

Everything in sub_mission — action ports, XML tree values, and internal state —
is REP-103, the same convention `sub_control` speaks:

- **World (ENU):** z is up, so underwater positions are *negative*
  (`<PosSetpoint z="-1.0"/>` is 1 m deep); yaw is counter-clockwise-positive
  with 0 at the sub's initial heading.
- **Body (FLU):** x forward, y left, z up. `<MoveRelative y="1.0"/>` moves 1 m
  to port.
- **Global (ENU):** `<MoveRelativePos x="1.0"/>` adds 1 m east to the commanded
  position, independent of attitude.
- **Exceptions:** `AltitudeSetpoint z` is height above the bottom (positive
  up), and `AlignToDetection depth_gain`/`ForwardAlign depth_offset` keep
  depth semantics (positive = deeper).

Camera bearings from `sub_vision` stay in the optical convention (positive =
right of / below center); the vision action nodes convert them to ENU/FLU
steering internally.

## Running

```sh
# By name -> resources/missions/<name>.xml from the installed share directory
ros2 run sub_mission mission --ros-args -r __ns:=/marlin_v2 -p mission:=pool_a

# Or with an explicit path (useful while iterating without rebuilding)
ros2 run sub_mission mission --ros-args -r __ns:=/marlin_v2 -p mission:=/path/to/my_mission.xml
```

Use the same namespace as the rest of the stack when starting `mission` or
`restart` manually. The bringup launch files use `robot_name:=marlin_v2` by
default, so manual `ros2 run` commands normally need `-r __ns:=/marlin_v2`.
Without it, mission commands publish to root-level topics such as
`/pos_setpoint`, and the namespaced controller will not receive them.

`restart` takes the same parameter and forwards it to each `mission` run it
spawns. Launching without `mission` (or with a bad name) prints the available
mission names and exits — before the node starts waiting on the kill switch.

```sh
ros2 run sub_mission restart --ros-args -r __ns:=/marlin_v2 -p mission:=pool_a
```

## Layout

```
resources/
  trees/       # task library, one file per element:
               #   gate, slalom, bins, torp, octagon, coin_flip,
               #   competition (the shared 2026 run), plus test trees
  missions/    # runnable entrypoints (pool_a..pool_d, prelim, pool_test,
               #   vision_test, pid_tuning, bins_test)
  Project.btproj  # Groot2 project covering both, open it to edit any tree
```

**trees vs missions.** A *tree* is a reusable, self-contained task (`GateMission`,
`SlalomMission`, ...) that is never selected directly. A *mission* is a runnable
entrypoint you pass to `mission:=`. Everything is plain BT.CPP XML. All files in
`resources/trees/` are registered with the factory first, then the mission
file's `main_tree_to_execute` is created and ticked — so a mission can reference
any `BehaviorTree ID` defined under `trees/`.

## The competition run is shared

The RoboSub 2026 task ordering (coin flip → gate → slalom → bins → torpedoes →
octagon, with the sim-derived transit legs) lives once in
`trees/competition.xml` as `CompetitionRun`. The four `pool_*` missions are
identical except for a handful of pool-side numbers, so each is just that data
plus `<SubTree ID="CompetitionRun"/>` — no copy-pasted task list to keep in
sync:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="CompetitionPoolA">
  <BehaviorTree ID="CompetitionPoolA">
    <Sequence name="pool_a">
      <!-- Pool-side + strategy values; task trees read them as {@key}. -->
      <Script code="@gate_exit_yaw := 0.34906585;
                    @torp_search_yaw := -2.35619449;
                    @octagon_search_yaw := -0.61086524;
                    @target_animal := 1"/>
      <SubTree ID="CompetitionRun"/>
    </Sequence>
  </BehaviorTree>
</root>
```

- `Script` with the `@` prefix writes to the **root blackboard** (BT.CPP >= 4.6),
  which any subtree can read without port remapping.
- Inside `CompetitionRun`, `ForceSuccess` gives the competition semantic "a
  failed task shouldn't end the run"; it's left off for tasks that must succeed.
- Because it's an ordinary tree, missions/trees can also use `Repeat`, `Timeout`,
  `Fallback`, etc. — no custom config schema to learn.

To run a one-off subset (say gate only), copy any mission file, replace the
`<SubTree ID="CompetitionRun"/>` line with just the `SubTree`s you want, and
point `mission:=` at it.

## Pool-side values

What used to be the `PoolAOrD` condition is now data. Trees reference root
blackboard entries (`<AttSetpoint yaw="{@gate_exit_yaw}"/>`), and each pool
mission sets them in its `Script` node:

| Entry                | Pools A/D    | Pools B/C    | Used by            |
|----------------------|--------------|--------------|--------------------|
| `gate_exit_yaw`      | 0.34906585   | -0.34906585  | gate               |
| `torp_search_yaw`    | -2.35619449  | 2.35619449   | torp               |
| `octagon_search_yaw` | -0.61086524  | 0.61086524   | octagon            |
| `target_animal`      | 1 (sawfish)  | 1 (sawfish)  | gate, torp         |

`target_animal` is the RoboSub 2026 gate side choice (`1` = sawfish, `0` = reef
shark), also used by the torpedoes. It's mission-wide strategy, not pool-side,
so it's the same in every file — set it once and gate/torp read
`{@target_animal}`. The bins tree instead uses the launch-time `role` parameter:
`SURVEY` targets the two fire bins (class `0`) and `SEARCH` targets the two
blood bins (class `1`) with the `torp_fire_blood` down-camera detector.

New pool-dependent numbers (headings, distances, depths) should follow the
same pattern: assign `@key` in each pool mission's `Script` and read it with
`{@key}` in the task tree, instead of branching on a condition node.

## Vision primitives

Perception-driven behavior is built from four generic leaves that talk to
`sub_vision` (`src/nodes/vision.cpp`). Task-specific behavior ("align with the
gate") is XML composition of these in `resources/trees/` — not new C++.

| Node | Kind | Does |
|------|------|------|
| `LoadModel` | action | asks the camera's `sub_vision` node to activate a task's detector (async `load_model` service call; no-op if already active) |
| `DetectionVisible` | condition | SUCCESS iff a matching detection newer than `max_age_msec` exists |
| `WaitForDetection` | action | waits up to `timeout_msec` for a matching detection that arrived **after the node started** (a stale frame can't satisfy it) |
| `AlignToDetection` | action | closed loop: re-targets yaw (and optionally depth) once per new camera frame until the bearing is within `tolerance` |

All four share the filter ports `camera` (default `front`; `down` is already
wired in `src/vision_client.cpp` for when a down-camera vision node exists),
`task`, `class_id`, and `min_score`. Detections arrive as
`sub_vision_interfaces/DetectionArray` on `vision/detections`, cached per
camera by `VisionClient`; the steering signal is the per-detection
`bearing_horizontal`/`bearing_vertical` that `sub_vision` computes from the
camera intrinsics (positive = right of / below center).

`AlignToDetection` references its absolute yaw/depth targets to the *measured*
state (commanded setpoint minus control error), so re-issuing a command on
every frame stays convergent instead of integrating the correction. Extra
ports: `yaw` (default true), `depth` (default false), `tolerance` (rad),
`depth_gain` (m of depth per rad of vertical bearing), `max_depth_step`,
`timeout_msec`.

**Kill-switch policy:** the passive leaves (`LoadModel`, `DetectionVisible`,
`WaitForDetection`) run while killed so the vision path is testable on the
bench; `AlignToDetection` moves the sub and fails immediately when killed.

The worked example is `trees/gate.xml`:

```xml
<BehaviorTree ID="AlignWithGate">
  <ForceSuccess>
    <Sequence name="align_with_gate">
      <LoadModel task="gate"/>
      <WaitForDetection task="gate" timeout_msec="15000"/>
      <AlignToDetection task="gate" tolerance="0.05" timeout_msec="30000"/>
    </Sequence>
  </ForceSuccess>
</BehaviorTree>
```

`missions/vision_test.xml` (LoadModel + WaitForDetection, no motion nodes) is
the bench smoke test for the whole mission↔vision path: run it next to a live
`sub_vision` node and a camera, and it succeeds as soon as the detector sees
the task object.

## Action implementation

Every custom node used by the packaged mission trees has a concrete C++
implementation. Mission sequencing and recovery belong in XML; reusable
movement, vision, and actuator mechanics belong in typed C++ actions. New
hardware behavior must use a typed ROS topic, service, or action rather than a
logging-only placeholder or raw serial write.
