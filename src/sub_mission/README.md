# sub_mission

Behavior-tree mission executive. A mission is selected with a single ROS
parameter, `mission`, that names a **BehaviorTree.CPP v4 XML file** — the same
format Groot2 edits and Nav2 uses for its behaviors. This replaces the old
pile of boolean task-selection parameters.

## Running

```sh
# By name -> resources/missions/<name>.xml from the installed share directory
ros2 run sub_mission mission --ros-args -r __ns:=/marlin_v3 -p mission:=pool_a

# Or with an explicit path (useful while iterating without rebuilding)
ros2 run sub_mission mission --ros-args -r __ns:=/marlin_v3 -p mission:=/path/to/my_mission.xml
```

Use the same namespace as the rest of the stack when starting `mission` or
`restart` manually. The bringup launch files use `robot_name:=marlin_v3` by
default, so manual `ros2 run` commands normally need `-r __ns:=/marlin_v3`.
Without it, mission commands publish to root-level topics such as
`/motion_setpoint`, and the namespaced controller will not receive them.

`restart` takes the same parameter and forwards it to each `mission` run it
spawns. Started without the `mission` parameter (or with a bad name), the
`mission` node prints the available mission names and exits — before it starts
waiting on the kill switch; `restart` exits at startup on the same mistakes.

```sh
ros2 run sub_mission restart --ros-args -r __ns:=/marlin_v3 -p mission:=pool_a
```

Groot2's Monitor mode can watch the tree run: connect it to port 5555 (the
`groot_port` parameter) on the machine running `mission`. See
[docs/mission.md](../../docs/mission.md#live-monitoring-groot2).

## Layout

```
resources/
  trees/       # task library, one file per element:
               #   gate, slalom, bins, torp, octagon, return_home, coin_flip,
               #   competition (the shared 2026 run), hard_coded_* (the same
               #   tasks dead-reckoned, without vision), plus test trees
  missions/    # runnable entrypoints (pool_a..pool_d, prelim, pool_test,
               #   vision_test, pid_tuning, hard_coded_*, and single tasks:
               #   bins_test, gate_slalom_test, slalom_three_test,
               #   slalom_perception_hold_test)
  Project.btproj  # Groot2 project covering both, open it to edit any tree
```

**trees vs missions.** A *tree* is a reusable, self-contained task (`GateMission`,
`SlalomMission`, ...) that is never selected directly. A *mission* is a runnable
entrypoint you pass to `mission:=`. Everything is plain BT.CPP XML. All files in
`resources/trees/` are registered with the factory first, then the mission
file's `main_tree_to_execute` is created and ticked — so a mission can reference
any `BehaviorTree ID` defined under `trees/`.

## The competition run is shared

The RoboSub 2026 task ordering (coin flip → gate → slalom → torpedoes →
octagon → return home, with the sim-derived transit legs) lives once in
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
                    @octagon_search_yaw := -0.61086524"/>
      <SubTree ID="CompetitionRun"/>
    </Sequence>
  </BehaviorTree>
</root>
```

- `Script` with the `@` prefix writes to the **root blackboard** (BT.CPP >= 4.6),
  which any subtree can read without port remapping.
- Only the coin flip may end the run. Every later task (gate, slalom,
  torpedoes, octagon, return home) runs inside the same wrapper:

  ```xml
  <SavePose name="torp_entry" yaw="{@task_entry_yaw}"/>
  <Fallback name="torp">
    <Timeout msec="150000"><SubTree ID="TorpMission"/></Timeout>
    <SubTree ID="RecoverTaskEntry"/>
  </Fallback>
  ```

  `Timeout` is the task's time budget (wall clock), sized so the tasks fit
  the handbook's 15 minutes of performance time. A task that fails or runs
  over is halted wherever it was, often mid-sweep and facing anywhere;
  `RecoverTaskEntry` stops the vehicle and turns it back to the heading it had
  when the task began, so the next transit still heads along the course.
- Transit legs are `<SubTree ID="Transit" z="-1.0" x="3.0" y="-1.0"/>`:
  `Stop` whatever the last task left running, take the depth, hop `x` forward
  and `y` left. A move in a transit that times out is logged and the run
  carries on.
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

The yaws are odom yaws in radians, counter-clockwise from the heading the sub
starts on, like every yaw in the trees (see Frames in `docs/mission.md`). A
test checks that every mission running `CompetitionRun` sets all three.

The gate is not pool-side data: the vehicle always takes the gate's left
opening and adopts the role its images show (`SelectGateRole`), and the slalom
passes left of each red pipe to match. The `role` parameter is the role until
the gate is read; `bins_test`, with no gate, runs on it.

New pool-dependent numbers (headings, distances, depths) should follow the
same pattern: assign `@key` in each pool mission's `Script` and read it with
`{@key}` in the task tree, instead of branching on a condition node.

## Vision primitives

Perception-driven behavior is built from generic leaves that talk to
`sub_vision` (`src/nodes/vision.cpp`). Task-specific behavior ("align with the
gate") is XML composition of these in `resources/trees/` — not new C++.

| Node | Kind | Does |
|------|------|------|
| `LoadModel` | action | asks the camera's `sub_vision` node to activate a task's detector (async `load_model` service call; no-op if already active) |
| `DetectionVisible` | condition | SUCCESS iff a matching detection newer than `max_age_msec` exists |
| `WaitForDetection` | action | waits up to `timeout_msec` for a matching detection that arrived **after the node started** (a stale frame can't satisfy it) |
| `SearchForDetection` | action | moves at a fixed velocity, holding the heading, until a matching detection arrives, then stops; or sweeps left then right |
| `AlignToDetection` | action | closed loop: re-targets yaw (and optionally depth) once per new camera frame until the bearing is within `tolerance` |

They share the filter ports `camera` (default `front`; `down` reads the
`sub_vision_down` instance, see `src/vision_client.cpp`),
`task`, `class_id`, and `min_score` (`LoadModel` only the first two).
`AlignToDetection` takes the front camera only, as do the other nodes
that steer on a bearing as a turn or a sideways move (`LateralAlignToDetection`,
`SweepCheck`, `ForwardSweepAlign`, `SweepAngle`, `ForwardContinuousAlign`,
`OrientToDetectionAtDist`): they fail on `camera="down"`. The `Down*` nodes
read the down camera and have no `camera` port. Detections arrive as
`sub_vision_interfaces/DetectionArray` on `vision/detections`
(`vision/detections_down` for the down camera), cached per
camera by `VisionClient`; the steering signal is the per-detection
`bearing_horizontal`/`bearing_vertical` that `sub_vision` computes from the
camera intrinsics (positive = right of / below center). The vision nodes turn
them into the mission's frames: a target right of center is a turn clockwise,
to a smaller yaw, and one below center on the front camera is a smaller z.

`AlignToDetection` references its absolute yaw/depth targets to the *measured*
state (`odometry/filtered`), so re-issuing a command on every frame stays
convergent instead of integrating the correction. Extra
ports: `yaw` (default true), `depth` (default false), `tolerance` (rad),
`depth_gain` (m of depth per rad of vertical bearing), `max_depth_step`,
`timeout_msec`.

**Kill-switch policy:** the passive leaves (`LoadModel`, `DetectionVisible`,
`WaitForDetection`) do not check the kill switch, as they move nothing; the
ones that move the sub fail immediately when killed. The executive itself
ticks a tree only while the sub is unkilled: `mission` waits for the release
before the first tick and ends the run on a kill, whatever the tree.

The task trees also use task mechanics built on these, listed in
`docs/mission.md`: sweeps, approaches that steer while moving
(`ForwardContinuousAlign`, `ForwardFixedTransit`), sideways and down-camera
centering (`LateralAlignToDetection`, the `Down*` nodes), the gate's role
(`SelectGateRole`) and the torpedo board's face (`OrientToDetectionAtDist`).

The worked example is `trees/gate.xml`:

```xml
<BehaviorTree ID="AlignWithGate">
  <ForceSuccess>
    <Sequence name="align_with_gate">
      <LoadModel task="gate"/>
      <WaitForDetection task="gate" class_id="8" timeout_msec="15000"/>
      <AlignToDetection task="gate" class_id="8" tolerance="0.05" timeout_msec="30000"/>
    </Sequence>
  </ForceSuccess>
</BehaviorTree>
```

`missions/vision_test.xml` (LoadModel + WaitForDetection, no motion nodes) is
the smoke test for the whole mission↔vision path: run it next to a live
`sub_vision` node and a camera, release the kill switch, and it succeeds as
soon as the detector sees the task object. It commands no motion, but
`sub_control` holds the vehicle where it is while it runs, so run it in the
water or the sim, not on the bench.

## Action implementation

Every custom node used by the packaged mission trees has a concrete C++
implementation. Mission sequencing and recovery belong in XML; reusable
movement, vision, and actuator mechanics belong in typed C++ actions. New
hardware behavior must use a typed ROS topic, service, or action rather than a
logging-only placeholder or raw serial write.

## Tests

```sh
pixi run colcon test --packages-select sub_mission && pixi run colcon test-result --verbose
```

`test/test_mission.cpp` loads every file in `resources/missions/` against the
registered C++ nodes, so a port the XML sets but the node no longer declares
fails here instead of at launch on the pool deck. It also drives the movement
and vision nodes, `Transit` and `RecoverTaskEntry` against a scripted
`control/error` and camera frames, and checks which way they move the vehicle;
no other ROS node needs to be running.
