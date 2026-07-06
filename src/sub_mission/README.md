# sub_mission

Behavior-tree mission executive. A mission is selected with a single ROS
parameter, `mission`, that names a **BehaviorTree.CPP v4 XML file** — the same
format Groot2 edits and Nav2 uses for its behaviors. This replaces the old
pile of boolean parameters (`POOL_A`, `POOL_B`, `GATE`, `BUOY`, ...).

## Running

```sh
# By name -> resources/missions/<name>.xml from the installed share directory
ros2 run sub_mission mission --ros-args -p mission:=pool_a

# Or with an explicit path (useful while iterating without rebuilding)
ros2 run sub_mission mission --ros-args -p mission:=/path/to/my_mission.xml
```

`restart` takes the same parameter and forwards it to each `mission` run it
spawns. Launching without `mission` (or with a bad name) prints the available
mission names and exits — before the node starts waiting on the kill switch.

## Layout

```
resources/
  trees/       # task library, one file per element:
               #   gate, slalom, bins, torp, octagon, coin_flip, buoy (legacy),
               #   competition (the shared 2026 run), plus test trees
  missions/    # runnable entrypoints (pool_a..pool_d, prelim, pool_test,
               #   vision_test, pid_tuning)
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
      <Script code="@gate_exit_yaw := -0.34906585;
                    @torp_search_yaw := 2.35619449;
                    @octagon_search_yaw := 0.61086524;
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
| `gate_exit_yaw`      | -0.34906585  | 0.34906585   | gate               |
| `torp_search_yaw`    | 2.35619449   | -2.35619449  | torp               |
| `octagon_search_yaw` | 0.61086524   | -0.61086524  | octagon            |
| `target_animal`      | 1 (sawfish)  | 1 (sawfish)  | gate, bins, torp   |

`target_animal` is the RoboSub 2026 gate side choice (`1` = sawfish, `0` = reef
shark), scored again at the bins and torpedoes. It's mission-wide strategy, not
pool-side, so it's the same in every file — set it once and gate/bins/torp read
`{@target_animal}`.

New pool-dependent numbers (headings, distances, depths) should follow the
same pattern: assign `@key` in each pool mission's `Script` and read it with
`{@key}` in the task tree, instead of branching on a condition node.

## Blackbox actions

The perception-driven primitives (`AlignWithGate`, `PassSlalomChannel`,
`ApproachTorpBoard`, ...) are **blackbox stubs**: they log a description and
return a fixed status so a full mission tree loads and dry-runs in the sim
before the primitive is wired up. They live in a single self-documenting
registry, `blackbox_actions` in `src/nodes/node.cpp`, grouped by task.

Real, closed-loop primitives (`PosSetpoint`, `AttSetpoint`, `MoveRelative`,
`Spin`, `WaitUntilHit`, `SurfaceAtOctagon`) are full `BT::StatefulActionNode`s
in the same file — they publish setpoints and return `RUNNING` until the goal
is reached. To promote a blackbox to a real action:

1. Write a `StatefulActionNode` for it (`PosSetpointAction` is the template);
   it can publish setpoints and read `node_.control_errors` / the `{@...}`
   blackboard.
2. `registerBuilder<YourAction>("Name", ...)` it in `registerMissionNodes()`.
3. Delete its row from `blackbox_actions`.

Whatever remains in `blackbox_actions` is, by definition, not implemented yet.
