# Simulation

## Overview

The simulator is [Stonefish](https://github.com/patrykcieslak/stonefish), a physics engine built for underwater vehicles. `stonefish_ros2` provides the ROS2 bridge. A custom fork (`avbotz/stonefish`, branch `claude-extra-features`) is required; it is built inside the workspace by the `stonefish_vendor` package from the submodule at `src/sub_sim/stonefish_vendor/stonefish`, so no global install is needed (see [setup.md](setup.md#stonefish) for switching versions).

Scenarios are described with Stonefish `.scn` XML files. Thalassic generates them at launch time from Jinja2 templates so task-object positions can be randomized.

## Scenario Generation Pipeline

`sim_launch.py` runs this pipeline before starting Stonefish:

1. **Render robot layout** — `generate_robot.py` renders `data/robots/marlin_v3/layout.scn.j2` into a temporary `.scn` file. The template uses a `thruster.j2` macro to stamp out a thruster for each entry in the vehicle description's `thrusters:` block, and gives the sensors their noise.

1. **Convert to URDF** — `robot_scenario_to_urdf.py` parses the rendered robot `.scn` and emits a URDF, which `robot_state_publisher` publishes as `robot_description` so RViz and Foxglove can draw the robot's meshes.

1. **Render pool scenario** — `randomize_locs.py` renders `scenarios/woollett.scn.j2` with the `DX/DY/DZ/DYAW` fuzz arguments, the water `current` and the starting `course`, producing the final simulation scenario that Stonefish loads.

## Scenario Template Variables

The Jinja2 pool scenario (`woollett.scn.j2`) has access to these helpers:

| Variable / Function | Description |
|---|---|
| `fuzz(base, mag)` | `base ± uniform(mag)` |
| `fuzz_z(base, mag)` | Same but clamped to ≥ 0 |
| `rand(a, b)` | `uniform(a, b)` |
| `choose(options)` | Pick one item at random |
| `DX`, `DY`, `DZ`, `DYAW` | Per-axis fuzz magnitudes passed from launch args |
| `course` | The dock the robot starts from (`A`, `B1`, `C1` or `D`), from the `course` launch argument |
| `render_robot(x, y, z, roll, pitch, yaw)` | Renders the robot at that pose and returns the `.scn` path to `<include>` |
| `PI` | `math.pi` |

Use `seed` in the launch argument to get reproducible scenarios. Without one, the launch picks a random seed and, with `record:=true`, records it in the run's bag ([recording.md](recording.md)).

## Simulation Nodes

`sim_launch.py` runs these in place of the vehicle's drivers, beside everything in `common_launch.py`. The `sub_sim_sensors` bridges run together inside a single composable-node container (`sim_sensors_container`); the others are processes of their own:

| Node | Package | Purpose |
|---|---|---|
| `stonefish_simulator` | `stonefish_ros2` | Physics engine + sensor simulation |
| `robot_state_publisher` | `robot_state_publisher` | Publishes the URDF on `robot_description`. Nothing publishes the `joint_states` it reads (Stonefish's are on `sim/joint_states`), so it publishes no TF; the grabber and dropper frames come from `static_transforms` |
| `sim_dvl_remapper` | `sub_sim_sensors` | `stonefish_ros2/DVL` → `dvl/velocity` (`TwistWithCovarianceStamped`) and `altitude` |
| `sim_pressure_to_depth` | `sub_sim_sensors` | `sim/pressure` → `odometry/depth` (ENU z), as `sub_low` reports it on the vehicle |
| `sim_thrusters` | `sub_sim_sensors` | `control/thrusters` → Stonefish, with ESC lag, the board's command watchdog and fault injection |
| `sim_imu_remapper` | `sub_sim_sensors` | Stonefish NED IMU → ENU `imu/data` |
| `sim_kill_switch` | `sub_sim_sensors` | `sim/kill_switch` → `kill_switch` after startup delay (`off_delay`; 0 starts released) |
| `sim_torpedo_launcher` | `sub_sim_sensors` | Torpedo launch service |
| `sim_dropper` | `sub_sim_sensors` | Dropper service, for both droppers |
| `sim_grabber` | `sub_sim_sensors` | `set_grabber` service: closing the jaws on an object glues it to the robot, opening them lets go |

## Sensor Simulation

### DVL

Stonefish publishes `stonefish_ros2/DVL` on `sim/dvl`. `sim_dvl_remapper` converts it to a `geometry_msgs/TwistWithCovarianceStamped` on `dvl/velocity` and a `sensor_msgs/Range` on `altitude` — the same topics the hardware WaterLinked driver publishes. Like the driver it drops readings without bottom lock, and it reports a velocity variance of at least `velocity_variance` so the EKF does not treat the DVL as perfect.

Like the real A50, a reading arrives `latency` (0.15 s) after the instant it is stamped with, which is what the EKF's `smooth_lagged_data` is for, and `dropout_probability` (0) of the readings can be dropped to rehearse losing lock near walls and the floor. Both can be changed while the simulation runs:

```bash
ros2 param set /marlin_v3/sim_dvl_remapper dropout_probability 0.5
```

### Sensor noise and current

By default the simulated sensors carry the noise of the real ones — DVL a few mm/s plus 1% of speed, IMU about 0.1° of attitude and 0.2°/s of rate noise with ~10°/hr of yaw drift (the NaviGuider has no magnetometer), pressure about 3 mm — and the water moves at a pool-pump `current` of a few cm/s. A controller that only works in still water with perfect sensors will not work in the pool. For a clean run:

```bash
pixi run sim sensor_noise:=false current:="0 0 0"
```

### Thrusters

`sub_control` publishes the eight normalized commands as one stamped `ThrusterCommand` on `control/thrusters`, the same message `sub_low` sends on to the real thrusters. `sim_thrusters` stands in for `sub_low`, the board and the ESCs. Like them, it stops every thruster when the commands stop or arrive older than `command_timeout` (0.5 s), so killing `sub_control` stops the simulated vehicle as it would the real one. It passes the commands to Stonefish as one `sim/thruster_setpoints` array, and each Stonefish thruster stands in for the ESC with its first-order `rotor_dynamics` (0.1 s, the Basic ESC's measured response, in `thruster.j2`) and converts the result to thrust with the T200 curve in its `linear_interpolation` thrust model. Stonefish reports each thruster's setpoint, rpm, thrust and torque on `sim/thruster_states`.

To fail a thruster mid-run, scale its command (0 is dead, 0.5 half thrust; values outside [0, 1] are rejected):

```bash
ros2 param set /marlin_v3/sim_thrusters effectiveness "[1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0]"
```

Nothing declares it automatically, as on the vehicle: the integrators take up its share until it is declared with `failed_thrusters`, and then `sub_control` allocates around it ([control.md](control.md#thruster-failure)).

### Cameras

The front and down cameras are always simulated, where the vehicle description mounts them and as the real ones: the front camera (Logitech C922) at 1920 × 1080, 24 Hz and a 70.4° horizontal field of view, the down camera (FLIR Blackfly S) at 1280 × 1024 and 5 Hz. Each publishes `image_raw` and `camera_info` under its name, as the vehicle's drivers do. The depth camera (`sim/depth_camera`) is added with `depth_camera:=true`, and the segmentation camera (`sim/segment`) with `segmentation_camera:=true`, whose images the bag records as training labels for [remora](../../remora/README.md): nothing in the stack reads either, and every camera costs GPU time (see [Performance](#performance)).

### Droppers

Each of the two droppers holds one ball, `dropper_ball_0` and `dropper_ball_1` in `layout.scn.j2`, glued to the robot above the dropper's gate. `set_dropper` with `open` turns the gate's servo, and once it has turned past `release_angle` `sim_dropper` releases the ball of the `dropper_id` opened. A dropper drops once; closing it before the gate has turned keeps its ball. Each ball reports where it is on `sim/dropper_ball_<id>/odometry`.

## Simulation Settings

Fixed in `sim_launch.py`:

| Setting | Value |
|---|---|
| `simulation_rate` | 500 Hz |
| `publish_clock` | true: every node runs on the simulation's `/clock` (`use_sim_time`) |
| `state_publish_rate` | 100 Hz: `/clock` and the robot's joint and thruster states |
| `OMP_WAIT_POLICY` | `PASSIVE`, for every node |
| Fast DDS profile | `config/stonefish_fastdds.xml`, for Stonefish only: a 64 MB shared-memory segment for the camera images |

Launch arguments:

| Argument | Default | |
|---|---|---|
| `course` | D | The dock the robot starts from on the course map: `A`, `B1`, `C1` or `D` (see [Course Layout](#course-layout)) |
| `window_width` × `window_height` | 1280 × 720 | Stonefish window |
| `rendering_quality` | medium | `low`, `medium` or `high`, for the window and the cameras |
| `current` | `"0.02 0.03 0.0"` | Uniform water current, ENU m/s (3.6 cm/s, a pool pump) |
| `sensor_noise` | true | Give the DVL, IMU and pressure sensor the noise of the real ones |
| `segmentation_camera` | false | Add the segmentation camera (`sim/segment`): the object each front-camera pixel shows, which `record:=true` keeps for remora's labels ([recording.md](recording.md#what-a-bag-holds)). It is paired with the front camera, so it renders in the front camera's frames instead of taking a turn of its own |
| `depth_camera` | false | Add the depth camera (`sim/depth_camera`) |
| `headless` | false | Run `stonefish_simulator_nogpu`: no window, no GPU or display, and no cameras (so no vision); physics, the DVL, IMU, pressure sensor and actuators as usual. For control, safety and mission-logic tests, and CI |

## Regression test

`src/sub_bringup/test/test_sim.py` runs the whole stack against the headless simulator on every `pixi run test` (and so in CI), on a ROS domain and Foxglove port of its own: it checks that `sub_control` arms after the simulated kill switch releases, that the `pid_tuning` mission (a depth step, a 2 m move, a quarter turn) finishes with `SUCCESS`, and that unloading the IMU bridge cuts the thrusters. It takes a minute or two and needs no GPU. Perception is not part of it: the simulator's renders are not what the detectors were trained on, so vision has to be checked on recorded pool footage (not automated yet).

## Performance

The simulation keeps real time (`/clock` advancing one second per second) on a laptop with integrated graphics, and the cameras keep their rates evenly: on a Radeon 780M (October 2026, `balanced` power profile, Foxglove connected) the front camera's frames arrived 41.7 ms apart on average, 50 ms at the 99th percentile, with 1 of 2146 late. What limits it:

- **Physics** runs on its own thread, paced by the wall clock: when a step cannot finish in time, Stonefish drops the time and warns *Simulation cannot keep up with real time*. A 500 Hz step takes about 1 ms. The octagon's bottles and ladles, resting on the table, cost as much as everything else together until they settle and sleep (`sleeping_thresholds` in `layout.scn.j2`). `/clock` and the robot's state are published from this thread, at a cost per subscriber, so they go out at `state_publish_rate` rather than every step. Every node on sim time subscribes to `/clock`, which at 250 Hz took a Python node 20% of a core, so it is 100 Hz, the fastest any node needs (the thruster setpoints).
- **Cameras** are rendered on the main thread, which renders one camera per frame as it becomes due (it checks every 4 ms and renders nothing when nothing is due). The fastest camera sets the pace: the slower cameras and the window are rendered in the gap right after its frames, so they do not delay its next one, and the window is redrawn at most 20 times a second (at least 5) while cameras run. The GPU sets how much fits: a front-camera frame takes about 20 ms on the 780M, which the power limit keeps at its lowest clock, 800 MHz, while the CPU is busy (the `performance` power profile did not raise it). Shrinking the window, `rendering_quality:=low` (which changes the camera images too) or fewer cameras all leave more room. Keep the robot's visual meshes low-poly; they are drawn in every view and in each of the four shadow passes.
- **Unused work is skipped.** A camera nobody subscribes to (in any transport) is not rendered; its `camera_info` keeps its rate. `sub_vision` subscribes its camera only once a model is loaded, so with no model, no Foxglove camera panel and no recording, the GPU only draws the window: at most 30 times a second, and not at all while it is minimized.
- **Other nodes** compete for the same cores. ONNX Runtime on the CPU runs 4 sleeping threads instead of a spinning one per core (see `sub_vision`'s README). The sensor container runs a single-threaded executor and the Foxglove bridge 2 threads with `sysinfo` off: by default each runs a thread per core, which took 13% and 11% of a core while idle. Nodes that do not read the clock (`static_transforms`, the sensor remappers) are not on sim time. OpenCV's OpenMP workers (in `cv_bridge` and the compressed image transport) spin between frames unless `OMP_WAIT_POLICY=PASSIVE`, which took several cores from the physics thread.
- **Images between processes** go through Fast DDS shared memory, as ~64 KB fragments in the publisher's segment. The default segment is 512 KB, and a raw front-camera frame is 6.2 MB: it was overwritten before subscribers read it, so best-effort subscribers (`sub_vision`) got half the frames and reliable ones (the Foxglove bridge) waited hundreds of milliseconds for repairs, making the cameras stutter and lag in Foxglove. Stonefish therefore runs with its own profile and a 64 MB segment (see the comment in `stonefish_fastdds.xml`). It publishes the camera images; the other nodes' images (`vision_depth`'s maps) are for viewing and keep the default.

Our Stonefish fork adds to this: the physics thread never waits for the renderer, camera images are published on their own thread (so a Foxglove client subscribing to `.../compressed` costs no frame rate), and actuator setpoints are handled on their own thread, a few milliseconds after they are published rather than after the frame being rendered. For the frame time it also computes the caustics only for the cameras, at most 30 times a second and only down to the pool's `<bottom>`, skips the wave simulation of a flat surface and the screen-space reflections a flat surface does not need, renders the surface's mirror image without the sun's shadows, and takes fewer shadow samples; it draws each mesh with its own index count, where Stonefish drew four times as many indices.

## Task Models

3D models for competition tasks live in `src/sub_sim/sub_sim/data/models/`:

| Task | Objects |
|---|---|
| Gate | gate with the Survey & Repair and Search & Rescue images, on either side at random |
| Slalom | slalom poles: white 1.524 m either side of red |
| Bin | four bins on a PVC pipeline, two with the fire image (Survey & Repair) and two with blood (Search & Rescue) |
| Torpedoes | board with both roles' images, in one of two layouts (`torpboard_v1`, `torpboard_v2`) at random |
| Octagon | octagon with its Survey, Search, Repair and Rescue images, table, bottles, ladles |
| Path | orange path markers, 1.2 m long |
| Pools | woollett, natatorium |

The Woollett pool model (`data/models/pools/woollett/`) is the default scenario.

## Course Layout

`woollett.scn.j2` lays out the RoboSub 2026 course map of Woollett. The map is drawn not to scale, so the positions were measured off it using the pool's walls for scale. The lane dividers (`x = 0` and `x = 12.351`) split the pool into three course areas: A/B1 (A and B1 start in the same area), C1 and D. Each area has a gate, two paths, Avoid Debris (the slalom), Recon (the bin), Deploy (the torpedo board) and Restore (the octagon). The map draws the slalom as one set of pipes; it is three, laid out as in the handbook (2 m apart, staggered), so the second path is moved past them. The robot starts at the dock picked by `course`, facing away from the docks' wall, across the pool (toward its gate for A, C1 and D). Only its own course's table has the bottles and ladles; the other tables are empty (see the comment in the template). The pingers are not simulated.

```bash
pixi run sim course:=C1
```

Each prop's folder holds one Stonefish body:

- `<prop>.obj` — the whole prop as one mesh, with UVs into `<prop>.png`
- `<prop>.png` — its texture. Images and solid colours share one texture, so the prop needs one look and draws in one call. Stonefish uses a look's `rgb` as linear colour but decodes textures as sRGB, so a colour that should match a look `rgb` of `c` is stored as sRGB(`c`).
- `<prop>_collision.obj` — a low-poly mesh of what can be hit, without flush detail like screws. Static props use it as a triangle mesh (`convex="false"`), so openings such as the torpedo holes stay open. Dynamic props (bottles, ladles) are always collided as the convex hull of this mesh, and Stonefish computes their mass and drag from it; set `<mass>` when the low-poly mesh is much thinner than the real prop.
- `<prop>.scn` — the `<static>` (or `<dynamic>`), placed by the `x y z roll pitch yaw` include arguments

A prop with variants that only look different has a visual mesh for each, picked by an include argument: the gate's `visual` is `gate.obj`, Survey & Repair left of the way through, or `gate_swapped.obj`, its UVs swapping the two images.

The name of the body is the prop's name, which the segmentation camera labels its pixels with and remora matches to a class by prefix. Add a new prop the same way: in Blender, bake its materials into one texture and export the visual and collision meshes. Splitting a prop into one body per colour multiplies its draw calls, collision objects and segmentation IDs.

## Robot Model

The Marlin V3 robot definition lives in `src/sub_sim/sub_sim/data/robots/marlin_v3/`:

- `layout.scn.j2` — Jinja2 template for the full robot Stonefish scenario (links, joints, sensors, actuators)
- `thruster.j2` — macro for a single thruster definition
- `torpedo.j2` — macro for a torpedo: a free body glued in its tube until `sim_torpedo_launcher` releases it
- `frame/`, `dropper/`, `grabber/`, `torpedoes/`, etc. — mesh files (`.obj`)

The thrusters are not written in `layout.scn.j2`: `sim_launch.py` reads them from the `thrusters:` block of `src/sub_bringup/config/vehicles/marlin_v3.yaml` and renders them into the scenario, as the launch files pass the same block to `sub_control`'s allocator and to the TF tree. Moving a thruster is one edit. See [Decision 7](decisions.md).
