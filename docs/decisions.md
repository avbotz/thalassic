# Decisions

Short records of the decisions in this repository.

## 1. pixi + RoboStack instead of apt ROS, rosdep, and a venv

**Was:** `install.sh` added the ROS apt repository at whatever version was newest that day, installed `ros-jazzy-desktop`, built Stonefish into `/usr/local` with `sudo`, downloaded the Spinnaker SDK from a Google Drive link, created a `--system-site-packages` venv, and pip-installed three packages that were listed nowhere. The Dockerfile repeated a different subset of that by hand. Two machines set up a week apart had different simulators, different `torch` builds, and different ROS patch versions.

**Now:** `pixi.toml` declares every dependency (ROS 2 from the `robostack-jazzy` channel, compilers, PCL, SDL2, PyTorch, and so on) and `pixi.lock` pins exact builds for `linux-64` and `linux-aarch64`. `pixi install --frozen` reproduces the environment bit for bit, without `sudo`, on any Linux distribution and the Jetson.

**Why pixi:**

- One lockfile covers conda and PyPI packages, native libraries, and the compiler, which `requirements.txt` + `rosdep` never could.
- Nothing global. Different checkouts (branches, older releases) coexist with different dependency sets.
- RoboStack itself recommends pixi, and packaged every ROS package we use except `rtabmap` and the submodules of decision 3.
- The same manifest serves developers, the container, CI, and the vehicle, so "works on my machine" differences shrink to hardware.

**Why not nix:** the reproducibility is higher, but it is much more difficult to configure, especially in regards to the Jetson where we would probably need to use NixOS.

**Cost:** the Jetson's GPU stack (TensorRT, CUDA bindings) is not in the lockfile because NVIDIA only ships Orin builds through JetPack. See [Decision 5](#5-gpu-inference-on-the-jetson-comes-from-jetpack).

## 2. Stonefish is built inside the workspace

**Was:** a global `/usr/local` build from a branch tip, installed with `sudo` by `install.sh` or baked into the Docker image. Trying another Stonefish version meant overwriting every machine's global install, and `stonefish_ros2` demands an exact library version, so mismatches failed at configure time.

**Now:** `src/sub_sim/stonefish_vendor` is an `ament_vendor` package that builds the git submodule `stonefish_vendor/stonefish` (AVBotz fork, pinned by commit) and installs it under `install/opt/stonefish_vendor`. An ament environment hook makes it visible to `stonefish_ros2`. Switching versions is `git checkout` in the submodule plus a rebuild of two packages, and the submodule pointer in git records exactly which simulator every commit was tested with.

`colcon.meta` declares that `stonefish_ros2` depends on `stonefish_vendor`, because that submodule's own `package.xml` cannot know about our vendor package and colcon needs the edge to order the build. The same file carries the one build flag a submodule needs under the conda toolchain: `libwaterlinked` uses `std::thread` without linking pthread, which passes on hosts whose glibc merged libpthread but not against conda-forge's glibc 2.28 sysroot, so it gets `-pthread` there. The FLIR driver gets two more: an implicit `<cstdint>` include (its headers rely on a transitive include that GCC 15 dropped, and the package builds with `-Werror`), and `--allow-shlib-undefined`, because the prebuilt Spinnaker library it downloads references glibc 2.34+ symbols that the conda sysroot cannot verify at link time; they resolve against the host's glibc at runtime, which is what happens on any apt-based install too. Fixes for submodules go in `colcon.meta` first and upstream second, so the workspace never depends on a patched checkout.

## 3. Submodules stay for packages RoboStack does not ship

`stonefish_ros2`, `waterlinked_dvl`, and `flir_camera_driver` remain git submodules. The FLIR driver downloads the Spinnaker SDK during its own build (from a mirror the upstream driver maintains), so the SDK is no longer installed system-wide and the Google Drive step is gone. The pieces the SDK needs on the vehicle (a `flirimaging` group, udev rules, and the GenTL producer path) are applied by `deploy/jetson/setup.sh` and the activation script.

## 4. DDS discovery is limited per developer

**Was:** default Fast DDS settings. Anyone running the simulator on the same network as another developer saw both `/marlin_v2` graphs, and stale shared-memory segments after crashes produced the `rmw_create_node` error the old README worked around with `pkill -f ros`.

**Now:** `scripts/setup.sh` applies a profile chosen by `THALASSIC_DDS_PROFILE`. `dev` (default) restricts discovery to localhost and derives the domain from the Unix user, so neither the network nor a shared lab machine leaks graphs. `vehicle` uses subnet discovery on a fixed domain, selected system-wide on the Jetson and opted into on a laptop that is plugged into the sub. `pixi run reset-dds` clears leftover shared memory.

The middleware is pinned to `rmw_fastrtps_cpp` in both profiles so all machines behave the same; switching to CycloneDDS would be a one-line change in the profiles if the need arises.

## 5. GPU inference on the Jetson comes from JetPack

conda-forge's `linux-aarch64` PyTorch is not compiled for the Orin's `sm_87` GPU, and TensorRT's Python bindings only exist in JetPack's system interpreter. So the environment installs CPU builds everywhere, and on the Jetson `deploy/jetson/link_jetpack_python.sh` adds a `.pth` file that lets the environment import JetPack's `tensorrt` and `cuda` modules. This relies on `pixi.toml` pinning Python to the Jetson's minor version (3.12). If NVIDIA publishes Python 3.12 wheels for the vehicle's JetPack release, pin them in `pixi.toml` instead.

## 6. Dependencies kept although currently unused in launch files

The depth camera is not on the vehicle right now, so nothing launches the OAK-D driver, the RTAB-Map visual odometry pair, or `sub_color_correction`. That is a hardware gap, not abandoned work, so the declarations stay in `sub_bringup/package.xml` and the packages keep building. RoboStack does not package `rtabmap` or `depthai-ros` v3, so when the camera returns they will be added as submodules.

**What changed:** those blocks used to sit commented out in the launch files. Commented-out node definitions rot — the OAK-D block still referenced a package nobody had installed, and the RTAB-Map block was written against topics the current sim no longer publishes — and they made the two launch files hard to read for the nodes that do run. They were deleted; `git log -S` finds them, and this decision records why they will come back.

`sim_labeling` stayed, because it works: it is behind `labeling:=true` in `sim_launch.py` rather than commented out of the entity list.

Removed for real: the `--skip-keys=pcl` workaround (the `pcl` package now comes from conda-forge), the venv, the old apt-era `install.sh`, `setup.sh` / `setup.zsh`, `clean.sh`, `docker/bashrc`, and the Google Drive Spinnaker download.

## 7. Hardware identity lives in a vehicle description

**Was:** the down camera's serial number, `/dev/pico`, `/dev/naviguider_imu`, every mounting offset, and all eight thruster poses were Python literals in `marlin_v2_launch.py` and `pool_test_launch.py`. Swapping a camera meant editing a launch file; the frame id `marlin_v2/imu_link` was hardcoded in one place while the node beside it built the same string from `robot_name`.

**Now:** `sub_bringup/config/vehicles/<robot_name>.yaml` holds everything that is a property of the physical hull, and the launch files read it through `sub_bringup.vehicle`. `description_launch.py` publishes its `transforms`, `thrusters` and each device's `mount` as static transforms (one `static_transforms` node); the hardware launch file reads each device's address (serial port, video device, camera serial, IP) and frame from its `devices` entry. Node *tuning* — controller gains, EKF configuration, camera exposure — is not vehicle identity and stays in its own file next to it.

Rotations in that file are degrees, not radians: `pitch: -90` is exact and readable where `-math.pi / 2` needed the launch file to import `math`.

**Thrusters too:** the thruster poses used to be copied into the Stonefish scenario template and a compile-time table in `sub_control`. Now `sim_launch.py` renders the simulated thrusters from the description and the launch files pass the layout to `sub_control` as parameters (`Vehicle.thruster_layout()`), so all three read the one block. The copies had diverged: the allocator's had hand-edited offsets (horizontal z zeroed, a stray `+0.08` in the verticals' roll), the description's frames pointed the other way, and none recorded which thrusters have left-hand propellers, which the allocator needs to command them correctly.

## 8. Vehicle OS configuration is files in the repository

**Was:** the static address, the NTP server, and the camera interface MTU were typed into the Jetson by hand. `deploy/jetson/setup.sh` wrote the rest from heredocs, so the only way to see what the vehicle ran was to read a shell script, and the MTU had to be re-applied after every boot by a second script.

**Now:** everything installed on the vehicle is a file in `deploy/jetson/files/`, and `setup.sh` is a list of `install` commands. The network configuration is a netplan file, so the address and MTU survive a reboot; the NTP server is a chrony drop-in; the Jetson power mode is a systemd unit, because `nvpmodel -m 0` by hand does not survive a reflash.

**Why not a configuration management tool** (Ansible, salt): the fleet is one Jetson, reachable over one cable, and the failure mode that matters is a change made at a competition at 2 a.m. `install`ing eleven files is inspectable by anyone on the team without learning anything new.

**The fan is pinned at full speed** rather than handed to `nvfancontrol`. Every profile that daemon ships is a temperature curve, and a curve means the airflow starts *after* the heat does. In a sealed hull the fan is the only thing moving heat from the SoC to the wall the water cools, and there is nobody aboard to be annoyed by the noise, so there is no reason to run it slower than maximum. `nvfancontrol` is disabled rather than reconfigured, because otherwise it would wind the PWM back down.

This is not a substitute for `jetson_clocks` staying off: DVFS is still the SoC's only way to back off if the fan is not enough.

**Not done here:** the `avbotz` user is created when the Jetson is flashed, and `chrony` is installed with `apt` once, by hand, because a package install is not something a provisioning script should attempt on a vehicle with no route to the internet. `setup.sh` does run `netplan apply`, which drops an SSH session over `end0` when the address changes, so a network change is applied from the console.

## 9. One formatter per language, run from one command

**Was:** four disagreeing systems, none of which anyone ran.

- `.clang-format` in the repository root described the C++ that is actually written here: 4-space indent, 120 columns, `AllowShortLoopsOnASingleLine`. Nothing invoked it.
- Every C++ package's `CMakeLists.txt` carried an `ament_lint_auto` block, so `colcon test` ran **uncrustify** against the ROS 2 style — 2-space indent, 100 columns — which contradicts `.clang-format` line for line. Some packages also silenced `ament_copyright` and `ament_cpplint`; `sub_mission` did not.
- The Python packages declared `ament_copyright`, `ament_flake8` and `ament_pep257` as test dependencies, but none of them shipped the test files that run those linters, so nothing happened. There was no Python formatter at all.
- `pyrightconfig.json` configured a type checker that was not in the environment.

**Now:** two formatters, one per language, both reading a config in the repository root, both run by one script:

```bash
pixi run format    # rewrite
pixi run lint      # check; this is what CI should run
```

`ruff` formats and lints Python (`ruff.toml`); `clang-format` formats C and C++ (`.clang-format`, unchanged — it already matched the code). `scripts/format.sh` takes its file list from `git ls-files`, which lists submodule *directories* but not their contents, so upstream code is never touched.

The `ament_lint_auto` blocks are gone from our packages, and `colcon.meta` builds the submodules with `-DBUILD_TESTING=OFF` so their lint suites do not report against their style in our test run. `pixi run test` is now for tests.

**Why ruff over black + flake8 + isort:** one tool, one config, one pinned version, and it does in a second what the three did in twenty.

**Why not pyright in `pixi run lint`:** `pyrightconfig.json` stays for editors, which ship their own copy. Adding it as a gate would pull node into the environment on every machine including the Jetson, and `rclpy`'s stubs are thin enough that it would mostly report on ROS rather than on us.

## 10. Model-based control with a trajectory generator, not a deeper cascade or MPC

**Was:** four banks of per-axis PIDs (position → velocity → force, attitude → rate → torque) acting on raw step setpoints, with a feedforward model whose terms were all zero, and one `velocity` flag per topic switching all three translational (or rotational) axes between position and velocity control at once.

**Now:** Bumblebee's structure (NUS, RoboSub 2025 champion): a jerk-limited trajectory generator per axis, a control law that feeds forward a model of the vehicle (mass, drag, buoyancy) and closes the loop with PID in accelerations, and a QP thrust allocator whose achieved wrench feeds the integrators' anti-windup. Commands set a mode per axis. See [control.md](control.md).

**Why not LQR or MPC:** they need a better model than a student team keeps current on a hull that changes every season, and other teams who tried them went back (ARVP dropped LQR in 2023; Tartan's MPC never left simulation; Brasília dropped computed torque as too sensitive to model error). Here the model is only feedforward, so a wrong one costs performance, not stability, and an LQR is still useful to *design* the gains from an identified model. **Why not keep the cascade:** the gains in a model-based law are the same accelerations on every axis whatever the vehicle weighs, the trajectory generator gives the controller acceleration to feed forward (step setpoints have none), and saturation is handled where it happens (the allocator) rather than by per-PID output clamps that do not know the thrusters' limits.

## 11. The mission works in ENU/FLU

**Was:** the mission had a frame of its own: x forward, y right, z down (depth positive), yaw clockwise-positive. The trees, the blackboard and the C++ all used it; `commands.hpp` flipped signs on the way to `sub_control`, and `node.cpp` flipped them back for odometry and `control/error`. Everything else in the workspace, `MotionSetpoint` included, is REP-103, so a number's sign depended on which side of that boundary it was read from.

**Now:** the mission uses `sub_control`'s frames end to end: positions and yaw in `odom` (ENU: z up, so 1 m deep is z = −1; yaw counter-clockwise), relative moves and velocities along the body (FLU: forward, left, up). `commands.hpp` copies targets into the message unchanged. The only conversion left is from `sub_vision`'s camera-optical bearings, made in `vision.cpp` where it steers on them; its yaw metadata is already ROS/FLU. Every vertical value is a z: the `Transit` port `depth` became `z`, and the ports still named for depth are flags and gains. See [mission.md](mission.md#frames).

**Behavior:** unchanged. Every value in the trees and missions was converted (z, y and yaw negated, the sweep and search patterns mirrored), so the vehicle makes the same moves, with two fixes found on the way: down-camera alignment moved forward toward a target *behind* the vehicle (image down on that camera is aft, by its mount in the vehicle description), and `sub_vision`'s counter-clockwise yaw metadata was added to the clockwise mission yaw. The detection pose is no longer read as a yaw at all, as it is in the camera's optical frame.

**Why:** every sign flip at a boundary is a place for a sign error, and in a mission one turns the vehicle the wrong way into a wall. With one convention, a value means the same in a tree, a log, Foxglove and `ros2 topic echo`.

## 12. The competition-week mission is ported onto this controller

**Was:** `mission_v3`, the July 2026 competition-week branch, rewrote the gate, slalom, torpedo, bins and octagon trees and added the vision and movement nodes they need, against the old controller: a `ControlSetpoint` action and separate position, attitude and spin topics.

**Now:** its trees and nodes run here, on `MotionSetpoint` with this branch's completion checks, timeouts and measured pose, and in its frames (decision 11). The gate goes through its left opening and takes the role shown there (`SelectGateRole`), so the slalom always passes left of the red pipes and `gate_side` is gone from the pool files. `ForwardContinuousAlign` replaces `ForwardAlign`. Its tuned numbers are kept; its laws are not where they were wrong: the down-camera centering and searches moved left for a target on the right, `NavigateToTransform` applied its sideways offset the wrong way, and its sweeps had reverted to turning left first. `Spin` now hands over to a heading target before the end, where `mission_v3` moved the spin into its controller.

**Not ported:** `StationarySweepAlign` (no tree used it, and it needs a `pole_count` no post-processor sends), the `torp_board_type` parameter and `TorpBoardV1` (no tree read them), `ignore_initial_kill` (the simulated kill switch is latched), `NavigateToTransform`'s hard-coded fallback offsets (the vehicle description has the frames), and ports that every tree left at their default. Bins is not in the competition run, as on `mission_v3`.

## 13. One node publishes the vehicle's static transforms

**Was:** `description_launch.py` started a `static_transform_publisher` process per transform in the vehicle description, twenty of them.

**Now:** one `sub_bringup` `static_transforms` node publishes them all in one latched `/tf_static` message.

**Why:** each process is a DDS participant, and with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` Fast DDS announces a participant only to the first 32 on the machine. The sim had reached 33 processes with the annotators, so two late participants could miss each other: runs where the mission never heard `sub_vision` and searched forever, intermittently. The stack is now 14 processes.
