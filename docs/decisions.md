# Decisions

Short records of the decisions in this repository.

## 1. pixi + RoboStack instead of apt ROS, rosdep, and a venv

**Was:** `install.sh` added the ROS apt repository at whatever version was
newest that day, installed `ros-jazzy-desktop`, built Stonefish into
`/usr/local` with `sudo`, downloaded the Spinnaker SDK from a Google Drive
link, created a `--system-site-packages` venv, and pip-installed three
packages that were listed nowhere. The Dockerfile repeated a different subset
of that by hand. Two machines set up a week apart had different simulators,
different `torch` builds, and different ROS patch versions.

**Now:** `pixi.toml` declares every dependency (ROS 2 from the
`robostack-jazzy` channel, compilers, PCL, SDL2, PyTorch, and so on) and
`pixi.lock` pins exact builds for `linux-64` and `linux-aarch64`. `pixi install --frozen` reproduces the environment bit for bit,
without `sudo`, on any Linux distribution, the Jetson, or a Mac.

**Why pixi:**

- One lockfile covers conda and PyPI packages, native libraries, and the
  compiler, which `requirements.txt` + `rosdep` never could.
- Nothing global. Different checkouts (branches, older releases) coexist
  with different dependency sets.
- RoboStack itself recommends pixi, and packaged every ROS package we use
  except `rtabmap` and the Spinnaker driver.
- The same manifest serves developers, the container, CI, and the vehicle,
  so "works on my machine" differences shrink to hardware.

**Why not nix:** the reproducibility is higher, but it is much more difficult to configure, especially in regards to the Jetson where we would probably need to use NixOS.

**Cost:** the Jetson's GPU stack (TensorRT, CUDA bindings) is not in the
lockfile because NVIDIA only ships Orin builds through JetPack. See
[Decision 5](#5-gpu-inference-on-the-jetson-comes-from-jetpack).

## 2. Stonefish is built inside the workspace

**Was:** a global `/usr/local` build from a branch tip, installed with `sudo`
by `install.sh` or baked into the Docker image. Trying another Stonefish
version meant overwriting every machine's global install, and `stonefish_ros2`
demands an exact library version, so mismatches failed at configure time.

**Now:** `src/sub_sim/stonefish_vendor` is an `ament_vendor` package that
builds the git submodule `stonefish_vendor/stonefish` (AVBotz fork, pinned by
commit) and installs it under `install/opt/stonefish_vendor`. An ament
environment hook makes it visible to `stonefish_ros2`. Switching versions is
`git checkout` in the submodule plus a rebuild of two packages, and the
submodule pointer in git records exactly which simulator every commit was
tested with.

`colcon.meta` declares that `stonefish_ros2` depends on `stonefish_vendor`,
because that submodule's own `package.xml` cannot know about our vendor
package and colcon needs the edge to order the build. The same file carries
the one build flag a submodule needs under the conda toolchain:
`libwaterlinked` uses `std::thread` without linking pthread, which passes on
hosts whose glibc merged libpthread but not against conda-forge's glibc 2.28
sysroot, so it gets `-pthread` there. The FLIR driver gets two more: an
implicit `<cstdint>` include (its headers rely on a transitive include that
GCC 15 dropped, and the package builds with `-Werror`), and
`--allow-shlib-undefined`, because the prebuilt Spinnaker library it downloads
references glibc 2.34+ symbols that the conda sysroot cannot verify at link
time; they resolve against the host's glibc at runtime, which is what happens
on any apt-based install too. Fixes for submodules go in `colcon.meta` first
and upstream second, so the workspace never depends on a patched checkout.

## 3. Submodules stay for packages RoboStack does not ship

`stonefish_ros2`, `waterlinked_dvl`, and `flir_camera_driver` remain git
submodules. The FLIR driver downloads the Spinnaker SDK during its own build
(from a mirror the upstream driver maintains), so the SDK is no longer
installed system-wide and the Google Drive step is gone. The pieces the SDK
needs on the vehicle (a `flirimaging` group, udev rules, the usbfs limit, and
the GenTL producer path) are applied by `deploy/jetson/setup.sh` and the
activation script.

## 4. DDS discovery is limited per developer

**Was:** default Fast DDS settings. Anyone running the simulator on the same
network as another developer saw both `/marlin_v2` graphs, and stale
shared-memory segments after crashes produced the `rmw_create_node` error the
old README worked around with `pkill -f ros`.

**Now:** `scripts/setup.sh` applies a profile chosen by
`THALASSIC_DDS_PROFILE`. `dev` (default) restricts discovery to localhost and
derives the domain from the Unix user, so neither the network nor a shared
lab machine leaks graphs. `vehicle` uses subnet discovery on a fixed domain,
selected system-wide on the Jetson and opted into on a laptop that is plugged
into the sub. `pixi run reset-dds` clears leftover shared memory.

The middleware is pinned to `rmw_fastrtps_cpp` in both profiles so all
machines behave the same; switching to CycloneDDS would be a one-line change
in the profiles if the need arises.

## 5. GPU inference on the Jetson comes from JetPack

conda-forge's `linux-aarch64` PyTorch is not compiled for the Orin's `sm_87`
GPU, and TensorRT's Python bindings only exist in JetPack's system
interpreter. So the environment installs CPU builds everywhere, and on the
Jetson `deploy/jetson/link_jetpack_python.sh` adds a `.pth` file that lets the
environment import JetPack's `tensorrt` and `cuda` modules. This relies on
`pixi.toml` pinning Python to the Jetson's minor version (3.12). If NVIDIA
publishes Python 3.12 wheels for the vehicle's JetPack release, the commented
block in `pixi.toml` shows where to pin them instead.

## 6. Dependencies kept although currently unused in launch files

`rtabmap_odom`, `rtabmap_sync` (depth-camera visual odometry) and the OAK-D
driver block are commented out in the launch files because the depth camera
is not on the vehicle right now, not because the work was abandoned. The
declarations stay in `sub_bringup/package.xml`. RoboStack does not package
`rtabmap` or `depthai-ros` v3, so when the camera returns they will be added
as submodules. `sub_color_correction` (which needs the depth camera) and
`sim_labeling` remain built for the same reason.

Removed for real: the `--skip-keys=pcl` workaround (the `pcl` package now
comes from conda-forge), the venv, the old apt-era `install.sh`, `setup.sh` /
`setup.zsh`, `clean.sh`, `docker/bashrc`, and the Google Drive Spinnaker
download.
