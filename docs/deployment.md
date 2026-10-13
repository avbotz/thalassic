# Deployment (Jetson AGX Orin)

The vehicle runs the same `pixi` environment as developers, on bare metal. The stack is launched by hand, or at boot by the `thalassic` service once it is enabled (see [Starting at boot](#starting-at-boot)).

## One-time provisioning

As the vehicle user (`avbotz`, created when the Jetson is flashed):

```bash
git clone --recurse-submodules https://github.com/avbotz/thalassic.git ~/thalassic
cd ~/thalassic
scripts/install.sh                            # pixi + environment (TensorRT and CUDA included) + build
sudo apt install chrony                       # only NTP server, needs a network once
sudo deploy/jetson/setup.sh                   # OS configuration, see below
pixi run build-engines                        # TensorRT engines from weights/*.onnx, minutes each, checked
```

Then reboot, so the new groups and the DDS profile apply.

### What `deploy/jetson/setup.sh` does

It installs the files in `deploy/jetson/files/` and the udev rules in `deploy/udev/`, and it is idempotent — re-run it after reflashing or after editing anything in either directory. Every setting it applies survives a reboot. [`deploy/jetson/README.md`](../deploy/jetson/README.md) is the table of what lands where and why; the short version:

| Setting | Why it is not just typed in |
|---|---|
| Static `192.168.5.245/24` and MTU 9000 on `end0` (netplan) | The DVL and the camera are pointed at that address, and GigE Vision needs jumbo frames. Both used to be re-applied by hand after every boot |
| `chrony` serving `192.168.5.0/24` | The DVL and the down camera take their time from the Jetson, and `systemd-timesyncd` cannot serve |
| MAXN power mode (systemd unit) | JetPack boots power-capped; `nvpmodel -m 0` by hand does not survive a reflash |
| Fan at full speed (systemd unit) | The hull is sealed; the fan is what carries heat to the wall the water cools. `nvfancontrol` waits for heat before it spins up, so it is disabled |
| 10 MB socket buffers | The GigE camera bursts a frame faster than the default receive buffer holds |
| udev rules, `flirimaging`/`dialout`/`video` groups | Stable device names and non-root access |
| `THALASSIC_DDS_PROFILE=vehicle` | Subnet DDS discovery on the vehicle domain for every login shell |

Two things to know:

- **`netplan apply`** runs every time, after `netplan generate` has validated the file (a rejected file is removed again, so the vehicle keeps its network). It drops any SSH session over `end0` if the address changes, so run the script from the console after changing the network file.
- **`apt install chrony`** is deliberately not done: on a vehicle with no route to the internet that is a failure at the worst possible moment. If chrony is missing the script skips the NTP server, says so, and carries on; install it and re-run the script.

The FLIR-related steps come from the `spinnaker_camera_driver` README for machines *without* the Spinnaker SDK installed. The driver builds against an SDK it downloads itself, so no SDK package is installed on the Jetson.

### GenTL producer

The GigE camera is opened through a GenTL producer shipped with the source-built driver. Because the workspace uses `--merge-install`, it ends up at `install/lib/spinnaker-gentl/Spinnaker_GenTL.cti`, and `scripts/setup.sh` exports `SPINNAKER_GENTL64_CTI` to that path whenever the file exists.

### The GPU stack

JetPack 7.2 provides the GPU driver (CUDA 13.2) and nothing else the stack uses. TensorRT (10.16.2, JetPack 7.2.1's version, built for the Orin), the CUDA 13.2 runtime libraries and `cuda-bindings` are `linux-aarch64` dependencies in `pixi.toml`, about 3 GB, so `pixi install --frozen` is the whole GPU install and only a `pixi.lock` change can change them ([Decision 5](decisions.md#5-the-jetsons-gpu-stack-comes-from-pixi-not-jetpack)). `pixi.toml` declares the Jetson platform with its driver (CUDA 13) and compute capability (8.7), and pixi checks both against the host when it installs, so a JetPack upgrade to a newer CUDA major version means updating that entry. If `pixi install` reports that the host has no `__cuda`, the driver is not visible to pixi: `CONDA_OVERRIDE_CUDA=13.2 pixi install --frozen` declares it by hand.

`pixi run build-engines` builds an engine for each ONNX model that has none, then runs each new one against ONNX Runtime on the CPU and fails on any output that disagrees beyond FP16 ([sub_vision README](../src/sub_vision/README.md#model-loading--device-selection)). `pixi run check-engines` repeats that for every engine; with `--bag bags/<run>` it uses frames from a recording instead of synthetic ones, which is what makes the box comparison mean something.

## Running

```bash
cd ~/thalassic
pixi run pool                                   # hardware stack
pixi run pool mission:=pool_a                   # with a mission
pixi shell                                      # ros2 topic echo ... etc.
```

Nothing drives until the kill switch has been engaged and released *after* the stack started: if it starts with the switch released, pull the switch and release it again. `sub_control` logs what it is waiting for, and `control/armed` says when it is armed ([control.md](control.md#safety)).

### Pool-day checklist

1. On the bench, with the stack running and the vehicle killed: `pixi run preflight`. It listens for 5 s and fails on any sensor or controller topic below its rate, a missing kill switch state, an uncalibrated camera, anything in error or stale on `/diagnostics_agg`, or a missing model or TensorRT engine. Fix what it reports.
1. Tag what is running: `git tag pool-$(date +%F) && git push origin pool-$(date +%F)` (bags record `git describe`, so each bag then names its tag).
1. In the water, killed: check `control/status` in Foxglove: `dvl_ok` once the DVL has bottom lock, `depth_ok`, `imu_ok`.
1. Release the kill switch, watch `armed` come up, and run.

### Starting at boot

`deploy/jetson/setup.sh` installs a `thalassic` systemd service that runs `pixi run --frozen pool` as `avbotz`, with launch arguments from `/etc/thalassic/pool.env` (e.g. `POOL_ARGS=mission:=competition record:=true`), and restarts it if it crashes. It is not enabled by default:

```bash
sudo systemctl enable --now thalassic    # start now and at every boot
journalctl -u thalassic -f               # its output
sudo systemctl disable --now thalassic   # back to launching by hand
```

With it, a brownout in the water leaves the stack coming back up on its own, waiting for the kill switch to be cycled, instead of a vehicle without software until someone connects. Do not also run `pixi run pool` by hand while it is enabled: two stacks on one vehicle fight over the serial ports.

### maritime

The thruster board runs [maritime](https://github.com/avbotz/maritime). `sub_low` speaks both its protocols, but only firmware with protocol 2 has the command watchdog, the hardware watchdog and the heartbeat: `/diagnostics_agg` warns "no heartbeat from the board" until the board is flashed with it. Build and flash it as maritime's README says (`west build -b rpi_pico`, then `west flash -r uf2`, or copy `build/zephyr/zephyr.uf2` to the board in BOOTSEL mode).

On the vehicle a mission runs under `sub_mission`'s restart supervisor ([mission.md](mission.md#restart-supervisor)): pulling the kill switch stops it, and releasing it starts it again from the beginning, so a botched run is rerun by carrying the vehicle back to the start. `restart:=false` runs it once, as the simulator does.

`pixi run pool record:=true` records the run into `~/thalassic/bags/` ([recording.md](recording.md)), which grows until it is cleared. To copy the bags to a laptop on the vehicle network:

```bash
rsync -av avbotz@192.168.5.245:thalassic/bags/ bags/
```

## Updating the vehicle

```bash
cd ~/thalassic
git pull --recurse-submodules
pixi install --frozen        # only changes anything if pixi.lock changed
pixi run --frozen build
pixi run --frozen fetch-weights   # downloads only models whose manifest entry changed
pixi run --frozen build-engines   # engines only for models that changed, each checked
```

`sub_vision` refuses to load a model on the vehicle until its engine is built, rather than run it on the CPU. Engines are tied to the TensorRT version, which `pixi.lock` pins, so a lockfile update that changes TensorRT needs `build-engines` too; the old engines are ignored, not loaded.

`--frozen` installs `pixi.lock` as it is: it never re-solves or rewrites the lockfile, even if `pixi.toml` has changed without it, which is what you want on a competition day.

If the pull touched `deploy/`, re-run `sudo deploy/jetson/setup.sh`.

### Rolling back

Every pool build is tagged (see the checklist), so going back is checking out the tag that last worked:

```bash
git checkout pool-2026-10-10 && git submodule update --init --recursive
pixi install --frozen && pixi run --frozen build && pixi run --frozen fetch-weights
```

This needs no network if the tag's packages are still in pixi's cache (`~/.cache/rattler`) and its models in `weights/` (`fetch-weights` keeps the replaced file as `.old`). A clean build of the FLIR driver downloads the Spinnaker SDK, so do not `pixi run clean` on the vehicle away from a network.

## Network

See [networking.md](networking.md) for addresses. The Jetson is `192.168.5.245`, the DVL `192.168.5.250`. A laptop plugged into the switch gets a DHCP lease and reaches the ROS graph with:

```bash
THALASSIC_DDS_PROFILE=vehicle pixi shell
```

## Checking a provisioned vehicle

[`deploy/jetson/README.md`](../deploy/jetson/README.md#after-a-change) lists the command that shows whether each setting took.

## Open items

- The down camera still takes its address from DHCP; it should be static like the DVL.
- Camera calibrations are not in the repository: neither camera config sets a `camera_info_url`, so on a vehicle without `~/.ros/camera_info/` files every bearing is 0. `sub_vision` reports it on `/diagnostics` and `preflight` fails on it; calibrate both cameras underwater (`camera_calibration`) and commit the files under `sub_bringup/config/devices/`.
- An offline bundle (a packed environment and built `install/`) for updating the vehicle without the internet.
