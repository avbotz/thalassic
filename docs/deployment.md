# Deployment (Jetson AGX Orin)

The vehicle runs the same `pixi` environment as developers, on bare metal. Nothing starts automatically; the stack is launched by hand.

## One-time provisioning

As the vehicle user (`avbotz`, created when the Jetson is flashed):

```bash
git clone --recurse-submodules https://github.com/avbotz/thalassic.git ~/thalassic
cd ~/thalassic
scripts/install.sh                            # pixi + environment + build
sudo apt install chrony                       # only NTP server, needs a network once
sudo deploy/jetson/setup.sh                   # OS configuration, see below
deploy/jetson/link_jetpack_python.sh          # TensorRT/CUDA bindings from JetPack
```

Then reboot, so the new groups, the DDS profile, and the network configuration all apply.

### What `deploy/jetson/setup.sh` does

It installs the files in `deploy/jetson/files/` and the udev rules in `deploy/udev/`, and it is idempotent — re-run it after reflashing or after editing anything in either directory. Every setting it applies survives a reboot. [`deploy/jetson/README.md`](../deploy/jetson/README.md) is the table of what lands where and why; the short version:

| Setting | Why it is not just typed in |
|---|---|
| Static `192.168.5.245/24` and MTU 9000 on `end0` (netplan) | The DVL and the camera are pointed at that address, and GigE Vision needs jumbo frames. Both used to be re-applied by hand after every boot |
| `chrony` serving `192.168.5.0/24` | The DVL and the down camera take their time from the Jetson, and `systemd-timesyncd` cannot serve |
| MAXN power mode (systemd unit) | JetPack boots power-capped; `nvpmodel -m 0` by hand does not survive a reflash |
| Fan at full speed (systemd unit) | The hull is sealed; the fan is what carries heat to the wall the water cools. `nvfancontrol` waits for heat before it spins up, so it is disabled |
| 10 MB socket buffers, `usbfs_memory_mb=1000` | Camera buffering; the Jetson boots with extlinux, so the usbfs limit cannot go on the kernel command line |
| udev rules, `flirimaging`/`dialout`/`video` groups | Stable device names and non-root access |
| `THALASSIC_DDS_PROFILE=vehicle` | Subnet DDS discovery on the vehicle domain for every login shell |

Two things it deliberately does not do on its own:

- **`netplan apply`** drops any SSH session running over `end0`, so the script installs and validates the file and leaves the decision to whoever is at the console. Pass `--apply-network`, or reboot.
- **`apt install chrony`** — on a vehicle with no route to the internet that is a failure at the worst possible moment. The script installs the chrony config, says clearly if chrony is missing, and carries on.

The FLIR-related steps come from the `spinnaker_camera_driver` README for machines *without* the Spinnaker SDK installed. The driver builds against an SDK it downloads itself, so no SDK package is installed on the Jetson.

### GenTL producer

The GigE camera is opened through a GenTL producer shipped with the source-built driver. Because the workspace uses `--merge-install`, it ends up at `install/lib/spinnaker-gentl/Spinnaker_GenTL.cti`, and `scripts/setup.sh` exports `SPINNAKER_GENTL64_CTI` to that path whenever the file exists.

### JetPack Python bridge

`deploy/jetson/link_jetpack_python.sh` writes a `.pth` file into the environment's `site-packages` that appends `/usr/lib/python3/dist-packages` to `sys.path`. Environment packages take precedence; only modules the environment lacks (`tensorrt`, `cuda`) resolve to JetPack's copies. The script refuses to run if the two interpreters differ in minor version, and prints whether both modules import.

## Running

```bash
cd ~/thalassic
pixi run pool                                   # hardware stack
pixi run pool mission:=pool_a                   # with a mission
pixi shell                                      # ros2 topic echo ... etc.
```

## Updating the vehicle

```bash
cd ~/thalassic
git pull --recurse-submodules
pixi install --frozen        # only changes anything if pixi.lock changed
pixi run build
```

`pixi install --frozen` never re-solves; if `pixi.lock` and `pixi.toml` disagree it stops, which is what you want on a competition day.

If the pull touched `deploy/`, re-run `sudo deploy/jetson/setup.sh`.

## Network

See [networking.md](networking.md) for addresses. The Jetson is `192.168.5.245`, the DVL `192.168.5.250`. A laptop plugged into the switch gets a DHCP lease and reaches the ROS graph with:

```bash
THALASSIC_DDS_PROFILE=vehicle pixi shell
```

## Checking a provisioned vehicle

[`deploy/jetson/README.md`](../deploy/jetson/README.md#after-a-change) lists the command that shows whether each setting took.

## Open items

- x86 CUDA builds for developer machines with NVIDIA GPUs.
- Model weights are unversioned (`weights/` is gitignored and `sub_vision` reads `/home/avbotz/sub_vision/models`); a manifest with checksums is the next step.
- The down camera still takes its address from DHCP; it should be static like the DVL.
