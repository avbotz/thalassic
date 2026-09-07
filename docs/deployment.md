# Deployment (Jetson AGX Orin)

The vehicle runs the same `pixi` environment as developers, on bare metal.
Nothing starts automatically; the stack is launched by hand.

## One-time provisioning

As the vehicle user (`avbotz`):

```bash
git clone --recurse-submodules https://github.com/avbotz/thalassic.git ~/thalassic
cd ~/thalassic
scripts/install.sh                          # pixi + environment + build
sudo deploy/jetson/setup.sh                   # OS configuration, see below
deploy/jetson/link_jetpack_python.sh          # TensorRT/CUDA bindings from JetPack
```

Log out and back in (or reboot) afterwards so the new groups and the DDS
profile apply.

### What `deploy/jetson/setup.sh` does

Idempotent; re-run after reflashing or after editing `udev/`.

| Step | Where it lands | Why |
|---|---|---|
| `flirimaging` group, add user to it plus `dialout`, `video` | `/etc/group` | The FLIR driver's SDK expects camera device nodes owned by this group |
| udev rules from `udev/` | `/etc/udev/rules.d/` | Stable device names (`/dev/pico`, `/dev/naviguider_imu`, `/dev/front_camera`), FLIR USB access |
| `usbfs_memory_mb = 1000` | `/etc/tmpfiles.d/thalassic-usbfs.conf` | USB3 Vision cameras need large USB buffers. The Jetson boots with extlinux, not GRUB, so the value is applied at boot by systemd-tmpfiles rather than a kernel cmdline edit |
| socket buffers 10 MB | `/etc/sysctl.d/60-thalassic-gige.conf` | GigE Vision down camera at high packet rates |
| `THALASSIC_DDS_PROFILE=vehicle` | `/etc/profile.d/thalassic.sh` | Subnet discovery on the vehicle domain for every login shell |

The FLIR-related steps come from the `spinnaker_camera_driver` README for
machines *without* the Spinnaker SDK installed. The driver builds against an
SDK it downloads itself, so no SDK package is installed on the Jetson.

Still manual, per boot: jumbo frames on the camera interface,
`sudo deploy/jetson/gige-camera-runtime.sh end0`, until the interface is
declared in netplan.

### GenTL producer

The GigE camera is opened through a GenTL producer shipped with the
source-built driver. Because the workspace uses `--merge-install`, it ends up
at `install/lib/spinnaker-gentl/Spinnaker_GenTL.cti`, and
`scripts/setup.sh` exports `SPINNAKER_GENTL64_CTI` to that path whenever
the file exists.

### JetPack Python bridge

`deploy/jetson/link_jetpack_python.sh` writes a `.pth` file into the
environment's `site-packages` that appends `/usr/lib/python3/dist-packages`
to `sys.path`. Environment packages take precedence; only modules the
environment lacks (`tensorrt`, `cuda`) resolve to JetPack's copies. The script
refuses to run if the two interpreters differ in minor version, and prints
whether both modules import.

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

`pixi install --frozen` never re-solves; if `pixi.lock` and `pixi.toml`
disagree it stops, which is what you want on a competition day.

## Network

See [networking.md](networking.md) for addresses. The Jetson is `192.168.0.245`,
the DVL `192.168.0.250`. A laptop plugged into the switch gets a DHCP lease
and reaches the ROS graph with:

```bash
THALASSIC_DDS_PROFILE=vehicle pixi shell
```

## Open items

- The NTP server the Jetson provides to the DVL and camera, the static
  address, and the camera interface MTU are still configured by hand on the
  device. They belong in `deploy/jetson` (chrony and netplan files) next.
- x86 CUDA builds for developer machines with NVIDIA GPUs.
- Model weights are unversioned (`weights/` is gitignored and `sub_vision`
  reads `/home/avbotz/sub_vision/models`); a manifest with checksums is the
  next step.
