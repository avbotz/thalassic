# Vehicle OS configuration

Everything the Jetson AGX Orin needs that is not a ROS package. `setup.sh` installs it all and is safe to re-run; the files it installs live in `files/` so the vehicle's configuration is reviewable in a diff instead of being remembered by whoever last set the sub up.

```bash
sudo deploy/jetson/setup.sh                  # install everything
```

Every setting below survives a reboot.

| What | Installed as | Why |
|---|---|---|
| Static address `192.168.5.245/24` and MTU 9000 on `end0` | `/etc/netplan/60-thalassic-vehicle.yaml` | The DVL and the GigE camera are pointed at a fixed address; jumbo frames carry GigE Vision at full frame rate. Both used to be set by hand after every boot |
| NTP server for `192.168.5.0/24` | `/etc/chrony/conf.d/thalassic.conf` | The DVL and the down camera take their time from the Jetson. `systemd-timesyncd` cannot serve time, so this needs `chrony` |
| Jetson power mode MAXN | `/etc/systemd/system/thalassic-power-mode.service` | JetPack boots into a power-capped mode; MAXN is what the inference budget assumes. `nvpmodel -m 0` by hand does not survive a reflash |
| Fan pinned at full speed | `/etc/systemd/system/thalassic-fan.service` + `/usr/local/sbin/thalassic-fan-max` | The hull is sealed, so the fan is what moves heat from the SoC to the wall the water cools. `nvfancontrol` only spins up once something is already hot, and is disabled |
| Socket buffers, 10 MB | `/etc/sysctl.d/60-thalassic-gige.conf` | The GigE camera bursts a frame faster than the default receive buffer holds |
| `usbfs_memory_mb = 1000` | `/etc/tmpfiles.d/thalassic-usbfs.conf` | USB3 Vision cameras need large USB buffers. The Jetson boots with extlinux, not GRUB, so the value is written at boot rather than passed on the kernel command line |
| udev rules from `../udev` | `/etc/udev/rules.d/` | Stable device names (`/dev/pico`, `/dev/naviguider_imu`, `/dev/front_camera`) and FLIR USB access |
| `flirimaging` group, user added to it plus `dialout` and `video` | `/etc/group` | Otherwise the cameras and the serial MCUs are root-only |
| `THALASSIC_DDS_PROFILE=vehicle` | `/etc/profile.d/thalassic.sh` | Subnet DDS discovery on the vehicle domain for every login shell |

## After a change

Editing anything in `files/` or `../udev` means re-running `setup.sh` on the
vehicle. Check what actually took:

```bash
ip -br addr show end0        # 192.168.5.245/24
ip link show end0            # mtu 9000
chronyc clients              # the DVL and the camera, once they have asked
nvpmodel -q                  # NV Power Mode: MAXN
cat /sys/class/hwmon/hwmon*/pwm1   # 255 on the fan's hwmon
systemctl is-enabled nvfancontrol  # disabled
sysctl net.core.rmem_max     # 10485760
ls /dev/pico /dev/naviguider_imu /dev/front_camera   # udev rules
systemctl status thalassic-power-mode.service thalassic-fan.service
```

## Also here

`link_jetpack_python.sh` exposes JetPack's `tensorrt` and `cuda` Python modules to the pixi environment. It runs as the vehicle user after `pixi install`, not as root. See [Decision 5](../../docs/decisions.md).
