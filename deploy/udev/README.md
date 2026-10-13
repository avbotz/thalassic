udev rules for stable device names and permissions on the vehicle.

| Rule | Device |
|---|---|
| `99-maritime.rules` | maritime firmware as `/dev/maritime` |
| `99-naviguider-imu.rules` | NaviGuider IMU as `/dev/naviguider_imu` |
| `99-logitech-front-camera.rules` | Logitech C922 as `/dev/front_camera` |

`sudo deploy/jetson/setup.sh` installs all of them and reloads udev. To apply a rule change by hand, copy the file to `/etc/udev/rules.d/` and run `deploy/udev/reload.sh`.
