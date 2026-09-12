udev rules for stable device names and permissions on the vehicle.

| Rule | Device |
|---|---|
| `99-raspberrypi-pico.rules` | maritime firmware (RP2040) as `/dev/pico` |
| `99-naviguider-imu.rules` | NaviGuider IMU as `/dev/naviguider_imu` |
| `99-logitech-front-camera.rules` | Logitech C922 as `/dev/front_camera` |
| `99-arduino-atmega-2560.rules` | legacy Nautical MCU as `/dev/arduino_mega` |
| `40-flir-spinnaker.rules` | FLIR USB3 cameras readable by the `flirimaging` group |

`sudo deploy/jetson/setup.sh` installs all of them and reloads udev. To apply a rule change by hand: copy the file to `/etc/udev/rules.d/` and run `deploy/udev/reload.sh`.
