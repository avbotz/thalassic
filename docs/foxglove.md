# Foxglove

Driving, tuning and watching the vehicle from a laptop is done in [Foxglove](https://foxglove.dev), over the `foxglove_bridge` that every launch file starts. Open a *Foxglove WebSocket* connection to `ws://localhost:8765` for the simulator, or `ws://192.168.5.245:8765` for the vehicle ([networking.md](networking.md)); the `foxglove_port` launch argument changes the port ([setup.md](setup.md#launch-arguments)).

## Extensions

The panels that drive and tune `sub_control` and show `sub_vision`'s detections are the three extensions in [avbotz/foxglove-extensions](https://github.com/avbotz/foxglove-extensions), checked out beside this repository as `../foxglove-extensions`: teleop (6-DOF Teleop, Motion Setpoint), tuning (Controller Gains, Tuning Routines, Controller Signals) and vision (Detections). `pnpm run local-install` there installs them in the desktop app; their READMEs explain the panels.

## Layouts

`src/sub_bringup/foxglove/` has a layout for each job, for the simulator and for the vehicle. Import one with *Layouts → Import from file*; it becomes a personal layout named after the file.

| Layout | For | Shows |
|---|---|---|
| `teleop` | Driving by hand | 6-DOF Teleop; both cameras, every frame, with the detections drawn; Motion Setpoint, diagnostics and the log in tabs |
| `tuning` | [Tuning `sub_control`](control.md#tuning) | Controller Gains and Tuning Routines; a tab per axis with Controller Signals graphs (target / reference / measured, tracking error, wrench / disturbance); the thruster commands; diagnostics and the log |
| `autonomy` | Watching a mission run | Arming, kill, failsafe and sensor state; both cameras with the detections, on the frames the detector processed; depth and heading graphs; a top-down map in odom; the position error and each axis's mode over time; the log, diagnostics, Motion Setpoint (to take over: *Hold* stops the vehicle) and the raw detections in tabs |

The `_sim` and `_vehicle` files differ in what the cameras publish (the simulator `image_color`, the vehicle `image_raw`; the layouts show each camera's JPEG copy), and the simulator's add **KILL** and **RELEASE** buttons for its kill switch (`sim/kill_switch`) and the robot model on the map. The vehicle's down camera sends a Bayer mosaic, which its JPEG copy stores as a grey image; for colour, add an Image panel on `down_camera/image_raw`.

### Changing them

`src/sub_bringup/sub_bringup/foxglove_layouts.py` generates the files, so the two variants stay in step: `pixi run foxglove-layouts` rewrites them, and `test_layouts.py` fails until they are current. To change a layout, change the generator and regenerate. The settings it writes are the extension panels' saved state and the built-in panels' settings; the quickest way to find one is to make the change in Foxglove, export the layout and copy the value over.

Foxglove names an extension's panels `<display name>.<panel name>` when the extension is installed from its `.foxe` file, which is what the layouts use (`AVBotz Teleop.6-DOF Teleop`). An extension published to an organization is named `org:<publisher>:<name>` instead; change the constants at the top of the generator for that.
