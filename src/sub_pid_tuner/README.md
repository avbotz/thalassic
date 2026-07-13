# AVBotz Dashboard

A focused, FTC Dashboard-style browser interface for the running ROS 2 stack. It discovers active nodes and their declared parameters, stages and applies runtime configuration changes with ROS readback, shows core control telemetry, publishes tuning setpoints, and graphs PID target/current/error together.

It intentionally does not own the controller, run automatic trials, or change the low-level firmware. Runtime Apply changes the running ROS node immediately; Save profile separately persists verified controller values to the launch YAML.

## Panels

- **Configuration:** select any discovered ROS node, search its parameters, edit writable values, and apply one or many changes. PID arrays on `sub_control` are shown as Kp/Ki/Kd/Limit. `Ctrl+S` applies staged values to the running nodes and verifies them by readback. `Ctrl+Shift+S` applies them and atomically saves the verified controller values to the sim or pool profile.
- **Graph:** select live signals or use the target/current/error PID preset, choose an automatic, fixed, or custom Y range, zoom, pause, clear, resize the time window, and export CSV. Live graph data is streamed at 30 Hz.
- **Telemetry:** searchable latest values for controller errors, state, setpoints, thrusters, altitude, and kill state.
- **Setpoint:** publish position, velocity, attitude, or angular-rate targets to the existing controller topics.

Tiles can be dragged and resized. Layout and graph selections are stored in the browser.

## Build

```bash
cd ~/dev/AVBotz/thalassic
source /opt/ros/jazzy/setup.zsh
rosdep install --from-paths src --ignore-src -r -y
colcon build --merge-install --symlink-install --packages-up-to sub_pid_tuner sub_bringup
source install/setup.zsh
```

## Run

Simulation:

```bash
ros2 launch sub_bringup sim_launch.py dashboard:=true
```

Pool/hardware:

```bash
ros2 launch sub_bringup pool_test_launch.py dashboard:=true
```

Then open http://127.0.0.1:8080. To access it from another computer on the pool network, add `dashboard_host:=0.0.0.0` and open `http://ROBOT_IP:8080`.

The dashboard can also attach to an already-running stack:

```bash
ros2 run sub_pid_tuner dashboard --robot-name marlin_v2 --profile install/share/sub_bringup/config/control_gains_sim.yaml
```


## Saving configuration

- Typing changes only stages the value; yellow rows have unapplied edits.
- A row's **Apply** button changes that parameter on the running node.
- **Apply runtime** or `Ctrl+S` sends all staged edits. Parameters for each ROS node are sent atomically, and the dashboard clears the yellow state only after ROS readback matches.
- **Save profile** or `Ctrl+Shift+S` first performs the same runtime update, then atomically replaces the configured YAML profile. A timestamped backup is stored under `~/.local/state/avbotz/dashboard/backups`.
- **Reload** discards staged browser edits and refreshes values from ROS and disk.
