# AVBotz Dashboard

A focused, FTC Dashboard-style browser interface for the running ROS 2 stack. It discovers active nodes and their declared parameters, stages and applies runtime configuration changes with ROS readback, shows core control telemetry, publishes tuning setpoints, and graphs PID target/current/error together.

For simulator PID tuning, start with the short, click-by-click
[TUNING.md](TUNING.md). For real-water feedforward tuning, use
[POOL_FF_QUICKSTART.md](POOL_FF_QUICKSTART.md).

It does not replace the controller or change the low-level firmware. Runtime Apply changes the running ROS node immediately; Save profile separately persists verified controller values to the launch YAML. A bounded tracking routine can publish repeatable controller references for manual tuning. The shared `sub_control_ff` controller exposes its reference, feedforward, feedback, commanded, allocated, and residual wrench signals here.

## Panels

- **Configuration:** select any discovered ROS node, search its parameters, edit writable values, and apply one or many changes. PID arrays on `sub_control` are shown as Kp/Ki/Kd/Limit. `Ctrl+S` applies staged values to the running nodes and verifies them by readback. `Ctrl+Shift+S` applies them and atomically saves the verified controller values to the sim or pool profile.
- **Graph:** select live signals or use the target/current/error PID preset, choose an automatic, fixed, or custom Y range, zoom, pause, clear, resize the time window, and export CSV. Live graph data is streamed at 30 Hz.
- **Telemetry:** searchable latest values for controller errors, state, setpoints, thrusters, altitude, and kill state.
- **Control:** run a guarded empirical feedforward calibration with steady bidirectional speed levels, or choose an inner-loop trapezoid cruise, minimum-jerk move, separated step/settle test, windowed sine, pose-hold disturbance test, Follower PID loop, Square Test, or Spline Test. Feedforward results remain review-only until explicitly applied and never write YAML automatically. Continuous paths use conservative AUV-scale speeds and a response-aware target clock. Square Test holds each corner and requires an operator press of **Next corner** after the vehicle arrives and settles. The graph automatically switches to useful traces for the selected op mode. See [TUNING.md](TUNING.md) for the short workflow. The separate manual setpoint row remains available for normal one-shot commands.
- **Pool Path:** green target and blue actual position trails in selectable XY/XZ/YZ planes. The vehicle marker shows yaw, pitch, or roll for the displayed plane. Path op modes are anchored to the measured start pose.

The persistent **Stop all** button in the header (or `Esc`) stops tracking and
feedforward-identification routines and clears dashboard motion commands. With
fresh telemetry on an armed vehicle it holds the measured position and
attitude; otherwise it commands zero linear and angular rate.

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
ros2 launch sub_bringup sim_launch.py controller:=feedforward dashboard:=true
```

Pool/hardware:

```bash
ros2 launch sub_bringup pool_test_launch.py controller:=feedforward dashboard:=true
```

Then open http://127.0.0.1:8080. To access it from another computer on the pool network, add `dashboard_host:=0.0.0.0` and open `http://ROBOT_IP:8080`.

The dashboard can also attach to an already-running stack:

```bash
ros2 run sub_pid_tuner dashboard --robot-name marlin_v2 --controller-node sub_control \
  --profile install/share/sub_bringup/config/control_ff_sim.yaml
```

For the known-working pool fallback, launch with `controller:=mcu`; its dashboard
automatically targets `sub_control_mcu` and `control_gains_mcu.yaml`.


## Saving configuration

- Typing changes only stages the value; yellow rows have unapplied edits.
- A row's **Apply** button changes that parameter on the running node.
- **Apply runtime** or `Ctrl+S` sends all staged edits. Parameters for each ROS node are sent atomically, and the dashboard clears the yellow state only after ROS readback matches.
- **Save profile** or `Ctrl+Shift+S` first performs the same runtime update, then atomically replaces the configured YAML profile. A timestamped backup is stored under `~/.local/state/avbotz/dashboard/backups`.
- **Reload** discards staged browser edits and refreshes values from ROS and disk.
