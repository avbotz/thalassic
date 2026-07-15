# Pool feedforward + PI quick start

## Launch

```bash
cd ~/dev/AVBotz/thalassic
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch sub_bringup pool_test_launch.py controller:=feedforward \
  dashboard:=true dashboard_host:=0.0.0.0
```

Open `http://ROBOT_IP:8080`. Run no mission. Confirm **Telemetry live**, node
`/marlin_v2/sub_control`, and no duplicate-source warning. Keep the sub tethered,
the kill operator ready, and `power_limit` at `0.15`–`0.25` initially.

## Tune feedforward first

Set `feedback.ki` to zero and leave `feedback.kp` conservative. Open **Control →
Feedforward identification**, select X, and use the defaults. Click
**Characterize axis**. The dashboard performs zero, low-speed, and high-speed
passes in both directions and rejects data with saturation, allocation error,
stale telemetry, or excessive cross-axis motion.

Repeat the same run before applying. `kV` should agree within about 15%. Increase
**Dwell** if the sub has not reached a clear speed plateau. Reduce **Maximum
speed** or **Fast slope** if any saturation is reported. A repeatable
positive/negative mismatch is a tether, trim, or thruster issue—not integral
gain.

Click **Apply estimates** only after two consistent runs, then verify with one
**Velocity → Trapezoid cruise** cycle. The calibration uses a robust effective
`kV`, sets `kQ` to zero, and keeps the current `kA`/effective mass. The displayed
transient `kA` estimate is diagnostic only because DVL/EKF lag makes automatic
mass fitting risky. Repeat for Y, Z, and Yaw; approach Roll/Pitch cautiously.

## Add PI, then the outer loops

With feedforward close:

1. Raise the selected `feedback.kp` until error corrects promptly; back off for
   oscillation or cross-axis motion.
2. Add `feedback.ki` slowly and only for repeatable steady bias.
3. Verify with **Step + settle**, then **Windowed sine**.
4. Tune `reference.position_kp` and `reference.attitude_kp` with **Smooth move**
   and **Hold / disturbance**.
5. Finish with the slow **Follower PID loop**. Path tests validate the follower;
   they do not identify feedforward constants.

Use **Apply** during experiments. After a stable positive/negative run, use
**Save profile**, relaunch, and verify the values reloaded from
`control_ff_pool.yaml`.

Kill immediately for stale telemetry, unexpected depth/attitude drift, strong
off-axis motion, oscillation, or persistent `wrench.residual` (saturation).
See [TUNING.md](TUNING.md) for the detailed workflow and troubleshooting.
