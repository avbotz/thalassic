# Simulation tuning quick start

## 1. Launch

```bash
cd ~/dev/AVBotz/thalassic
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch sub_bringup sim_launch.py controller:=feedforward dashboard:=true
```

Open <http://127.0.0.1:8080>. Run no mission. Confirm **Telemetry live**,
**Armed**, and no duplicate-source warning.

## 2. Identify feedforward first

1. In **Configuration**, select `/marlin_v2/sub_control`, filter `feedback.ki`,
   set all six axes to `0`, and click the row's **Apply** button. Leave
   `feedback.kp` at its current value.
2. Go to **Control → Feedforward identification**. Test axes in this order:
   **X, Y, Z, Yaw, Roll, Pitch**. Select an axis, keep its automatically loaded
   defaults, and click **Characterize axis**. Each run takes about 42–55 seconds.
3. Wait for **Fit passed**. If rejected:

   - Saturation/residual failure: lower **Maximum speed** or **Fast slope**.
   - Poor steady fit/not settled: increase **Dwell**.
   - Cross-axis motion: lower speed and fix that axis's feedback before retrying.

4. Review the result, then click **Apply estimates**. It updates that axis's
   trim and linear drag, sets quadratic drag to zero, and for Roll/Pitch also
   updates restoring stiffness. The displayed acceleration/mass estimate is
   diagnostic only; the current effective mass is retained.
5. Run the same axis again if you want a repeatability check, then validate it
   with **Trapezoid cruise**. Applied estimates are runtime-only until saved.

## 3. Tune velocity and angular-rate PI

Tune **Velocity X/Y/Z**, then **Angular rate Roll/Pitch/Yaw**, one axis at a
time. In **Control**, select **Trapezoid cruise**, one cycle, and start with:

| Controller | Command | Ramp slope | Cruise/settle |
|---|---:|---:|---:|
| Velocity | `0.10 m/s` | `0.04 m/s²` | `3 s` |
| Angular rate | `0.12 rad/s` | `0.06 rad/s²` | `3 s` |

Watch **Graph → Target / current / error** and edit the matching element of
`feedback.kp` in **Configuration**:

- Slow response or large error: increase Kp by about 20%, **Apply**, and rerun.
- Oscillation or overshoot: decrease Kp by about 20%.
- Clean tracking and clean return to zero: keep it.

If steady error remains on the flat cruise, start Ki near `Kp/100`. Increase it
slowly; reduce it if the response drifts or overshoots after returning to zero.
Verify each axis with **Step + settle**. This inner controller is PI—there is no
D gain. If `wrench.residual` remains nonzero, reduce the command; do not tune
through saturation.

## 4. Tune position and attitude

After the six inner axes are stable, use **Control → Smooth move**. Tune
**Position X/Y/Z**, then **Attitude Roll/Pitch/Yaw**. Start with `0.5 m` or
`0.15 rad`, a `4 s` move, `2 s` hold, and one cycle.

Edit the matching `reference.position_kp` or `reference.attitude_kp` element:
increase 20% for a slow response; decrease 20% for overshoot. Finish with
**Windowed sine**, then a slow **Follower PID loop**.

## 5. Save

Use **Apply** while testing. When everything is stable, click **Save profile**
to write `control_ff_sim.yaml`. Relaunch once and confirm the values reload.

For pool work, use [POOL_FF_QUICKSTART.md](POOL_FF_QUICKSTART.md).
