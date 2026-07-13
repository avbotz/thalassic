# PID tuning with AVBotz Dashboard

Tune one axis at a time with no mission running. Start in sim, keep the kill
switch within reach in the pool, and begin at a low `power_limit`.

## 1. Tune the inner loops first

Tune velocity x/y/z, then angular rate roll/pitch/yaw. Select the controller and
axis in **Control**, then use:

- **Step + settle** for rise time, overshoot, steady error, and positive/negative
  asymmetry. Each direction returns to zero and settles before the next one, so
  momentum from a reversal is not mistaken for bad PID.
- **Windowed sine** after the step looks stable. Start with an 8–12 s period.
  Shorten it gradually to reveal phase lag; stop if the response attenuates,
  oscillates, couples strongly into another axis, or saturates the thrusters.

Increase P until tracking is responsive, then back off if it oscillates. Add a
small D only when damping is needed. Add I slowly and only for persistent bias;
too much I causes slow overshoot and poor recovery after saturation.

## 2. Tune the outer loops

With inner gains fixed, tune position x/y/z and attitude roll/pitch/yaw.

- **Smooth move** uses a minimum-jerk out-and-back reference. It has no target
  jump and no linear-ramp corner impulse. Judge peak/RMS error, lag, overshoot,
  and off-axis motion—not perfect overlap during the move. The current cascade
  creates velocity/rate demand from pose error, so some moving-target error is
  expected.
- **Hold / disturbance** captures the current pose. Gently displace the vehicle
  and watch recovery. This is the best check for holding stiffness, damping,
  and integral bias without commanding a large move.
- **Step + settle** is an optional small-signal transient check. Keep its travel
  much smaller than a normal mission setpoint.

## 3. Apply and save

- Edit a gain and click its row **Apply**, or use **Apply runtime** / `Ctrl+S`.
  The running controller changes immediately after verified ROS readback.
- Use **Save profile** / `Ctrl+Shift+S` only after a stable run. Sim writes the
  sim gains file; pool writes `sub_control_mcu/config/control_gains_mcu.yaml`.
- Sim gains are `[Kp, Ki, Kd, output limit]`. Pool MCU gains are standard-form
  `[Kp, Ti, Td]`; `Ti` is integral time, not `Ki`. Do not copy gains between the
  two controllers.

## Pool stop conditions

Stop immediately for stale telemetry, unexpected depth/attitude drift, strong
off-axis motion, persistent thruster saturation, or oscillation. The dashboard
stops and holds measured pose when the initiating browser disconnects, telemetry
goes stale, the operator presses Stop, or the routine finishes. A kill aborts
without publishing another command.

Road Runner-style feedforward fitting is not exposed as a dashboard tuner yet.
The AUV first needs body effort telemetry and a controller feedforward path; a
closed-loop ramp cannot identify drag or added mass honestly.
