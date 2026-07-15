# sub_control_mcu

Reimplementation of the Nautical-Private MCU control firmware
(`low/Nautical-Private`, the "maritime-ec" library plus `main.cpp`'s run
loop) as a ROS 2 node. It is a drop-in alternative to `sub_control`: same
subscriptions, publications, `control_setpoint` action, and namespace layout — swap the
`package`/`executable` in the launch file and nothing else changes.

## Algorithm (ported as-is)

Two independent cascades, evaluated every tick (default 10 Hz, the MCU's
100 ms period), then a fixed sign-matrix mixer:

```
attitude ──P──▶ angvel sp (±2 rad/s) ──PID──▶ torque (±1)  ┐
                                                           ├─▶ mix[8][6] ─▶ clamp(±power) ─▶ thrusters
position ──P──▶ body vel sp (±1 m/s) ──PID──▶ force  (±1)  ┘
```

- PIDs are **standard form**: `u = kp * (e + ∫e/ti − td·de/dt)`; `ti` is an
  integral *time* (0 disables), `td` a derivative time. This differs from
  `sub_control`'s parallel-form `[kp, ki, kd, limit]` arrays, so the gains are
  **not interchangeable** (and `pid_tuner` candidates are not either).
- Anti-reset windup: when a force/torque output saturates, the tick's error
  contribution is removed from the integral (exactly the MCU's inline code;
  the standalone `angvel_controller_update` in Nautical had a `<= limit` typo
  which `main.cpp` fixed by inlining — the fixed version is ported).
- Position error is computed in the world frame, rotated into the body frame
  with the MCU's Euler matrix (`offsets_to_frame` with negated angles), then
  fed per-axis. Attitude error is per-axis `angle_difference` (unlike
  `sub_control`'s geodesic error, this couples axes near pitch ±90°).
- Altitude mode holds a distance off the floor: `down error = altitude −
  altitude_sp` (MCU `'b'` command → `Setpoint.altitude` flag).
- Mixer: `sub_mix_data` ported byte-for-byte. Deriving the sign matrix from
  Marlin V2's `THRUSTER_GEOMETRY` (sub_control/utils.cpp) and rotating
  FLU→FRD reproduces the MCU matrix row-for-row in the same channel order, so
  the mixer output feeds `control/thruster_{0..7}` directly (±1 = ±400
  counts, the same scale as the MCU's `send_thrusts`).

## Frames

The mec code runs in the MCU's native **NED/FRD**, unchanged. All topics stay
REP-103 **ENU/FLU** like the rest of the workspace; the node converts at the
callback/publish boundary only (x kept, y/z negated; roll kept, pitch/yaw
negated). `control/error` is published in FLU.

## MCU serial command → ROS mapping

| MCU | ROS |
| --- | --- |
| `s` / `z` (position + depth sp) | `pos_setpoint` (`Setpoint`, flags false) |
| `b` (altitude sp) | `pos_setpoint` with `altitude: true` |
| `v` (body velocity sp) | `pos_setpoint` with `velocity: true` |
| `n` (attitude sp) | `att_setpoint` (flags false) |
| `t` (angular rate sp) | `att_setpoint` with `velocity: true` |
| `v`+`t` together | `cmd_vel` (`Twist`) |
| monitored position / velocity / attitude request | `control_setpoint` (`ControlSetpoint` action) |
| `p` (power 0–1) | `power_limit` parameter |
| `u` (live PID gains) | `pos_pid.* / vel_pid.* / att_pid.* / ang_pid.*` parameters (`[kp, ti, td]`) |
| kill pin / `x` reset | `kill_switch` topic (revive zeroes EKF via `set_pose`, setpoints, and overrides) |
| `c` / `m` / `h` / `d` state queries | `odometry/filtered`, `altitude` topics |
| `r` (relative move) | not ported — send absolute setpoints |
| `g` / `f` / `o` (dropper/grabber/torpedo) | out of scope |

## Deliberate deviations

- **State estimation is external.** The MCU dead-reckoned DVL velocity and
  differentiated AHRS angles for rates; here `odometry/filtered` (EKF)
  supplies position, body velocity, attitude, and angular rates. The MCU's
  sensor sanity checks (5 m/s DVL spike rejection, 0.5 m position-step
  rejection) belong to that estimation layer and are not ported.
- **PID state resets on revive.** The MCU carried integrals across kill
  cycles; since the EKF teleports to the origin on revive, stale integrals
  are cleared here.
- `dt` is capped at 0.2 s to survive sim-clock stalls.
- `startup_pause_s` defaults to 0: the firmware intended a 7 s ESC startup
  pause (`PAUSE_TIME`) but compared `micros()` to a `millis()` stamp, so the
  pause it actually ran was ~0. Set the parameter to 7.0 for the intended
  behavior.
- `power_limit` defaults to 0.6 (the MCU boots at 0 until `p` is sent).

## Running

Swap the node in a bringup launch file:

```python
sub_control_node = Node(
    package="sub_control_mcu",
    executable="sub_control_mcu",
    name="sub_control_mcu",
    namespace=LaunchConfiguration("robot_name"),
    parameters=[
        PathJoinSubstitution([FindPackageShare("sub_control_mcu"), "config/control_gains_mcu.yaml"]),
        {"robot_name": LaunchConfiguration("robot_name")},
    ],
)
```
