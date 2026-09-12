# Control System

## Cascade PID Controller

`sub_control` runs a 6-DOF cascade PID controller at 50 Hz. Each of the six body axes is driven by an outer "guidance" PID feeding an inner "effort" PID:

```
position error ──► position PID ──► velocity setpoint ─┐
                                                        ├─► velocity PID ──► body force  ─┐
                              (direct cmd_vel / setpoint)┘                                │
                                                                                          ├─► thruster allocation ──► control/thruster_0..7
attitude error ──► attitude PID ──► angular-rate setpoint ─┐                              │
                                                           ├─► angular-rate PID ─► body torque ┘
                              (direct cmd_vel / setpoint)──┘
```

There are four PID banks, each holding one independent controller per axis (`x`, `y`, `z` for translation; roll/pitch/yaw for rotation):

| Bank | Param group | Input | Output |
|---|---|---|---|
| Position | `pos_pid` | world position error [m] | world velocity setpoint [m/s] |
| Velocity | `vel_pid` | body velocity error [m/s] | body force [N] |
| Attitude | `att_pid` | geodesic attitude error [rad] | body angular-rate setpoint [rad/s] |
| Angular rate | `ang_pid` | body angular-rate error [rad/s] | body torque [N·m] |

The position loop produces a velocity setpoint in the world frame, which is rotated into the body frame (via the inverse of the current orientation) before the velocity loop runs. The attitude and angular-rate loops both operate in the body frame, so the geodesic attitude error feeds the rate loop directly.

When an outer loop is bypassed (see [Command Interface](#command-interface)), its inner loop is driven by the commanded velocity or angular rate instead.

## PID Internals

Each axis is an independent-form PID:

```
output = kp * error + ki * ∫error dt + kd * d(-measurement)/dt
```

- **Derivative on measurement.** The derivative term uses the negated measurement rate, not the error rate, to avoid derivative kick on setpoint steps. It is additionally low-pass smoothed (`ALPHA`) to reject noise.
- **Hard saturation with anti-windup.** Output is clamped to `±max_output`. When the raw output is clipped, the excess is back-calculated off the integrator (`integral -= raw - clamped`) so the integral cannot wind up against the limit. A `max_output` of `0` disables both the clamp and the anti-windup.
- **Reset.** `reset()` zeroes the integrator, smoothed derivative, and previous measurement. The controller calls it on every kill-switch release.

Each bank's gains are supplied as a `[kp, ki, kd, max_output]` array per axis.

## Command Interface

Setpoints arrive on three topics. All command frames are REP-103 body **FLU** (x forward, y left, z up); a downward (dive) command is negative `z`.

| Topic | Type | Meaning |
|---|---|---|
| `pos_setpoint` | `sub_control_interfaces/Setpoint` | position **or** body-velocity hold |
| `att_setpoint` | `sub_control_interfaces/Setpoint` | attitude **or** body angular-rate hold |
| `cmd_vel` | `geometry_msgs/Twist` | direct body velocity + angular rate |

`Setpoint` carries two flags and a `SetpointAxes` payload:

```
bool velocity      # true: treat payload as a velocity/rate setpoint (bypass outer loop)
bool altitude      # pos_setpoint only: hold DVL altitude instead of odom z
SetpointAxes setpoint
```

`SetpointAxes` fills `x/y/z` for translation (position [m] or linear velocity [m/s]) and `roll/pitch/yaw` for rotation (attitude [rad] or angular rate [rad/s]); the unused triple is ignored.

- `pos_setpoint` with `velocity=false` holds a position; with `velocity=true` it holds a body linear velocity (bypassing the position loop).
- `pos_setpoint` with `altitude=true` regulates the z axis against DVL altitude rather than the odometry z position.
- `att_setpoint` with `velocity=false` holds an attitude (RPY normalized to `[-π, π]`); with `velocity=true` it holds a body angular rate.
- `cmd_vel` enables both velocity and angular-rate hold at once.

Position and attitude setpoints are expressed relative to the pose captured when the kill switch was last released — `x=1.0` is one metre forward of the arm point. The default state is zero-velocity, level-attitude hold.

## Attitude Error

Attitude error is **not** computed by subtracting Euler angles per axis. Current and target RPY are converted to rotation matrices and the relative rotation is expressed as a body-frame rotation vector (`attitude_error()` in `utils.cpp`). This is the same frame as the angular-rate loop and avoids axis coupling and the gimbal singularity at pitch = ±90°.

## Safety

While the kill switch is active, the node publishes zero to every thruster and holds the loop timer reset. On the **release** edge (killed → unkilled) it:

1. resets the EKF pose to the origin via the `set_pose` service,
2. resets all four PID banks (integrators and derivative state), and
3. zeroes every cached setpoint and state estimate.

This captures the current pose as the new origin for subsequent position and attitude commands.

## Coordinate Frames

The controller works entirely in the EKF output frame: REP-103 **ENU** world (`marlin_v3/odom`) and **FLU** body (`marlin_v3/base_link`). Position errors are formed in the world frame and rotated into the body frame; force, torque, velocity, and angular-rate quantities are all body FLU. There is no NED/FRD conversion in the control path.

(The `base_link_ned` frame still exists in the robot model — sensors and thrusters are mounted relative to it — but it is a static mounting frame, not the control frame.)

## Thruster Allocation

The allocator builds the 6×8 actuation matrix from each thruster's position and orientation, held as `THRUSTER_GEOMETRY` in `sub_control/src/utils.cpp`. It is a compile-time constant, so it cannot read the vehicle description — the same eight poses appear in `sub_bringup/config/marlin_v3.yaml` (the TF tree) and in the simulator's `layout.scn.j2`, and moving a thruster means changing all three.

When the requested wrench keeps every thruster within the force limit, the allocator applies the exact (weighted) pseudoinverse.

When the unconstrained solution exceeds the per-thruster force limit, allocation becomes a bounded weighted least-squares problem:

```
minimize ||W (B f − requested_wrench)||² + ε||f||²
subject to −max_force ≤ f_i ≤ max_force
```

solved by an LDLᵀ seed followed by projected-gradient refinement that enforces the box bounds. Axis weights `W` prioritize heave, roll, and pitch so depth and leveling stay controlled when simultaneous surge/sway/yaw commands saturate the vehicle (default `[1, 1, 2, 2, 2, 1.5]`).

The force limit comes from `power_limit`: the normalized cap is mapped through the T200 thrust curve to a force, the allocator is bounded by that force, and each thruster output is converted back to a normalized command and clamped to `±power_limit` before publishing. `power_limit` is therefore the real authority limit.

## Configuration

Gains live in two profiles loaded at startup (sim loads the sim profile):

- `src/sub_bringup/config/control_gains.yaml` — hardware
- `src/sub_bringup/config/control_gains_sim.yaml` — simulation

| Parameter | Meaning |
|---|---|
| `control_rate_hz` | Control loop frequency (default `50.0`) |
| `power_limit` | Normalized thrust cap in `[0, 1]`; bounds the allocator |
| `robot_name` | Namespace prefix used for the `set_pose` frame id |
| `pos_pid.{x,y,z}` | Position → velocity PID, `[kp, ki, kd, max]` |
| `vel_pid.{x,y,z}` | Velocity → force PID, `[kp, ki, kd, max]` |
| `att_pid.{x,y,z}` | Attitude → rate PID, `[kp, ki, kd, max]` |
| `ang_pid.{x,y,z}` | Angular-rate → torque PID, `[kp, ki, kd, max]` |

Gains are plain declared parameters: edit the YAML and restart the node to retune. There is no live autotuner.

## Diagnostics

`control/error` publishes `sub_control_interfaces/Error` every cycle with position, velocity, geometric-attitude, and angular-rate errors, each a body-FLU `[x, y, z]` triple.
