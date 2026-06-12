# Control System

## State-Feedback Controller

`sub_control` runs a fault-aware cascaded state-feedback controller at 50 Hz.
The ROS topics and `Setpoint` message are unchanged.

```
position error ──► nonlinear guidance ──► velocity setpoint ──► PI force control
attitude error ──► nonlinear guidance ──► rate setpoint ─────► PI torque control
                                                                  │
                                                                  ▼
                                      bounded weighted thruster allocation
                                                                  │
                                                                  ▼
                                                     control/thruster_0..7
```

The outer position guidance law combines proportional and bounded integral
position feedback, then applies `limit * tanh(command / limit)`. This removes
steady position offsets caused by velocity bias or drag while approaching the
configured speed limit smoothly. Velocity and angular-rate setpoints are
slew-limited to prevent current spikes and abrupt vehicle motion.

The inner loops are PI state-feedback controllers. Measured DVL velocity and
IMU angular rate already provide the damping term, so numerical derivatives are
not used. Conditional integration prevents windup while force or torque output
is saturated.

Default translation mode is zero-velocity hold. Default attitude mode holds
level and the heading captured when the kill switch is released.

## Attitude Control

Attitude error is not calculated by independently subtracting Euler angles.
Current and desired RPY values are converted to rotation matrices, and the
relative rotation is converted to a body-frame rotation vector. This avoids
axis coupling and angle-wrap errors during combined roll, pitch, and yaw motion.

## Safety

The node publishes zero to every thruster when:

- the kill switch is active;
- filtered odometry is older than `feedback_timeout`;
- altitude mode is active and DVL altitude is stale.

Controller integrators and command ramps reset on every safety stop. Releasing
the kill switch captures the current position and heading as the new origin.

## Coordinate Frames

ROS uses ENU world and FLU body coordinates. Control calculations use NED world
and FRD body coordinates.

| Conversion | Formula |
|---|---|
| ENU position to NED | north=y, east=x, down=-z |
| ENU RPY to NED RPY | roll=roll, pitch=-pitch, yaw=pi/2-yaw |
| FLU twist to FRD | x=x, y=-y, z=-z |

Position setpoints are expressed in the initial-heading frame. A position
command of `x=5` means five metres forward from the heading captured when the
controller was armed. Direct velocity and angular-rate commands are body-frame
FRD commands.

For altitude control, set `use_altitude=true` on a position command. Positive
vertical error commands downward force, consistent with NED.

## Thruster Allocation

The allocator builds the 6x8 actuation matrix from the vehicle's thruster
positions and orientations. Unsaturated commands use its exact pseudoinverse.

When a command exceeds a thruster force limit, allocation becomes a bounded
weighted least-squares problem:

```
minimize ||W(Bf - requested_wrench)||^2 + epsilon||f||^2
subject to -max_force <= f_i <= max_force
```

Projected-gradient refinement enforces every thruster bound. Axis weights are
configurable; the supplied profiles prioritize heave, roll, and pitch so depth
and leveling remain controlled when simultaneous surge or yaw commands
saturate the vehicle.

## Configuration

Hardware and simulation profiles are in:

- `src/sub_bringup/config/control_gains.yaml`
- `src/sub_bringup/config/control_gains_sim.yaml`

Important parameter groups:

| Parameter | Meaning |
|---|---|
| `controller.position_gain` | Position-error to velocity guidance gain |
| `controller.position_integral_gain` | Removes persistent absolute-position error |
| `controller.attitude_gain` | Rotation-error to angular-rate guidance gain |
| `controller.velocity_kp/ki` | Linear velocity PI gains |
| `controller.angular_rate_kp/ki` | Angular-rate PI gains |
| `limits.velocity` | Maximum body velocity command |
| `limits.angular_rate` | Maximum body angular-rate command |
| `limits.linear_acceleration` | Velocity setpoint slew rate |
| `limits.angular_acceleration` | Rate setpoint slew rate |
| `limits.position_integral` | Position-guidance integral bound |
| `allocator.axis_weights` | Surge, sway, heave, roll, pitch, yaw priorities |
| `feedback_timeout` | Maximum feedback age before zero thrust |

## Diagnostics

`control/error` publishes `sub_control_interfaces/Error` with position,
velocity, geometric attitude, and angular-rate errors.

## Automated Tuning

The installed `tune_sub_control` executable performs a conservative staged
coordinate search on one axis. It tunes inner velocity/rate gains first with a
direct command, then tunes outer position/attitude guidance. For each gain array
it scores error, overshoot, steady-state error, and thruster effort.

By default it is a dry run: all original parameters are restored after printing
the recommendation. The command always sends a zero setpoint on exit or abort.

```bash
# Dry-run tune of a 0.5 m surge position step
ros2 run sub_control tune_sub_control \
  --namespace /marlin_v2 --loop position --axis x --amplitude 0.5

# Keep the selected live parameters
ros2 run sub_control tune_sub_control \
  --namespace /marlin_v2 --loop position --axis x --amplitude 0.5 --apply

# Tune yaw attitude with a 0.35 rad step
ros2 run sub_control tune_sub_control \
  --namespace /marlin_v2 --loop attitude --axis z --amplitude 0.35
```

Run one axis at a time in an obstacle-free test area. Start with the supplied
`0.75,1.0,1.25` search scales. Use `--scales` only after reviewing the first
report. `--inner-amplitude` controls the direct velocity or angular-rate step.
`--apply` changes the running node but does not edit the YAML profile; copy the
printed arrays into the hardware or simulation profile after testing.
