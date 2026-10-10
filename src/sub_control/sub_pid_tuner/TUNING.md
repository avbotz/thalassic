# Tuning sub_control with the dashboard

The procedure of [docs/control.md](../../docs/control.md#tuning), step by step in the dashboard. Tune in simulation first (`gains_sim.yaml`), then on the vehicle (`gains.yaml`); gains do not carry over between them, the procedure does.

Before starting: no mission running (the dashboard refuses routines while anything else publishes `motion_setpoint` or `cmd_vel`), the vehicle unkilled at least 1.5 m deep and 2 m from walls, and in the pool someone on the kill switch. Press **Zero pose** so that odom x is forward and y left: routines move along odom axes, and the gains act along body axes.

## What to watch

The controller turns each target into a smooth reference within `trajectory.*` and tracks that reference; the feedback acts on **reference minus measured**, not on the distance to the target. So for each axis the useful views are:

- *Target / reference / measured*: the reference should lead the measurement by little and arrive at the target at rest. Measured overshooting where the reference does not is too little damping or too much bandwidth.
- *Tracking error*: small during a move with a good model and gains; how it behaves says what is wrong (below).
- *Wrench / disturbance*: commanded against achieved thrust. A gap means saturation, as does the **Saturated** pill in the header.

Body axes are FLU: +x forward, +y left, +z up; +roll lowers the right side, +pitch lowers the nose, +yaw turns left (counter-clockwise from above).

## 1. Thrust directions

In **Manual command**, push one axis at a time with `Effort` (10 N, or 2 N·m for rotations) and a 2 s timeout. The vehicle must move the way the axis points, as above. If it does not, fix the vehicle description's thruster layout before anything else.

## 2. Vehicle model

Run `identify_model` (docs/control.md, [Identifying the model](../../docs/control.md#identifying-the-model)) and paste its `model:` block into the gains file. The model is feedforward: with it right, the gains below only correct what is left. To bring it in gradually, apply `feedforward.acceleration`, `.drag` and `.restoring` between 0 and 1 from **Configuration**.

## 3. Bandwidth per axis

In **Configuration**, type a bandwidth ω into the **ω** field of `gains.<axis>`; it sets `[kp, ki, kd]` to `[3ω², ω³, 3ω]`, a critically damped triple pole. **Apply** it, then run a **Step** on that axis with a hold long enough to settle (the hint gives the move time).

- Start low: x and y 0.6, z 1.0, roll, pitch and yaw 1.5 rad/s.
- Raise ω until the step rings, then back off by a third.
- Keep x and y at about 1 rad/s or less: their feedback is the DVL, 0.1 to 0.2 s late.
- Step small: 0.5 m in z, 1 m in x and y, 0.2 rad in roll and pitch, 0.8 rad in yaw. z steps stay at least 0.3 m below the surface.

Reading the tracking error on a step:

| Tracking error | Likely cause |
|---|---|
| Follows the reference acceleration (peaks as the move starts and ends) | `model.mass` or `inertia` off, or `feedforward.acceleration` below 1 |
| Grows with speed, steady while cruising | Drag off (`model.linear_drag`, `quadratic_drag`) |
| Offset at rest that the integral slowly removes | `model.net_buoyancy` or trim off; or `ki` too low if the model is right |
| Oscillates around the reference | Bandwidth too high, or sensing too slow for it |

`gains.integral_limit` caps each integrator and `gains.anti_windup` bleeds off what the thrusters could not produce; neither needs tuning unless an axis saturates for long.

## 4. Speed: Cruise

**Cruise** an axis at a steady speed (x, y, z or yaw). Once at speed, the *integral* and *disturbance* should sit near zero with a right drag model; an integral that climbs while cruising is drag the model is missing. A speed above `trajectory.<group>` v max is clipped by the controller.

## 5. Trajectory limits

Raise `trajectory.<group>` `[v max, a max, j max]` until long steps and cruises show **Saturated**, then back off. They must stay inside what the thrusters can do at `power_limit`.

## 6. Disturbances and coupling

- **Hold** keeps every axis where it is. Push the vehicle and watch it return without ringing. In sim, the `current` launch argument sets a steady current to hold against.
- With the model identified, set `observer.bandwidth` per axis (at most the feedback bandwidth on x and y) and cut `ki` to a small residual; the observer then takes out steps in buoyancy or current faster than the integrators.
- **Square** moves through four corners in x/y together: the reference runs in straight lines, and depth, attitude and heading should hold through it. Check the Path view from above and the side.

## 7. Save

**Save profile** writes the controller's tuned values to the gains file, keeping its comments. Update the file's comments if they no longer describe the values (for example the bandwidths they were set from), and review the change with `git diff`.

## Stop

Press **Stop** or **Hold here**, or kill the vehicle, on oscillation, a persistent **Saturated**, strong motion on an axis that was not commanded, depth or attitude drifting, or stale telemetry. A routine stops on its own if telemetry goes stale, another publisher appears, or the browser tab that started it closes; it then leaves every axis stopping smoothly and holding, level.
