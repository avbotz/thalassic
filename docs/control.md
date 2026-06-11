# Control System

## Cascade PID

The `sub_control` node runs a two-layer (cascade) PID loop at 50 Hz (`control_rate_hz`).

```
pos_setpoint ──► pos_pid ──► vel_setpoint ──► vel_pid ──► wrench[0:3] (Fx,Fy,Fz)
                                                                       ├──► allocate ──► force_to_norm ──► control/thruster_i
att_setpoint ──► att_pid ──► ang_setpoint ──► ang_pid ──► wrench[3:6] (Mx,My,Mz)
```

The outer loops convert position/attitude error into velocity/angular-rate setpoints for the inner loops; the inner loops produce the body wrench. Either layer can be bypassed by sending a `Setpoint` with `velocity=true`, which writes directly into the inner-loop setpoint.

**Default hold mode:** an un-commanded sub damps to rest and holds its attitude. The translation loops default to **velocity-hold** (zero body velocity) rather than absolute position hold, because horizontal/vertical position is dead-reckoned from DVL velocity (no absolute fix) — actively holding an absolute position would chase that drift and slowly drive the sub around. Attitude is absolute (IMU), so the attitude loop holds level + startup heading by default. Send a position `Setpoint` (`velocity=false`) to switch the translation loops into absolute position hold on demand.

### PID implementation

Each axis is a parallel-form PID (`src/sub_drivers/sub_control/src/PID.cpp`):

```
u = kp*error + ki*∫error dt + kd * d(measurement)/dt   (clamped to [out_min, out_max])
```

- **Derivative on measurement**, low-pass filtered (`tau_d`) — no derivative kick on setpoint steps, and thruster/DVL/IMU noise is not amplified.
- **Back-calculation anti-windup** — when the output saturates, the part of the integrator that pushed past the limit is removed.
- Inner-loop (`vel_pid`/`ang_pid`) output limits are derived at startup from the **allocator's real per-axis force/torque capacity** (`ThrusterAllocator::max_wrench`), not hand-picked.

### Gains (ROS parameters — live-tunable)

All gains are ROS parameters with an `on_set_parameters` callback, so they can be tuned on a **running** node (no rebuild). Outer loops are P-only; inner loops are full PID. Defaults:

```
pos_kp = [0.8, 0.8, 0.8]                          -> vel setpoint [m/s], limit ±max_speed (1.0)
vel_kp = [40, 40, 60]  vel_ki = [8, 8, 12]  vel_kd = [2, 2, 3]   (vel_tau_d 0.05)  -> body force [N]
att_kp = [1.5, 1.5, 1.0]                          -> ang-rate setpoint [rad/s], limit ±max_ang_rate (0.6)
ang_kp = [6, 6, 8]     ang_ki = [0, 0, 0]   ang_kd = [1, 1, 1.5] (ang_tau_d 0.05)  -> body torque [N·m]
```

Arrays are `[roll/x, pitch/y, yaw/z]`. Example live tune:

```
ros2 param set /marlin_v2/sub_control ang_kp "[8.0, 8.0, 10.0]"
ros2 param set /marlin_v2/sub_control max_ang_rate 0.8
```

**Why the attitude loop starts gentle:** the horizontal layout has ~20× weaker yaw authority than surge/sway, so `cap[yaw]` ≈ 4.9 N·m. The attitude loops were never exercised before (the inner torque loop used to be commented out), so the defaults are deliberately conservative — low `max_ang_rate`, modest `ang_kp`, and **no integrator** (`ang_ki = 0`) to avoid windup/limit-cycling. Recommended tuning order: raise `ang_kp` until the rate response is crisp, add a little `ang_kd` for damping, then raise `att_kp`, and only add `ang_ki` last to trim steady-state offset.

## Coordinate Frames

ROS2 uses REP-103: **ENU world** (x=East, y=North, z=Up) and **FLU body** (x=Forward, y=Left, z=Up). The Arduino firmware the PID gains were originally tuned for uses **NED world / FRD body**. `sub_control` converts at its boundaries so the PID math stays in NED/FRD internally.

| Conversion | Formula |
|---|---|
| ENU position → NED | north=y, east=x, down=−z |
| ENU RPY → NED RPY | roll_n=roll_e, pitch_n=−pitch_e, yaw_n=π/2−yaw_e |
| FLU twist → FRD | forward same, y→−y, z→−z |

## Setpoint Interpretation

Position setpoints are in the **initial-body-aligned frame**: axes are aligned with the heading the sub had when `sub_control` first received a valid TF. This matches embedded dead-reckoning behavior. `pos_setpoint.x=5` means "5 m forward from the startup heading," not "5 m East."

For depth control, set `use_altitude=true` in the position `Setpoint` to control altitude above the seafloor instead of absolute depth. The depth error becomes `altitude_current − altitude_setpoint` (positive = sink, consistent with the NED down axis).

## Setpoint Message

```
# sub_control_interfaces/Setpoint
bool velocity       # true = setpoint is for the inner loop (velocity/angular-rate)
bool use_altitude   # true = z is altitude above bottom, not depth (pos only)
geometry_msgs/Vector3 setpoint  # x, y, z in NED/FRD units (m or m/s or rad/s)
```

Topics:
- `pos_setpoint` — position (m) or linear velocity (m/s) in initial-body frame
- `att_setpoint` — attitude (rad) or angular rate (rad/s) in body frame

## Thruster Allocation

The inner loops produce a desired body wrench `[Fx, Fy, Fz, Mx, My, Mz]` in the FRD control frame. `ThrusterAllocator` (`src/sub_drivers/sub_control/src/utils.cpp`) maps it to a force [N] for each of the 8 thrusters, which `force_to_norm()` then converts to the normalized `[−1, 1]` command published on `control/thruster_i`.

- **Thrusters 0–3** are the horizontal vectored units (45°) → **surge / sway / yaw**.
- **Thrusters 4–7** are the vertical units → **heave / roll / pitch**.

The allocation matrix is **built at startup from the thruster geometry**, not hand-derived. For each thruster the actuation column is `[ axis ; r × axis ]` (unit thrust direction and its moment), giving the actuation matrix `B (6×8)`; the allocator is `A = pinv(B)` (Eigen `completeOrthogonalDecomposition`). The geometry constants in `utils.cpp` are copied 1:1 from the `base_link_ned → thruster_i_link` static transforms in `sub_bringup/launch/marlin_v2_launch.py`, so the allocator stays consistent with the frames and the simulator by construction.

> Why this matters: the previous hand-derived matrix baked in a stale 90° rotation (it assumed `base_link_ned` was yaw=π/2, but the launch file defines it as pure roll=π). That turned every **surge** command into **sway**, so the world-frame position loop never converged and the sub circled away from its target. The geometry-driven build removes that whole class of error; `test/test_utils.cpp` asserts `B·allocate(w) == w` and that surge produces pure forward thrust.

This layout has **weak yaw authority** — a unit of yaw demand costs ~7× the thrust of a unit of surge — so yaw saturates first and is given a high gain / low torque limit.

If any thruster force exceeds `thruster_max_force` (default 35 N), the whole vector is scaled down uniformly, preserving the wrench **direction** so combined-axis commands don't veer off course at saturation.

## Diagnostic Topics

The control node publishes per-axis errors for tuning/debugging:

| Topic | Content |
|---|---|
| `/control/pos/x|y|z` | Position error (m) in body frame |
| `/control/vel/x|y|z` | Velocity error (m/s) |
| `/control/ang/x|y|z` | Attitude error (rad) |
| `/control/angvel/x|y|z` | Angular-velocity error (rad/s) |

Use `rqt_plot /control/pos/x /control/pos/y /control/pos/z` to monitor convergence.
