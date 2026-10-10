# sub_control's state, every control cycle except while it waits for the EKF's
# pose reset after a kill-switch release (docs/control.md#diagnostics). Axes
# are [x, y, z, roll, pitch, yaw]; translation is in odom ENU and rotation in
# ZYX Euler angles unless a field says body (FLU).
std_msgs/Header header

uint8[6] mode    # MotionSetpoint mode of each axis
bool altitude    # z is tracking DVL altitude
float64[6] target  # where POSITION axes are headed; the reference position on the others

# The smooth trajectory the controller tracks toward the target.
float64[6] reference_position
float64[6] reference_velocity
float64[6] reference_acceleration

float64[6] wrench_command   # body FLU [N, N*m] the control law asked for
float64[6] wrench_achieved  # what the thrusters can actually produce of it
float64[6] integral         # integral action, body [m/s^2, rad/s^2]
float64[6] disturbance      # disturbance observer estimate, body [N, N*m]

float64[8] thruster_force    # [N] along each thruster's mounted +X
float64[8] thruster_command  # normalized command sent to each thruster
bool[8] thruster_saturated   # at its force limit, or held back by the command slew limit
float64[8] thruster_health   # 1 healthy, 0 failed
bool[6] axis_controllable    # false where the working thrusters cannot move this axis independently

bool killed
bool failsafe      # no state estimate: thrusters off, the vehicle floats up
bool dvl_ok        # x/y feedback active
bool depth_ok      # z feedback active
bool altitude_ok   # DVL altitude available
float64 state_age  # [s] since the last odometry
