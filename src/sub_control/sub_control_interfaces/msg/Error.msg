# What the mission still has to go, every control cycle except while sub_control
# waits for the EKF's pose reset after a kill-switch release: the commanded target
# minus the measured state, so it reaches zero when the vehicle arrives (not when
# it catches the smoothed reference). Axis order is [x, y, z].
#   pos_error    - odom (ENU) frame [m]; z is altitude when holding altitude.
#                  Zero on axes that are not in POSITION mode.
#   vel_error    - body FLU [m/s]: the VELOCITY target, or else the trajectory
#                  reference velocity, minus the measured velocity
#   att_error    - body FLU rotation vector [rad] from the current attitude to
#                  the target; zero on axes that are not in POSITION mode
#   angvel_error - body FLU [rad/s], as vel_error
std_msgs/Header header
float64[3] pos_error
float64[3] vel_error
float64[3] att_error
float64[3] angvel_error

# The id of the last MotionSetpoint sub_control took up (0 if it had none), and
# whether it rejected it (unknown mode, non-finite target, no DVL altitude,
# killed: sub_control logs why). Errors above reflect that command.
uint64 command_id
bool command_rejected
