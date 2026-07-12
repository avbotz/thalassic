# Consolidated controller tracking errors, REP-103 body FLU (x forward,
# y left, z up) to match the Setpoint command topics. Axis order is [x, y, z].
std_msgs/Header header
float64[3] pos_error
float64[3] vel_error
float64[3] att_error
float64[3] angvel_error
# True while a Spin command is executing; att_error[2] then reports the
# remaining unwrapped spin angle instead of the geodesic yaw error.
bool spin_active
