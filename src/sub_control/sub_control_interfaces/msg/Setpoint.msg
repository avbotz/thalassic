# Control setpoint for sub_control. When velocity is false the axes are
# absolute targets (position in m on pos_setpoint, attitude in rad on
# att_setpoint); when true they are rates (linear m/s or angular rad/s).
# Axes follow REP-103 body FLU (x forward, y left, z up). Translational
# commands use setpoint.x/y/z; rotational commands use setpoint.yaw/pitch/roll.
bool velocity
SetpointAxes setpoint
