# The normalized command for each of the eight thrusters, from sub_control every
# control cycle (zeros while killed or in failsafe). Thruster i is
# thruster_<i>_link in the vehicle description and ESC channel i; a positive
# command pushes along the thruster's +X for a right-hand propeller. sub_low,
# the firmware and the simulator each clamp it to +-0.6 (MAX_POWER_LIMIT) and
# drop a command older than their watchdog, so header.stamp must be the time it
# was computed.
std_msgs/Header header
float64[8] command
