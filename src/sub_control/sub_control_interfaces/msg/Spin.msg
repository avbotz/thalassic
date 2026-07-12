# Relative multi-turn yaw command (REP-103 ENU, CCW-positive). Handled by the
# sub_control node only; sub_control_mcu ignores the spin_setpoint topic.
float64 yaw       # signed relative yaw to rotate in rad; may exceed 2*pi
float64 max_rate  # cruise yaw-rate cap in rad/s; <= 0 uses the spin_max_yaw_rate parameter
