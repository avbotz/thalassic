# One motion command for sub_control, with a mode per axis so that, e.g., the
# vehicle can surge at a set speed while holding depth and heading. Axes are
# [x, y, z, roll, pitch, yaw]; an axis left at KEEP carries on with whatever
# it was doing. See docs/control.md#command-interface.
#
# Frames: positions are in the odom frame (REP-103 ENU, z up). x, y and yaw
# are zeroed where the kill switch was last released (or sub_control's
# reset_pose service was last called); z is depth from the
# pressure sensor, so 1 m deep is z = -1.
# Rotations are ZYX Euler angles [rad] of the body in odom, and rotational
# velocities are the rates of those angles [rad/s].

std_msgs/Header header

uint8 KEEP=0      # leave this axis as it is
uint8 POSITION=1  # move smoothly to `position` and hold it
uint8 VELOCITY=2  # track `velocity`
uint8 HOLD=3      # come to a smooth stop and hold wherever that is
uint8 EFFORT=4    # open loop: push with `effort` along the body axis, no feedback
uint8[6] mode

float64[6] position  # POSITION target [m, m, m, rad, rad, rad]
float64[6] velocity  # VELOCITY target [m/s, m/s, m/s, rad/s, rad/s, rad/s]
float64[6] effort    # EFFORT [N, N, N, N*m, N*m, N*m] along/about body x, y, z

# x and y always share a mode. Their velocity is along the vehicle's heading
# (yaw only, so pitching does not turn "forward" into a dive) or along odom x/y.
uint8 HEADING=0
uint8 WORLD=1
uint8 velocity_frame

# z tracks DVL altitude above the bottom (positive up) instead of odom z.
# Applies to a z POSITION target only, which must not be negative.
bool altitude

# VELOCITY and EFFORT axes come to a HOLD unless set again within this many
# seconds of this message arriving, so a stream that stops cannot drive the
# vehicle away. 0 latches them until they are replaced.
float64 timeout
