#ifndef SUB_CONTROL_UTILS_HPP_
#define SUB_CONTROL_UTILS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

static constexpr int NUM_THRUSTERS = 8;
static constexpr int NUM_DOF = 6;

using Vector6d = Eigen::Matrix<double, NUM_DOF, 1>;

// Axis order of every 6-vector here: body forces [x, y, z] then torques [roll,
// pitch, yaw], or positions then ZYX Euler angles.
enum Axis { X = 0, Y = 1, Z = 2, ROLL = 3, PITCH = 4, YAW = 5 };

// Invert the BlueRobotics T200 thrust curve of a right-hand propeller: desired
// force [N] along the thruster's +X -> normalized command in [-1, 1] (+/-1 ==
// +/-400 PWM counts == full range). The curve's deadband collapses to 0. This is
// the exact curve the simulator models, so the round trip (command -> sim
// thrust) is faithful, and the same normalized value drives the real ESCs.
// NaN maps to 0 (off), here and in norm_to_force().
double force_to_norm(double force_n);

// Forward BlueRobotics T200 curve: normalized command to force [N].
double norm_to_force(double normalized);

// The same maps for any thruster. A left-hand propeller is the mirror image of
// a right-hand one, so it pushes along +X when spun backwards, with the curve's
// stronger forward half: thrust(cmd) = norm_to_force(-cmd). Either way the force
// along +X can range over [norm_to_force(-1), norm_to_force(1)].
double thruster_command(double force_n, bool reversed);
double thruster_force(double command, bool reversed);

// Angle wrapped into [-pi, pi].
double wrap_angle(double angle);

// Body-to-world rotation for ZYX Euler angles [roll, pitch, yaw]
// (R = Rz(yaw) * Ry(pitch) * Rx(roll)), and back.
Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d& rpy);
Eigen::Vector3d rpy_from_quaternion(const Eigen::Quaterniond& q);

// Geodesic attitude error: the body-frame rotation vector (axis * angle, the
// short way round) that carries `current` onto `target`. Unlike per-axis Euler
// differences this stays in the frame of the body angular-rate feedback and has
// no gimbal singularity, so a large yaw move does not leak roll/pitch torque.
Eigen::Vector3d attitude_error(const Eigen::Quaterniond& target, const Eigen::Quaterniond& current);

// Body angular velocity [p, q, r] for ZYX Euler angles changing at `rates`.
Eigen::Vector3d euler_rates_to_body(const Eigen::Vector3d& rpy, const Eigen::Vector3d& rates);

#endif  // SUB_CONTROL_UTILS_HPP_
