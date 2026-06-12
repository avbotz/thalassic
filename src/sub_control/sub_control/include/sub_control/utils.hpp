#ifndef SUB_CONTROL_UTILS_HPP_
#define SUB_CONTROL_UTILS_HPP_

#include <array>

static const int NUM_THRUSTERS = 8;
static const int NUM_DOF = 6;

// Wrap an angle to [-pi, pi].
double normalize_angle(double angle);

// Smallest signed rotation from `current` to `target`, wrapped to [-pi, pi].
double angle_difference(double target, double current);

// Body-frame rotation vector from current RPY to target RPY. Unlike independent
// Euler subtraction, this remains coupled and well behaved through large turns.
std::array<double, 3> attitude_error(const std::array<double, 3>& target_rpy,
                                     const std::array<double, 3>& current_rpy);

//   World position: north = enu.y, east = enu.x, down = -enu.z
void enu_to_ned_position(double ex, double ey, double ez, double& n, double& e, double& d);

//   roll_ned  =  roll_enu
//   pitch_ned = -pitch_enu
//   yaw_ned   =  pi/2 - yaw_enu   (ENU yaw=0 -> east; NED yaw=0 -> north)
void enu_to_ned_rpy(double roll_e, double pitch_e, double yaw_e, double& roll_n, double& pitch_n, double& yaw_n);

// Express a planar vector (vx, vy) in a frame rotated by +yaw about its z axis,
// i.e. multiply by R(-yaw). Used to rotate world-NED position/error into the
// body (or initial-body) frame. Returns {x_in_frame, y_in_frame}.
std::array<double, 2> to_rotated_frame(double vx, double vy, double yaw);

//   thrusters 0-3 : horizontal vectored units (45-deg) -> surge / sway / yaw
//   thrusters 4-7 : vertical units                     -> heave / roll / pitch
class ThrusterAllocator {
   public:
    ThrusterAllocator();

    // Bounded weighted least-squares allocation. Higher axis weights preserve
    // those wrench components when the requested wrench is not fully achievable.
    std::array<double, NUM_THRUSTERS> allocate(
        const std::array<double, NUM_DOF>& wrench, double max_force,
        const std::array<double, NUM_DOF>& axis_weights = {1.0, 1.0, 2.0, 2.0, 2.0, 1.5}) const;

    // Largest single-axis wrench magnitude reachable before any thruster hits
    // max_force, per DOF: max_force / max_t |A[t][dof]|. Used to size the inner
    // PID output limits from real actuator capacity instead of guesses.
    std::array<double, NUM_DOF> max_wrench(double max_force) const;

    // Forward map: the body wrench [Fx,Fy,Fz,Mx,My,Mz] that the given per-thruster
    // forces actually produce (B * forces). Since A = pinv(B) and B has full row
    // rank, wrench_from_forces(allocate(w)) == w for any unsaturated w -- the test
    // that proves the allocation is a true inverse and not rotated/swapped.
    std::array<double, NUM_DOF> wrench_from_forces(const std::array<double, NUM_THRUSTERS>& forces) const;

    // A[thruster][dof]: per-thruster force [N] per unit wrench. Exposed for tests.
    const std::array<std::array<double, NUM_DOF>, NUM_THRUSTERS>& matrix() const { return alloc_; }

   private:
    std::array<std::array<double, NUM_DOF>, NUM_THRUSTERS> alloc_{};  // A = pinv(B)
    std::array<std::array<double, NUM_THRUSTERS>, NUM_DOF> act_{};    // B
};

// Invert the BlueRobotics T200 thrust curve: desired force [N] -> normalized
// command in [-1, 1] (+/-1 == +/-400 PWM counts == full range). The curve's
// deadband collapses to 0. This is the exact curve the simulator models, so the
// round trip (command -> sim thrust) is faithful, and the same normalized value
// drives the real ESCs.
double force_to_norm(double force_n);

// Forward BlueRobotics T200 curve: normalized command to force [N].
double norm_to_force(double normalized);

#endif  // SUB_CONTROL_UTILS_HPP_
