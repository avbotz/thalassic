#ifndef SUB_CONTROL_UTILS_HPP_
#define SUB_CONTROL_UTILS_HPP_

#include <array>

static const int NUM_THRUSTERS = 8;
static const int NUM_DOF = 6;

// Channel order matches the sim scene (layout.scn.j2) and the ESC wiring. A
// positive command drives thrust along the thruster's mounted +X; the allocator
// derives each thruster's body-frame (FLU) push direction from its rpy in
// THRUSTER_GEOMETRY (axis = R(rpy) * X, then NED->FLU), so a positive wrench is
// realized with the correct sign on every axis -- verified by round-tripping a
// unit wrench through allocate() -> wrench_from_forces() with no cross-axis leak:
//   thrusters 0-3 : vertical units       (+cmd -> +Z up)  -> heave / roll / pitch
//   thrusters 4-7 : horizontal vectored  (+cmd -> +X fwd) -> surge / sway / yaw
class ThrusterAllocator {
   public:
    ThrusterAllocator();

    // Bounded weighted least-squares allocation. Higher axis weights preserve
    // those wrench components when the requested wrench is not fully achievable.
    std::array<double, NUM_THRUSTERS> allocate(const std::array<double, NUM_DOF>& wrench, double max_force,
                                               const std::array<double, NUM_DOF>& axis_weights = {1.0, 1.0, 2.0, 2.0,
                                                                                                  2.0, 1.5}) const;

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

// Geodesic attitude error: the body-frame rotation vector (axis * angle) that
// carries the current orientation to the target. Both inputs are [roll, pitch,
// yaw] for R = Rz(yaw) * Ry(pitch) * Rx(roll). Unlike per-axis Euler differences
// this stays in the same frame as the body angular-rate loop and has no gimbal
// singularity, so it does not leak roll/pitch torque during large yaw moves.
std::array<double, 3> attitude_error(const std::array<double, 3>& target_rpy, const std::array<double, 3>& current_rpy);

#endif  // SUB_CONTROL_UTILS_HPP_
