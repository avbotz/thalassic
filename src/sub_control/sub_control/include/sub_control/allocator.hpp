#ifndef SUB_CONTROL_ALLOCATOR_HPP_
#define SUB_CONTROL_ALLOCATOR_HPP_

#include "sub_control/utils.hpp"

#include <array>

// Where a thruster sits and which way it pushes, in base_link (FLU).
struct ThrusterGeometry {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};    // [m]
    Eigen::Vector3d direction{Eigen::Vector3d::UnitX()};  // unit vector along the mounted +X
    bool reversed{false};                                 // left-hand propeller (see thruster_command)
};

using ThrusterArray = std::array<double, NUM_THRUSTERS>;
using ThrusterGeometries = std::array<ThrusterGeometry, NUM_THRUSTERS>;
using AxisFlags = std::array<bool, NUM_DOF>;

// Maps a body wrench [Fx, Fy, Fz, Mx, My, Mz] (FLU, moments about the centre of
// gravity) to per-thruster forces along each thruster's mounted +X.
class ThrusterAllocator {
   public:
    ThrusterAllocator() = default;
    ThrusterAllocator(const ThrusterGeometries& thrusters, const Eigen::Vector3d& center_of_gravity);

    // Thruster forces [N] that best produce `wrench` within [lower, upper] per
    // thruster. Minimizes, in order of priority:
    //   1. the weighted wrench error ||W (B f - wrench)||, and
    //   2. ||f||^2, a stand-in for power draw, among the forces that achieve (1).
    // Higher axis weights preserve those axes when the wrench is not achievable,
    // so the weights should differ by orders of magnitude to act as priorities.
    // A failed thruster is given lower == upper == 0.
    ThrusterArray allocate(const Vector6d& wrench, const ThrusterArray& lower, const ThrusterArray& upper,
                           const Vector6d& axis_weights) const;

    // Forward map: the wrench the given per-thruster forces produce (B * forces).
    Vector6d wrench_from_forces(const ThrusterArray& forces) const;

    // Which axes the usable thrusters can still move independently. Axes are
    // taken in `priority` order (most important first); an axis whose row of B
    // is a combination of the rows already kept is reported uncontrollable, so
    // the important axes are the ones that keep the remaining authority.
    AxisFlags controllable_axes(const std::array<bool, NUM_THRUSTERS>& usable,
                                const std::array<int, NUM_DOF>& priority) const;

    const Eigen::Matrix<double, NUM_DOF, NUM_THRUSTERS>& matrix() const { return b_; }

   private:
    Eigen::Matrix<double, NUM_DOF, NUM_THRUSTERS> b_{Eigen::Matrix<double, NUM_DOF, NUM_THRUSTERS>::Zero()};
};

// Hard ceiling on |normalized command|: 1500 +/- 400 * 0.6 us (1260-1740 us) to
// the Basic ESCs. sub_low and sim_thrusters clamp to the same value, so
// power_limit can only lower it.
inline constexpr double MAX_POWER_LIMIT = 0.6;

// How the thrusters may be driven this cycle.
struct ThrusterSettings {
    double power_limit{MAX_POWER_LIMIT};           // max |normalized command|, in [0, MAX_POWER_LIMIT]
    ThrusterArray health{1, 1, 1, 1, 1, 1, 1, 1};  // fraction of nominal thrust each delivers; 0 failed
    std::array<bool, NUM_THRUSTERS> reversed{};    // left-hand propellers
    Vector6d axis_weights{Vector6d::Ones()};
    double max_command_rate{0.0};  // [1/s] slew limit on each normalized command; 0 off
};

struct ThrusterOutput {
    ThrusterArray force{};    // [N] along each mounted +X, as actually commanded
    ThrusterArray command{};  // normalized, for the ESCs
    std::array<bool, NUM_THRUSTERS> saturated{};
    Vector6d achieved{Vector6d::Zero()};  // wrench those forces produce
};

// One cycle of the output stage: allocate `wrench` within each thruster's
// thrust range (scaled by its health), convert to commands through the T200
// curve, slew-limit them from `previous`, and report the wrench the resulting
// commands really produce, which is what the controller's anti-windup needs.
ThrusterOutput drive_thrusters(const ThrusterAllocator& allocator, const Vector6d& wrench,
                               const ThrusterSettings& settings, const ThrusterArray& previous, double dt);

#endif  // SUB_CONTROL_ALLOCATOR_HPP_
