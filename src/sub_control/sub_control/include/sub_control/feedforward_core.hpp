#ifndef SUB_CONTROL_FEEDFORWARD_CORE_HPP_
#define SUB_CONTROL_FEEDFORWARD_CORE_HPP_

#include <array>
#include <cstddef>

namespace sub_control::feedforward
{

constexpr std::size_t kDof = 6;
constexpr std::size_t kThrusters = 8;

using Vector6 = std::array<double, kDof>;
using Vector8 = std::array<double, kThrusters>;

struct ThrusterGeometry
{
  std::array<double, 3> position_flu_m{};
  std::array<double, 3> positive_direction_flu{};
  double max_forward_n{0.0};
  double max_reverse_n{0.0};
};

struct AllocationResult
{
  Vector8 thrust_n{};
  Vector6 achieved_wrench{};
  Vector6 residual_wrench{};
  std::array<bool, kThrusters> saturated{};
};

// Thruster allocation matrix (TAM) with columns [direction; position x direction].
// All geometry is explicit in the controller's FLU body frame, so frame and
// propeller-handedness conversions cannot be hidden inside the allocator.
class ThrusterAllocator6D {
public:
  explicit ThrusterAllocator6D(const std::array<ThrusterGeometry, kThrusters> & geometry);

  AllocationResult allocate(
    const Vector6 & desired_wrench,
    const Vector6 & axis_weights = {1.0, 1.0, 2.0, 2.0, 2.0, 1.5}) const;

  Vector6 wrench_from_thrust(const Vector8 & thrust_n) const;
  const std::array<std::array<double, kThrusters>, kDof> & matrix() const {return tam_;}

private:
  std::array<ThrusterGeometry, kThrusters> geometry_{};
  std::array<std::array<double, kThrusters>, kDof> tam_{};
};

// Marlin V2 geometry expressed directly in FLU. Positive direction includes
// the installed propeller handedness and matches sub_control_mcu's proven
// channel order/sign matrix.
std::array<ThrusterGeometry, kThrusters> marlin_v2_thruster_geometry(
  double max_forward_n,
  double max_reverse_n);

struct DiagonalMarineModel
{
  Vector6 effective_mass{};         // kg for XYZ, kg*m^2 for roll/pitch/yaw
  Vector6 linear_drag{};            // wrench per velocity
  Vector6 quadratic_drag{};         // wrench per velocity*abs(velocity)
  Vector6 trim{};                   // constant body wrench (buoyancy/weight/trim)
  Vector6 restoring_stiffness{};    // wrench per sin(configuration), mainly roll/pitch

  Vector6 feedforward(
    const Vector6 & target_velocity, const Vector6 & target_acceleration,
    const Vector6 & configuration = {}) const;
};

struct MotionReference
{
  Vector6 velocity{};
  Vector6 acceleration{};
};

struct FeedbackResult
{
  Vector6 effort{};
  Vector6 integral{};
};

// Diagonal PI correction in wrench units. Feedforward is passed in so
// conditional integration uses the remaining total-wrench authority rather
// than pretending feedback owns the entire actuator range.
class FeedbackPI6D {
public:
  FeedbackPI6D(const Vector6 & kp, const Vector6 & ki, const Vector6 & integral_limit);

  FeedbackResult update(
    const Vector6 & error, const Vector6 & feedforward, const Vector6 & positive_total_limit,
    const Vector6 & negative_total_limit, double dt);
  void apply_allocation_residual(
    const Vector6 & residual_wrench, double dt,
    double back_calculation_gain);
  void reset();
  void configure(const Vector6 & kp, const Vector6 & ki, const Vector6 & integral_limit);

private:
  Vector6 kp_{};
  Vector6 ki_{};
  Vector6 integral_limit_{};
  Vector6 integral_{};
};

// Acceleration-limits velocity commands and provides a trustworthy target
// acceleration for feedforward. It never differentiates noisy measurements.
class VelocityReferenceLimiter {
public:
  explicit VelocityReferenceLimiter(const Vector6 & max_acceleration);

  MotionReference update(const Vector6 & target_velocity, double dt);
  void reset(const Vector6 & velocity = {});
  void set_max_acceleration(const Vector6 & max_acceleration);

private:
  Vector6 max_acceleration_{};
  Vector6 velocity_{};
};

}  // namespace sub_control::feedforward

#endif  // SUB_CONTROL_FEEDFORWARD_CORE_HPP_
