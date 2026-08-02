#include "sub_control/feedforward_core.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sub_control::feedforward
{
namespace
{

constexpr double kSqrtHalf = 0.7071067811865476;

bool finite(double value) {return std::isfinite(value);}

std::array<double, 3> cross(const std::array<double, 3> & a, const std::array<double, 3> & b)
{
  return {
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  };
}

}  // namespace

ThrusterAllocator6D::ThrusterAllocator6D(const std::array<ThrusterGeometry, kThrusters> & geometry)
: geometry_(geometry)
{
  for (std::size_t thruster = 0; thruster < kThrusters; ++thruster) {
    const auto & g = geometry_[thruster];
    const double norm = std::sqrt(g.positive_direction_flu[0] * g.positive_direction_flu[0] +
                                      g.positive_direction_flu[1] * g.positive_direction_flu[1] +
                                      g.positive_direction_flu[2] * g.positive_direction_flu[2]);
    if (!finite(norm) || norm < 1e-9 || !finite(g.max_forward_n) || !finite(g.max_reverse_n) ||
      g.max_forward_n <= 0.0 || g.max_reverse_n <= 0.0)
    {
      throw std::invalid_argument("invalid thruster geometry or force limit");
    }

    std::array<double, 3> direction{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      if (!finite(g.position_flu_m[axis]) || !finite(g.positive_direction_flu[axis])) {
        throw std::invalid_argument("thruster geometry must be finite");
      }
      direction[axis] = g.positive_direction_flu[axis] / norm;
      tam_[axis][thruster] = direction[axis];
    }
    const auto moment = cross(g.position_flu_m, direction);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      tam_[axis + 3][thruster] = moment[axis];
    }
  }
}

Vector6 ThrusterAllocator6D::wrench_from_thrust(const Vector8 & thrust_n) const
{
  Vector6 wrench{};
  for (std::size_t axis = 0; axis < kDof; ++axis) {
    for (std::size_t thruster = 0; thruster < kThrusters; ++thruster) {
      wrench[axis] += tam_[axis][thruster] * thrust_n[thruster];
    }
  }
  return wrench;
}

AllocationResult ThrusterAllocator6D::allocate(
  const Vector6 & desired_wrench,
  const Vector6 & axis_weights) const
{
  using Matrix68 = Eigen::Matrix<double, kDof, kThrusters>;
  using Matrix88 = Eigen::Matrix<double, kThrusters, kThrusters>;
  using Vector6e = Eigen::Matrix<double, kDof, 1>;
  using Vector8e = Eigen::Matrix<double, kThrusters, 1>;

  Matrix68 matrix;
  Vector6e desired;
  Eigen::DiagonalMatrix<double, kDof> weights;
  for (std::size_t axis = 0; axis < kDof; ++axis) {
    if (!finite(desired_wrench[axis]) || !finite(axis_weights[axis]) || axis_weights[axis] <= 0.0) {
      throw std::invalid_argument("desired wrench and axis weights must be finite");
    }
    desired(axis) = desired_wrench[axis];
    weights.diagonal()(axis) = axis_weights[axis];
    for (std::size_t thruster = 0; thruster < kThrusters; ++thruster) {
      matrix(axis, thruster) = tam_[axis][thruster];
    }
  }

  const Matrix68 weighted_matrix = weights * matrix;
  const Vector6e weighted_desired = weights * desired;
  constexpr double regularization = 1e-6;
  const Matrix88 hessian = weighted_matrix.transpose() * weighted_matrix + regularization *
    Matrix88::Identity();
  const Vector8e gradient_offset = weighted_matrix.transpose() * weighted_desired;
  Vector8e thrust = hessian.ldlt().solve(gradient_offset);

  for (std::size_t thruster = 0; thruster < kThrusters; ++thruster) {
    thrust(thruster) =
      std::clamp(thrust(thruster), -geometry_[thruster].max_reverse_n,
        geometry_[thruster].max_forward_n);
  }

  double lipschitz = 0.0;
  for (std::size_t row = 0; row < kThrusters; ++row) {
    lipschitz = std::max(lipschitz, hessian.row(row).cwiseAbs().sum());
  }
  const double step = 1.0 / std::max(lipschitz, 1e-9);
  for (int iteration = 0; iteration < 256; ++iteration) {
    const Vector8e gradient = hessian * thrust - gradient_offset;
    thrust -= step * gradient;
    for (std::size_t thruster = 0; thruster < kThrusters; ++thruster) {
      thrust(thruster) =
        std::clamp(thrust(thruster), -geometry_[thruster].max_reverse_n,
          geometry_[thruster].max_forward_n);
    }
  }

  AllocationResult result;
  for (std::size_t thruster = 0; thruster < kThrusters; ++thruster) {
    result.thrust_n[thruster] = thrust(thruster);
    result.saturated[thruster] = thrust(thruster) >= geometry_[thruster].max_forward_n - 1e-6 ||
      thrust(thruster) <= -geometry_[thruster].max_reverse_n + 1e-6;
  }
  result.achieved_wrench = wrench_from_thrust(result.thrust_n);
  for (std::size_t axis = 0; axis < kDof; ++axis) {
    result.residual_wrench[axis] = desired_wrench[axis] - result.achieved_wrench[axis];
  }
  return result;
}

std::array<ThrusterGeometry, kThrusters> marlin_v2_thruster_geometry(
  double max_forward_n,
  double max_reverse_n)
{
    // Position and positive-command direction in base_link FLU. These columns
    // reproduce sub_control_mcu's FLU sign matrix while retaining the real
    // lever arms for roll, pitch, and yaw allocation.
  return {{
    {{{0.22, 0.23, 0.0}}, {{0.0, 0.0, -1.0}}, max_forward_n, max_reverse_n},
    {{{0.22, -0.23, 0.0}}, {{0.0, 0.0, 1.0}}, max_forward_n, max_reverse_n},
    {{{-0.22, 0.23, 0.0}}, {{0.0, 0.0, 1.0}}, max_forward_n, max_reverse_n},
    {{{-0.22, -0.23, 0.0}}, {{0.0, 0.0, -1.0}}, max_forward_n, max_reverse_n},
    {{{0.315, 0.285, 0.0}}, {{kSqrtHalf, -kSqrtHalf, 0.0}}, max_forward_n, max_reverse_n},
    {{{0.315, -0.285, 0.0}}, {{-kSqrtHalf, -kSqrtHalf, 0.0}}, max_forward_n, max_reverse_n},
    {{{-0.315, 0.285, 0.0}}, {{-kSqrtHalf, -kSqrtHalf, 0.0}}, max_forward_n, max_reverse_n},
    {{{-0.315, -0.285, 0.0}}, {{kSqrtHalf, -kSqrtHalf, 0.0}}, max_forward_n, max_reverse_n},
  }};
}

Vector6 DiagonalMarineModel::feedforward(
  const Vector6 & target_velocity, const Vector6 & target_acceleration,
  const Vector6 & configuration) const
{
  Vector6 wrench{};
  for (std::size_t axis = 0; axis < kDof; ++axis) {
    const std::array<double, 6> values = {effective_mass[axis], linear_drag[axis],
      quadratic_drag[axis],
      trim[axis], target_velocity[axis], target_acceleration[axis]};
    if (!std::all_of(values.begin(), values.end(), finite)) {
      throw std::invalid_argument("feedforward model and reference values must be finite");
    }
    if (!finite(restoring_stiffness[axis]) || !finite(configuration[axis])) {
      throw std::invalid_argument("restoring model and configuration must be finite");
    }
    wrench[axis] = effective_mass[axis] * target_acceleration[axis] + linear_drag[axis] *
      target_velocity[axis] +
      quadratic_drag[axis] * std::abs(target_velocity[axis]) * target_velocity[axis] + trim[axis] +
      restoring_stiffness[axis] * std::sin(configuration[axis]);
  }
  return wrench;
}

FeedbackPI6D::FeedbackPI6D(const Vector6 & kp, const Vector6 & ki, const Vector6 & integral_limit)
{
  configure(kp, ki, integral_limit);
}

void FeedbackPI6D::configure(const Vector6 & kp, const Vector6 & ki, const Vector6 & integral_limit)
{
  for (std::size_t axis = 0; axis < kDof; ++axis) {
    if (!finite(kp[axis]) || !finite(ki[axis]) || !finite(integral_limit[axis]) || kp[axis] < 0.0 ||
      ki[axis] < 0.0 || integral_limit[axis] < 0.0)
    {
      throw std::invalid_argument("PI gains and integral limits must be finite and nonnegative");
    }
  }
  kp_ = kp;
  ki_ = ki;
  integral_limit_ = integral_limit;
  reset();
}

FeedbackResult FeedbackPI6D::update(
  const Vector6 & error, const Vector6 & feedforward,
  const Vector6 & positive_total_limit, const Vector6 & negative_total_limit,
  double dt)
{
  if (!finite(dt) || dt <= 0.0) {
    throw std::invalid_argument("PI update dt must be finite and positive");
  }

  FeedbackResult result;
  for (std::size_t axis = 0; axis < kDof; ++axis) {
    if (!finite(error[axis]) || !finite(feedforward[axis]) || !finite(positive_total_limit[axis]) ||
      !finite(negative_total_limit[axis]) || positive_total_limit[axis] <= 0.0 ||
      negative_total_limit[axis] <= 0.0)
    {
      throw std::invalid_argument("PI inputs and total limits must be finite");
    }

    const double candidate_integral =
      std::clamp(integral_[axis] + error[axis] * dt, -integral_limit_[axis], integral_limit_[axis]);
    const double candidate_feedback = kp_[axis] * error[axis] + ki_[axis] * candidate_integral;
    const double candidate_total = feedforward[axis] + candidate_feedback;
    const bool saturated_with_error = (candidate_total > positive_total_limit[axis] &&
      error[axis] > 0.0) ||
      (candidate_total < -negative_total_limit[axis] && error[axis] < 0.0);
    if (!saturated_with_error) {
      integral_[axis] = candidate_integral;
    }

    const double lower_feedback = -negative_total_limit[axis] - feedforward[axis];
    const double upper_feedback = positive_total_limit[axis] - feedforward[axis];
    result.effort[axis] =
      std::clamp(kp_[axis] * error[axis] + ki_[axis] * integral_[axis], lower_feedback,
        upper_feedback);
    result.integral[axis] = integral_[axis];
  }
  return result;
}

void FeedbackPI6D::apply_allocation_residual(
  const Vector6 & residual_wrench, double dt,
  double back_calculation_gain)
{
  if (!finite(dt) || dt <= 0.0 || !finite(back_calculation_gain) || back_calculation_gain < 0.0) {
    throw std::invalid_argument("invalid anti-windup back-calculation settings");
  }
  for (std::size_t axis = 0; axis < kDof; ++axis) {
    if (!finite(residual_wrench[axis])) {
      throw std::invalid_argument("allocation residual must be finite");
    }
    if (ki_[axis] > 1e-9) {
      integral_[axis] -= back_calculation_gain * residual_wrench[axis] * dt / ki_[axis];
      integral_[axis] = std::clamp(integral_[axis], -integral_limit_[axis], integral_limit_[axis]);
    }
  }
}

void FeedbackPI6D::reset() {integral_.fill(0.0);}

VelocityReferenceLimiter::VelocityReferenceLimiter(const Vector6 & max_acceleration)
{
  set_max_acceleration(max_acceleration);
}

void VelocityReferenceLimiter::set_max_acceleration(const Vector6 & max_acceleration)
{
  for (double value : max_acceleration) {
    if (!finite(value) || value <= 0.0) {
      throw std::invalid_argument("maximum accelerations must be finite and positive");
    }
  }
  max_acceleration_ = max_acceleration;
}

MotionReference VelocityReferenceLimiter::update(const Vector6 & target_velocity, double dt)
{
  if (!finite(dt) || dt <= 0.0) {
    throw std::invalid_argument("reference update dt must be finite and positive");
  }

  MotionReference result;
  for (std::size_t axis = 0; axis < kDof; ++axis) {
    if (!finite(target_velocity[axis])) {
      throw std::invalid_argument("target velocities must be finite");
    }
    const double previous = velocity_[axis];
    const double maximum_delta = max_acceleration_[axis] * dt;
    velocity_[axis] += std::clamp(target_velocity[axis] - velocity_[axis], -maximum_delta,
        maximum_delta);
    result.velocity[axis] = velocity_[axis];
    result.acceleration[axis] = (velocity_[axis] - previous) / dt;
  }
  return result;
}

void VelocityReferenceLimiter::reset(const Vector6 & velocity)
{
  if (!std::all_of(velocity.begin(), velocity.end(), finite)) {
    throw std::invalid_argument("reset velocity must be finite");
  }
  velocity_ = velocity;
}

}  // namespace sub_control::feedforward
