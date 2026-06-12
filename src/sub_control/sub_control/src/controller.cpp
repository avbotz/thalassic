#include "sub_control/controller.hpp"

#include <algorithm>
#include <cmath>

StateFeedbackController::StateFeedbackController(const ControllerConfig& config) { set_config(config); }

ControlOutput StateFeedbackController::update(const ControlInput& input, double dt) {
    ControlOutput output;
    dt = std::clamp(dt, 1e-3, 0.2);

    for (int axis = 0; axis < CONTROL_AXES; ++axis) {
        const double target_velocity =
            input.position_control
                ? update_guidance(input.position_error[axis], config_.position_gain[axis],
                                  config_.position_integral_gain[axis], config_.position_integral_limit[axis],
                                  config_.max_velocity[axis], dt, position_integral_[axis])
                : std::clamp(input.velocity_command[axis], -config_.max_velocity[axis], config_.max_velocity[axis]);
        if (!input.position_control) {
            position_integral_[axis] = 0.0;
        }
        velocity_setpoint_[axis] =
            slew(velocity_setpoint_[axis], target_velocity, config_.max_linear_acceleration[axis], dt);

        const double target_rate =
            input.attitude_control
                ? smooth_limit(config_.attitude_gain[axis] * input.attitude_error[axis],
                               config_.max_angular_rate[axis])
                : std::clamp(input.angular_rate_command[axis], -config_.max_angular_rate[axis],
                             config_.max_angular_rate[axis]);
        angular_rate_setpoint_[axis] =
            slew(angular_rate_setpoint_[axis], target_rate, config_.max_angular_acceleration[axis], dt);

        output.velocity_setpoint[axis] = velocity_setpoint_[axis];
        output.angular_rate_setpoint[axis] = angular_rate_setpoint_[axis];
        output.velocity_error[axis] = velocity_setpoint_[axis] - input.velocity[axis];
        output.angular_rate_error[axis] = angular_rate_setpoint_[axis] - input.angular_rate[axis];

        output.wrench[axis] =
            update_pi(output.velocity_error[axis], config_.velocity_kp[axis], config_.velocity_ki[axis],
                      config_.velocity_integral_limit[axis], config_.max_force[axis], dt, velocity_integral_[axis]);
        output.wrench[3 + axis] =
            update_pi(output.angular_rate_error[axis], config_.angular_rate_kp[axis],
                      config_.angular_rate_ki[axis], config_.angular_integral_limit[axis], config_.max_torque[axis], dt,
                      angular_integral_[axis]);
    }

    return output;
}

void StateFeedbackController::reset() {
    velocity_integral_.fill(0.0);
    angular_integral_.fill(0.0);
    position_integral_.fill(0.0);
    velocity_setpoint_.fill(0.0);
    angular_rate_setpoint_.fill(0.0);
}

void StateFeedbackController::set_config(const ControllerConfig& config) {
    config_ = config;
    for (int axis = 0; axis < CONTROL_AXES; ++axis) {
        config_.position_gain[axis] = std::max(config_.position_gain[axis], 0.0);
        config_.position_integral_gain[axis] = std::max(config_.position_integral_gain[axis], 0.0);
        config_.attitude_gain[axis] = std::max(config_.attitude_gain[axis], 0.0);
        config_.velocity_kp[axis] = std::max(config_.velocity_kp[axis], 0.0);
        config_.velocity_ki[axis] = std::max(config_.velocity_ki[axis], 0.0);
        config_.angular_rate_kp[axis] = std::max(config_.angular_rate_kp[axis], 0.0);
        config_.angular_rate_ki[axis] = std::max(config_.angular_rate_ki[axis], 0.0);
        config_.max_velocity[axis] = std::fabs(config_.max_velocity[axis]);
        config_.max_angular_rate[axis] = std::fabs(config_.max_angular_rate[axis]);
        config_.max_linear_acceleration[axis] = std::fabs(config_.max_linear_acceleration[axis]);
        config_.max_angular_acceleration[axis] = std::fabs(config_.max_angular_acceleration[axis]);
        config_.velocity_integral_limit[axis] = std::fabs(config_.velocity_integral_limit[axis]);
        config_.angular_integral_limit[axis] = std::fabs(config_.angular_integral_limit[axis]);
        config_.position_integral_limit[axis] = std::fabs(config_.position_integral_limit[axis]);
        config_.max_force[axis] = std::fabs(config_.max_force[axis]);
        config_.max_torque[axis] = std::fabs(config_.max_torque[axis]);
    }
    reset();
}

double StateFeedbackController::smooth_limit(double value, double limit) {
    if (limit <= 0.0) {
        return 0.0;
    }
    return limit * std::tanh(value / limit);
}

double StateFeedbackController::slew(double current, double target, double rate_limit, double dt) {
    if (rate_limit <= 0.0) {
        return target;
    }
    const double max_step = rate_limit * dt;
    return current + std::clamp(target - current, -max_step, max_step);
}

double StateFeedbackController::update_pi(double error, double kp, double ki, double integral_limit,
                                          double output_limit, double dt, double& integral) {
    const double candidate_integral = std::clamp(integral + error * dt, -integral_limit, integral_limit);
    const double candidate_output = kp * error + ki * candidate_integral;
    const double output = std::clamp(candidate_output, -output_limit, output_limit);

    const bool saturated_high = candidate_output > output_limit && error > 0.0;
    const bool saturated_low = candidate_output < -output_limit && error < 0.0;
    if (!saturated_high && !saturated_low) {
        integral = candidate_integral;
    }

    return output;
}

double StateFeedbackController::update_guidance(double error, double kp, double ki, double integral_limit,
                                                double output_limit, double dt, double& integral) {
    const double candidate_integral = std::clamp(integral + error * dt, -integral_limit, integral_limit);
    const double raw_target = kp * error + ki * candidate_integral;
    const double target = smooth_limit(raw_target, output_limit);

    // Do not accumulate farther into saturation, but allow the integral to
    // unwind immediately when the position error reverses.
    const bool saturated = std::fabs(raw_target) >= output_limit;
    if (!saturated || raw_target * error < 0.0) {
        integral = candidate_integral;
    }
    return target;
}
