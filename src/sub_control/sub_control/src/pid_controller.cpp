#include "sub_control/pid_controller.hpp"
#include <algorithm>
#include <cmath>
#include <span>

PID_Controller::PID_Controller(double kp, double ki, double kd)
    : kp_{kp}, ki_{ki}, kd_{kd}, output_limit_{0.0} {}

PID_Controller::PID_Controller(double kp, double ki, double kd, double output_limit)
    : kp_{kp}, ki_{ki}, kd_{kd}, output_limit_{output_limit} {}

PID_Controller::PID_Controller(std::span<const double> params) {
    if (params.size() == 3) {
        kp_ = params[0];
        ki_ = params[1];
        kd_ = params[2];
        output_limit_ = 0.0;
    } else if (params.size() == 4) {
        kp_ = params[0];
        ki_ = params[1];
        kd_ = params[2];
        output_limit_ = params[3];
    }
}

void PID_Controller::configure(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    output_limit_ = 0.0;

    reset();
}

void PID_Controller::configure(double kp, double ki, double kd, double output_limit) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    output_limit_ = output_limit;

    reset();
}

void PID_Controller::configure(std::span<const double> params) {
    if (params.size() == 3) {
        kp_ = params[0];
        ki_ = params[1];
        kd_ = params[2];
        output_limit_ = 0.0;
    } else if (params.size() == 4) {
        kp_ = params[0];
        ki_ = params[1];
        kd_ = params[2];
        output_limit_ = params[3];
    }

    reset();
}


double PID_Controller::update(double measurement, double error, double dt, double output_limit_override) {
    dt = std::max(0.0, dt);

    // Seed the measurement history after a reset so the derivative does not
    // see a spurious jump from 0 to the current measurement.
    if (!primed_) {
        prev_measurement_ = measurement;
        primed_ = true;
    }

    // Derivative on measurement (d(-PV) / dt) to avoid derivative kick
    // Derivative term is smoothed to avoid sudden changes in output
    if (dt > 0.0) {
        smoothed_derivative_ = ALPHA * -(measurement - prev_measurement_) / dt + (1 - ALPHA) * smoothed_derivative_;
    }
    prev_measurement_ = measurement;

    const double limit = output_limit_override > 0.0 ? output_limit_override : output_limit_;

    const double candidate_integral = integral_ + error * dt;
    double raw = kp_ * error + ki_ * candidate_integral + kd_ * smoothed_derivative_;
    if (limit <= 0.0) {
        integral_ = candidate_integral;
        return raw;
    }

    // Conditional-integration anti-windup: hold the integral while the output
    // is saturated in the error's direction, so a large setpoint step neither
    // winds the integral up nor (as one-shot back-calculation did) injects a
    // correction that outlives the saturation and starves the output after.
    if (std::fabs(raw) <= limit || (raw > 0.0) != (error > 0.0)) {
        integral_ = candidate_integral;
    } else {
        raw = kp_ * error + ki_ * integral_ + kd_ * smoothed_derivative_;
    }

    return std::clamp(raw, -limit, limit);
}

void PID_Controller::reset() {
    integral_ = 0.0;
    prev_measurement_ = 0.0;
    smoothed_derivative_ = 0.0;
    primed_ = false;
}
