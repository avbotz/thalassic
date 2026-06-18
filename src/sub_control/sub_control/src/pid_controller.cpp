#include "sub_control/pid_controller.hpp"
#include <algorithm>
#include <cmath>
#include <span>

PID_Controller::PID_Controller(double kp, double ki, double kd)
    : kp_{kp}, ki_{ki}, kd_{kd}, output_limit_{0.0} {}

PID_Controller::PID_Controller(double kp, double ki, double kd, double output_limit)
    : kp_{kp}, ki_{ki}, kd_{kd}, output_limit_{output_limit} {}

PID_Controller::PID_Controller(std::span<const double> params, bool smooth_saturation)
    : smooth_saturation_{smooth_saturation} {
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


double PID_Controller::update(double measurement, double error, double dt) {
    dt = std::max(0.0, dt);

    integral_ += error * dt;

    // Derivative on measurement (d(-PV) / dt) to avoid derivative kick
    // Derivative term is smoothed to avoid sudden changes in output
    if (dt > 0.0) {
        smoothed_derivative_ = ALPHA * -(measurement - prev_measurement_) / dt + (1 - ALPHA) * smoothed_derivative_;
    }
    prev_measurement_ = measurement;

    // Independent PID (kp_ does not affect other two terms)
    const double raw = kp_ * error + ki_ * integral_ + kd_ * smoothed_derivative_;
    if (output_limit_ <= 0.0) {
        return raw;
    }

    // Soft (tanh) saturation eases the output up to output_limit_ so the inner
    // loop chases a continuous setpoint instead of a hard corner; the inner loops
    // keep the hard clamp. Either way back-calculation below bleeds (raw - output)
    // off the integral to stop windup.
    const double clamped = smooth_saturation_ ? output_limit_ * std::tanh(raw / output_limit_)
                                              : std::clamp(raw, -output_limit_, output_limit_);

    if (raw != clamped) {
        integral_ -= raw - clamped;
    }

    return clamped;
}

void PID_Controller::reset() {
    integral_ = 0.0;
    prev_measurement_ = 0.0;
    smoothed_derivative_ = 0.0;
}
