#include "sub_control/PID.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

PID::PID()
    : kp_(0.0),
      ki_(0.0),
      kd_(0.0),
      tau_d_(0.0),
      out_min_(-std::numeric_limits<double>::infinity()),
      out_max_(std::numeric_limits<double>::infinity()),
      integral_(0.0),
      deriv_filt_(0.0),
      prev_meas_(0.0),
      have_prev_(false) {}

PID::PID(double kp, double ki, double kd, double tau_d, double out_min, double out_max)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      tau_d_(tau_d),
      out_min_(out_min),
      out_max_(out_max),
      integral_(0.0),
      deriv_filt_(0.0),
      prev_meas_(0.0),
      have_prev_(false) {}

double PID::update(double error, double measurement, double dt) {
    if (dt <= 0.0) {
        return std::clamp(kp_ * error, out_min_, out_max_);
    }

    const double p = kp_ * error;

    integral_ += error * dt;
    const double i = ki_ * integral_;

    // Derivative on measurement, low-pass filtered
    double d = 0.0;
    if (have_prev_) {
        const double d_raw = -(measurement - prev_meas_) / dt;
        const double alpha = (tau_d_ > 0.0) ? dt / (tau_d_ + dt) : 1.0;
        deriv_filt_ += alpha * (d_raw - deriv_filt_);
        d = kd_ * deriv_filt_;
    }
    prev_meas_ = measurement;
    have_prev_ = true;

    const double u_unsat = p + i + d;
    const double u = std::clamp(u_unsat, out_min_, out_max_);

    if (ki_ != 0.0 && u != u_unsat) {
        integral_ += (u - u_unsat) / ki_;
    }

    return u;
}

void PID::reset() {
    integral_ = 0.0;
    deriv_filt_ = 0.0;
    have_prev_ = false;
}

void PID::set_gains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::set_limits(double out_min, double out_max) {
    out_min_ = out_min;
    out_max_ = out_max;
}
