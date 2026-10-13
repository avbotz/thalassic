#include "sub_control/controller.hpp"

#include <algorithm>

Vector6d VehicleModel::inertia_vector() const {
    Vector6d m;
    m << mass, inertia;
    return m;
}

Vector6d VehicleModel::restoring(const Eigen::Quaterniond& q) const {
    // Weight acts at the centre of gravity, so only buoyancy has a moment about it.
    const Eigen::Matrix3d R = q.toRotationMatrix();
    Vector6d wrench;
    wrench.head<3>() = R.transpose() * Eigen::Vector3d(0.0, 0.0, net_buoyancy);
    wrench.tail<3>() =
        (center_of_buoyancy - center_of_gravity).cross(R.transpose() * Eigen::Vector3d(0.0, 0.0, buoyancy));
    return wrench;
}

Vector6d VehicleModel::restoring_hold(const Eigen::Quaterniond& q, const Eigen::Quaterniond& q_hold) const {
    // The force is held at the actual attitude (it is a fixed world-vertical
    // push); the righting moment at the reference attitude, so the hull's own
    // stiffness still pulls toward the reference rather than being cancelled.
    Vector6d hold;
    hold.head<3>() = -restoring(q).head<3>();
    hold.tail<3>() = -restoring(q_hold).tail<3>();
    return hold;
}

Vector6d VehicleModel::drag(const Vector6d& nu) const {
    return linear_drag.cwiseProduct(nu) + quadratic_drag.cwiseProduct(nu.cwiseAbs()).cwiseProduct(nu);
}

void MotionController::clamp_integrals() {
    for (int i = 0; i < 3; ++i) {
        integral_world_[i] = std::clamp(integral_world_[i], -gains_.integral_limit[i], gains_.integral_limit[i]);
        integral_body_rotation_[i] =
            std::clamp(integral_body_rotation_[i], -gains_.integral_limit[3 + i], gains_.integral_limit[3 + i]);
    }
}

void MotionController::set_model(const VehicleModel& model) {
    // The observer's estimate is K (M nu - p0 - integral), with p0 measured with
    // the old M: shift p0 with M nu at the last velocity, or a new mass would
    // step the estimate by K (M_new - M_old) nu.
    if (observer_started_) {
        const Vector6d nu = momentum_.cwiseQuotient(model_.inertia_vector());
        const Vector6d shift = (model.inertia_vector() - model_.inertia_vector()).cwiseProduct(nu);
        momentum_ += shift;
        momentum_start_ += shift;
    }
    model_ = model;
}

void MotionController::set_gains(const ControlGains& gains) {
    // The observer's state is d_hat / K, so a new bandwidth alone would scale
    // the estimate (and step the wrench) by K_new / K_old. Rescale the state
    // to keep d_hat; the integrators store accelerations and need nothing.
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        const double from = gains_.observer_bandwidth[axis];
        const double to = gains.observer_bandwidth[axis];
        if (from > 0.0 && to > 0.0 && from != to) {
            const double residual = momentum_[axis] - momentum_start_[axis] - observer_integral_[axis];
            observer_integral_[axis] = momentum_[axis] - momentum_start_[axis] - residual * (from / to);
        }
    }
    gains_ = gains;
}

void MotionController::reset() {
    integral_world_.setZero();
    integral_body_rotation_.setZero();
    disturbance_.setZero();
    observer_integral_.setZero();
    observer_started_ = false;
    command_.setZero();
}

void MotionController::rotate_world(double yaw) {
    integral_world_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * integral_world_;
    clamp_integrals();
}

Vector6d MotionController::integral() const {
    Vector6d integral;
    integral << rotation_.transpose() * integral_world_, integral_body_rotation_;
    return integral;
}

Vector6d MotionController::update(const VehicleState& state, const ControlReference& reference,
                                  const std::array<bool, NUM_DOF>& active, double dt, double altitude,
                                  bool use_altitude) {
    const Eigen::Quaterniond q = state.orientation.normalized();
    const Eigen::Matrix3d R = q.toRotationMatrix();
    rotation_ = R;
    active_ = active;

    // Translation: world error, rotated into the body.
    Eigen::Vector3d position = state.position;
    if (use_altitude) {
        position.z() = altitude;
    }
    const Eigen::Vector3d linear_velocity_ref = R.transpose() * reference.velocity.head<3>();
    const Eigen::Vector3d linear_acceleration_ref = R.transpose() * reference.acceleration.head<3>();
    position_error_.head<3>() = R.transpose() * (reference.position.head<3>() - position);
    velocity_error_.head<3>() = linear_velocity_ref - state.linear_velocity;

    // Rotation: geodesic error, and the reference's Euler rates as body rates.
    const Eigen::Vector3d rpy_ref = reference.position.tail<3>();
    const Eigen::Quaterniond q_ref = quaternion_from_rpy(rpy_ref);
    const Eigen::Matrix3d to_body = R.transpose() * q_ref.toRotationMatrix();
    const Eigen::Vector3d angular_velocity_ref = to_body * euler_rates_to_body(rpy_ref, reference.velocity.tail<3>());
    const Eigen::Vector3d angular_acceleration_ref =
        to_body * euler_rates_to_body(rpy_ref, reference.acceleration.tail<3>());
    position_error_.tail<3>() = attitude_error(q_ref, q);
    velocity_error_.tail<3>() = angular_velocity_ref - state.angular_velocity;

    Vector6d nu_ref;
    nu_ref << linear_velocity_ref, angular_velocity_ref;
    Vector6d acceleration_ref;
    acceleration_ref << linear_acceleration_ref, angular_acceleration_ref;

    // Servo law, in accelerations.
    const Vector6d integral_body = integral();
    Vector6d acceleration = gains_.acceleration_feedforward * acceleration_ref;
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        if (active[axis]) {
            acceleration[axis] +=
                gains_.kp[axis] * position_error_[axis] + gains_.kd[axis] * velocity_error_[axis] + integral_body[axis];
        }
    }

    // Integrate the position errors of the axes under feedback.
    Eigen::Vector3d translation_step = Eigen::Vector3d::Zero();
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        if (!active[axis]) {
            continue;
        }
        const double step = gains_.ki[axis] * position_error_[axis] * dt;
        if (axis < 3) {
            translation_step[axis] = step;
        } else {
            integral_body_rotation_[axis - 3] += step;
        }
    }
    integral_world_ += R * translation_step;
    clamp_integrals();

    // Model: inertia times the servo acceleration, plus what holds the vehicle
    // against drag and buoyancy along the reference.
    const Vector6d inertia = model_.inertia_vector();
    Vector6d wrench = inertia.cwiseProduct(acceleration) + gains_.drag_feedforward * model_.drag(nu_ref) +
                      gains_.restoring_feedforward * model_.restoring_hold(q, q_ref);
    const Eigen::Vector3d spin = inertia.tail<3>().cwiseProduct(state.angular_velocity);
    wrench.tail<3>() += state.angular_velocity.cross(spin);

    // Disturbance observer on the generalized momentum p = M nu:
    //   dp/dt = tau + restoring - drag - coriolis + d
    //   d_hat = K (p - p0 - integral(tau + restoring - drag - coriolis + d_hat))
    // so d_hat follows d through a first-order lag of bandwidth K.
    Vector6d nu;
    nu << state.linear_velocity, state.angular_velocity;
    momentum_ = inertia.cwiseProduct(nu);
    Vector6d coriolis;
    coriolis << state.angular_velocity.cross(momentum_.head<3>()), state.angular_velocity.cross(spin);
    known_rate_ = model_.restoring(q) - model_.drag(nu) - coriolis;
    if (!observer_started_) {
        momentum_start_ = momentum_;
        observer_integral_.setZero();
        disturbance_.setZero();
        observer_started_ = true;
    }
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        const double bandwidth = gains_.observer_bandwidth[axis];
        if (bandwidth <= 0.0) {
            disturbance_[axis] = 0.0;
        } else if (active[axis]) {
            disturbance_[axis] = bandwidth * (momentum_[axis] - momentum_start_[axis] - observer_integral_[axis]);
        }
        // An axis without feedback holds its estimate: rebase so it stays put.
        if (bandwidth <= 0.0 || !active[axis]) {
            observer_integral_[axis] =
                momentum_[axis] - momentum_start_[axis] - (bandwidth > 0.0 ? disturbance_[axis] / bandwidth : 0.0);
        }
    }
    wrench -= disturbance_;

    command_ = wrench;
    return wrench;
}

void MotionController::achieved(const Vector6d& wrench, double dt) {
    // Back-calculation: bleed off the part of the command that was not produced.
    const Vector6d shortfall = (wrench - command_).cwiseQuotient(model_.inertia_vector());
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        if (!active_[axis]) {
            continue;
        }
        const double step = gains_.anti_windup * shortfall[axis] * dt;
        if (axis < 3) {
            translation[axis] = step;
        } else {
            integral_body_rotation_[axis - 3] += step;
        }
    }
    integral_world_ += rotation_ * translation;
    clamp_integrals();

    observer_integral_ += (wrench + known_rate_ + disturbance_) * dt;
}
