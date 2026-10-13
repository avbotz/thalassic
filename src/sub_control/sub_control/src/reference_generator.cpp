#include "sub_control/reference_generator.hpp"

#include <cmath>

namespace {

// A HOLD latches its position once the reference is this still.
constexpr double STOPPED_VELOCITY = 0.02;      // [m/s] or [rad/s]
constexpr double STOPPED_ACCELERATION = 0.05;  // [m/s^2] or [rad/s^2]

template <int N>
void leash(KinematicShaper<N>& shaper, const Eigen::Matrix<double, N, 1>& measured, double length) {
    const Eigen::Matrix<double, N, 1> offset = shaper.position() - measured;
    const double distance = offset.norm();
    if (length > 0.0 && distance > length) {
        shaper.set_position(measured + offset * (length / distance));
    }
}

Eigen::Matrix<double, 1, 1> scalar(double value) { return Eigen::Matrix<double, 1, 1>(value); }

}  // namespace

void ReferenceGenerator::reset(const Vector6d& pose) {
    horizontal_.reset(pose.head<2>());
    vertical_.reset(scalar(pose[Z]));
    for (int i = 0; i < 3; ++i) {
        rotation_[i].reset(scalar(pose[ROLL + i]));
    }
    mode_.fill(AxisMode::POSITION);
    position_target_ = pose;
    position_target_[ROLL] = 0.0;
    position_target_[PITCH] = 0.0;
    velocity_target_.setZero();
    expires_.fill(0.0);
    altitude_ = false;
}

double ReferenceGenerator::continuous_angle(int axis, double angle) const {
    const double reference = rotation_[axis - ROLL].position()(0);
    return reference + wrap_angle(angle - reference);
}

std::string ReferenceGenerator::apply(const MotionCommand& command, const Vector6d& pose, double altitude) {
    if (command.mode[X] != command.mode[Y]) {
        return "x and y must share a mode";
    }
    const bool z_position = command.mode[Z] == AxisMode::POSITION;
    if (z_position && command.altitude && !std::isfinite(altitude)) {
        return "altitude hold needs a DVL altitude, and there is none";
    }
    if (z_position && command.altitude && command.position[Z] < 0.0) {
        return "altitude target is below the bottom";
    }
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        if ((command.mode[axis] == AxisMode::POSITION && !std::isfinite(command.position[axis])) ||
            (command.mode[axis] == AxisMode::VELOCITY && !std::isfinite(command.velocity[axis])) ||
            (command.mode[axis] == AxisMode::EFFORT && !std::isfinite(command.effort[axis]))) {
            return "non-finite target";
        }
    }

    // Switching z between depth and altitude rebases it onto the new
    // measurement without disturbing its motion.
    if (z_position && command.altitude != altitude_) {
        vertical_.set_position(scalar(command.altitude ? altitude : pose[Z]));
        altitude_ = command.altitude;
    }

    for (int axis = 0; axis < NUM_DOF; ++axis) {
        switch (command.mode[axis]) {
            case AxisMode::KEEP:
                break;
            case AxisMode::POSITION:
                mode_[axis] = AxisMode::POSITION;
                position_target_[axis] =
                    axis >= ROLL ? continuous_angle(axis, command.position[axis]) : command.position[axis];
                break;
            case AxisMode::VELOCITY:
                mode_[axis] = AxisMode::VELOCITY;
                velocity_target_[axis] = command.velocity[axis];
                expires_[axis] = command.expires;
                break;
            case AxisMode::HOLD:
                start_hold(axis);
                break;
            case AxisMode::EFFORT:
                mode_[axis] = AxisMode::EFFORT;
                effort_[axis] = command.effort[axis];
                expires_[axis] = command.expires;
                break;
        }
    }
    if (command.mode[X] == AxisMode::VELOCITY) {
        velocity_frame_ = command.velocity_frame;
    }
    return "";
}

void ReferenceGenerator::start_hold(int axis) {
    // A held axis decelerates to rest, then holds where it stopped.
    mode_[axis] = AxisMode::HOLD;
    velocity_target_[axis] = 0.0;
    expires_[axis] = 0.0;
}

void ReferenceGenerator::set_passive(int axis, bool passive) { passive_[axis] = passive; }

void ReferenceGenerator::drop_altitude(double depth_z) {
    if (!altitude_) {
        return;
    }
    altitude_ = false;
    vertical_.set_position(scalar(depth_z));
    start_hold(Z);
}

void ReferenceGenerator::update(double now, double dt, const Vector6d& pose, const ReferenceLimits& limits) {
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        const bool streamed = mode_[axis] == AxisMode::VELOCITY || mode_[axis] == AxisMode::EFFORT;
        if (streamed && expires_[axis] > 0.0 && now > expires_[axis]) {
            start_hold(axis);
        }
    }
    const auto follows_vehicle = [this](int axis) { return passive_[axis] || mode_[axis] == AxisMode::EFFORT; };

    // x and y, shaped together.
    const Eigen::Vector2d horizontal_pose = pose.head<2>();
    if (follows_vehicle(X) || follows_vehicle(Y)) {
        horizontal_.reset(horizontal_pose);
    } else {
        if (mode_[X] == AxisMode::POSITION) {
            horizontal_.update_position(position_target_.head<2>(), limits.horizontal, dt);
        } else {
            Eigen::Vector2d velocity = velocity_target_.head<2>();
            if (mode_[X] == AxisMode::VELOCITY && velocity_frame_ == VelocityFrame::HEADING) {
                velocity = Eigen::Rotation2Dd(rotation_[YAW - ROLL].position()(0)) * velocity;
            }
            horizontal_.update_velocity(velocity, limits.horizontal, dt);
            if (mode_[X] == AxisMode::HOLD && horizontal_.stopped(STOPPED_VELOCITY, STOPPED_ACCELERATION)) {
                mode_[X] = mode_[Y] = AxisMode::POSITION;
                position_target_.head<2>() = horizontal_.stopping_point(limits.horizontal);
            }
        }
        leash(horizontal_, horizontal_pose, limits.translation_leash);
    }

    // z, and each rotation on its own.
    const auto update_scalar = [&](KinematicShaper<1>& shaper, int axis, double measured, const ShaperLimits& limit,
                                   double leash_length) {
        if (follows_vehicle(axis)) {
            shaper.reset(scalar(measured));
            return;
        }
        if (mode_[axis] == AxisMode::POSITION) {
            shaper.update_position(scalar(position_target_[axis]), limit, dt);
        } else {
            shaper.update_velocity(scalar(velocity_target_[axis]), limit, dt);
            if (mode_[axis] == AxisMode::HOLD && shaper.stopped(STOPPED_VELOCITY, STOPPED_ACCELERATION)) {
                mode_[axis] = AxisMode::POSITION;
                position_target_[axis] = shaper.stopping_point(limit)(0);
            }
        }
        leash(shaper, scalar(measured), leash_length);
    };

    update_scalar(vertical_, Z, pose[Z], limits.vertical, limits.translation_leash);
    const std::array<const ShaperLimits*, 3> rotation_limits{&limits.roll, &limits.pitch, &limits.yaw};
    for (int i = 0; i < 3; ++i) {
        const int axis = ROLL + i;
        update_scalar(rotation_[i], axis, continuous_angle(axis, pose[axis]), *rotation_limits[i],
                      limits.rotation_leash);
    }
}

Vector6d ReferenceGenerator::target() const {
    Vector6d target = position();
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        if (mode_[axis] == AxisMode::POSITION) {
            target[axis] = position_target_[axis];
        }
    }
    return target;
}

Vector6d ReferenceGenerator::position() const {
    Vector6d v;
    v << horizontal_.position(), vertical_.position(), rotation_[0].position(), rotation_[1].position(),
        rotation_[2].position();
    return v;
}

Vector6d ReferenceGenerator::velocity() const {
    Vector6d v;
    v << horizontal_.velocity(), vertical_.velocity(), rotation_[0].velocity(), rotation_[1].velocity(),
        rotation_[2].velocity();
    return v;
}

Vector6d ReferenceGenerator::acceleration() const {
    Vector6d v;
    v << horizontal_.acceleration(), vertical_.acceleration(), rotation_[0].acceleration(), rotation_[1].acceleration(),
        rotation_[2].acceleration();
    return v;
}
