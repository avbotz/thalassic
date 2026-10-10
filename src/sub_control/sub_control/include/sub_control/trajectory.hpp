#ifndef SUB_CONTROL_TRAJECTORY_HPP_
#define SUB_CONTROL_TRAJECTORY_HPP_

#include <Eigen/Dense>

#include <algorithm>

// Jerk-limited kinematic shaping: turns a position or velocity target that may
// jump into a smooth reference (position, velocity, acceleration) the vehicle
// can actually follow, so the controller gets velocity and acceleration to feed
// forward and the thrusters are not asked for a step. Online, so the target can
// change at any time (visual servoing moves it every frame). The shaping laws are
// ArduPilot's (AP_Math/control.cpp): a square-root position controller that
// brakes at half the acceleration limit, then a jerk-limited acceleration slew.
// One change: near the target ArduPilot's gains make the approach a damping
// ratio of 0.7, which overshoots a few percent; here the position gain is a
// quarter of the velocity gain, which makes it critically damped.

struct ShaperLimits {
    double max_velocity{0.5};
    double max_acceleration{0.25};
    double max_jerk{0.5};
};

// Velocity that closes `error` braking at `accel`: proportional (gain p) near
// zero, square-root (constant deceleration) far from it, and never more than the
// error can absorb in one step of dt.
double sqrt_controller(double error, double p, double accel, double dt);

// Position, velocity and acceleration of an N-dimensional reference. The limits
// apply to the vector's length, so a diagonal move travels in a straight line.
template <int N>
class KinematicShaper {
   public:
    using Vector = Eigen::Matrix<double, N, 1>;

    void reset(const Vector& position, const Vector& velocity = Vector::Zero(),
               const Vector& acceleration = Vector::Zero()) {
        position_ = position;
        velocity_ = velocity;
        acceleration_ = acceleration;
    }

    // Advance by dt toward `target`, arriving at rest.
    void update_position(const Vector& target, const ShaperLimits& limits, double dt) {
        const double accel = std::max(limits.max_acceleration, 1e-6);
        Vector velocity = scaled_sqrt(target - position_, position_gain(limits), 0.5 * accel, dt);
        limit_length(velocity, limits.max_velocity);
        shape_velocity(velocity, limits, dt);
    }

    // The target update_position() would ask for the current velocity at: where
    // to latch a hold so that switching to it neither brakes nor pulls back.
    Vector stopping_point(const ShaperLimits& limits) const { return position_ + velocity_ / position_gain(limits); }

    // Advance by dt toward moving at `target` (position keeps integrating).
    void update_velocity(Vector target, const ShaperLimits& limits, double dt) {
        limit_length(target, limits.max_velocity);
        shape_velocity(target, limits, dt);
    }

    // Moves the reference without disturbing its motion (used to keep it on a
    // leash near the vehicle, or to rebase it after the estimate jumps).
    void set_position(const Vector& position) { position_ = position; }

    bool stopped(double velocity_tolerance, double acceleration_tolerance) const {
        return velocity_.norm() <= velocity_tolerance && acceleration_.norm() <= acceleration_tolerance;
    }

    const Vector& position() const { return position_; }
    const Vector& velocity() const { return velocity_; }
    const Vector& acceleration() const { return acceleration_; }

   private:
    // Velocity gain (jerk / acceleration, as ArduPilot) and a quarter of it for
    // position, so the linear region near the target is critically damped.
    static double velocity_gain(const ShaperLimits& limits) {
        return limits.max_jerk / std::max(limits.max_acceleration, 1e-6);
    }
    static double position_gain(const ShaperLimits& limits) { return 0.25 * velocity_gain(limits); }

    static void limit_length(Vector& v, double max_length) {
        const double length = v.norm();
        if (max_length >= 0.0 && length > max_length) {
            v *= max_length / length;
        }
    }

    static Vector scaled_sqrt(const Vector& error, double p, double accel, double dt) {
        const double length = error.norm();
        if (length <= 0.0) {
            return Vector::Zero();
        }
        return error * (sqrt_controller(length, p, accel, dt) / length);
    }

    void shape_velocity(const Vector& velocity_target, const ShaperLimits& limits, double dt) {
        const double accel = std::max(limits.max_acceleration, 1e-6);
        Vector accel_target = scaled_sqrt(velocity_target - velocity_, velocity_gain(limits), limits.max_jerk, dt);
        limit_length(accel_target, accel);

        Vector delta = accel_target - acceleration_;
        limit_length(delta, limits.max_jerk * dt);
        acceleration_ += delta;

        position_ += velocity_ * dt + 0.5 * acceleration_ * dt * dt;
        velocity_ += acceleration_ * dt;
    }

    Vector position_{Vector::Zero()};
    Vector velocity_{Vector::Zero()};
    Vector acceleration_{Vector::Zero()};
};

#endif  // SUB_CONTROL_TRAJECTORY_HPP_
