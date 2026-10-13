#ifndef SUB_CONTROL_CONTROLLER_HPP_
#define SUB_CONTROL_CONTROLLER_HPP_

#include "sub_control/utils.hpp"

#include <array>

// Low-speed 6-DOF model of the vehicle (Fossen, with diagonal mass and drag and
// moments about the centre of gravity), in body FLU. All of it is
// feedforward: what it gets wrong shows up as a disturbance that the feedback,
// the integrators and the disturbance observer take out.
struct VehicleModel {
    Eigen::Vector3d mass{30.0, 30.0, 30.0};                      // rigid-body plus added mass per body axis [kg]
    Eigen::Vector3d inertia{1.0, 1.0, 1.0};                      // rigid-body plus added inertia per body axis [kg m^2]
    double net_buoyancy{0.0};                                    // buoyancy minus weight [N]; positive floats
    double buoyancy{0.0};                                        // buoyancy alone [N], for its righting moment
    Eigen::Vector3d center_of_gravity{Eigen::Vector3d::Zero()};  // base_link [m]
    Eigen::Vector3d center_of_buoyancy{Eigen::Vector3d::Zero()};  // base_link [m]
    Vector6d linear_drag{Vector6d::Zero()};                       // [N/(m/s)], [N m/(rad/s)]
    Vector6d quadratic_drag{Vector6d::Zero()};                    // [N/(m/s)^2], [N m/(rad/s)^2]

    // Wrench [N, N m] holding the vehicle against net buoyancy (at attitude q)
    // and the buoyancy's righting moment (at attitude q_hold).
    Vector6d restoring_hold(const Eigen::Quaterniond& q, const Eigen::Quaterniond& q_hold) const;
    // Restoring wrench acting on the vehicle at attitude q (the negative of the
    // hold, both at q).
    Vector6d restoring(const Eigen::Quaterniond& q) const;
    // Drag wrench resisting body velocity nu.
    Vector6d drag(const Vector6d& nu) const;
    Vector6d inertia_vector() const;
};

// Per-axis feedback gains, normalized by the model's mass/inertia, so they are
// accelerations: a = kp * position error + ki * integral + kd * velocity error.
// A critically damped triple pole at -w is kp = 3w^2, ki = w^3, kd = 3w.
struct ControlGains {
    Vector6d kp{Vector6d::Zero()};                                                     // [1/s^2]
    Vector6d ki{Vector6d::Zero()};                                                     // [1/s^3]
    Vector6d kd{Vector6d::Zero()};                                                     // [1/s]
    Vector6d integral_limit{(Vector6d() << 1.0, 1.0, 1.0, 6.0, 6.0, 6.0).finished()};  // [m/s^2], [rad/s^2]
    double anti_windup{5.0};                                                           // [1/s] back-calculation rate
    Vector6d observer_bandwidth{Vector6d::Zero()};  // [rad/s]; 0 turns the observer off on that axis
    // Fraction of each feedforward term applied: reference acceleration, drag
    // at the reference velocity, and the restoring (buoyancy) hold.
    double acceleration_feedforward{1.0};
    double drag_feedforward{1.0};
    double restoring_feedforward{1.0};
};

// Measured state: odom ENU pose, body FLU velocity (at base_link).
struct VehicleState {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

// Trajectory reference: odom ENU translation, ZYX Euler angles and their rates.
struct ControlReference {
    Vector6d position{Vector6d::Zero()};
    Vector6d velocity{Vector6d::Zero()};
    Vector6d acceleration{Vector6d::Zero()};
};

// Bumblebee-style control law partitioning (Craig, "Introduction to Robotics",
// ch. 9; Fossen 2021, ch. 15):
//
//   tau = M a* + D(nu_ref) nu_ref + g_hold(q, q_ref) + w x J w - d_hat
//   a*  = a_ref + Kd (nu_ref - nu) + Kp e + I
//
// The model part linearizes and decouples the vehicle, so the servo part a* is
// the same unit-mass double integrator on every axis and one set of gains
// means the same response everywhere. e is the position error (world, rotated
// into the body) and the geodesic attitude error; everything is in body FLU.
// The integrators are saturation-aware: they bleed off whatever part of the
// command the thrusters could not produce (achieved()), so a saturated axis
// never winds up and a failed thruster does not leave a stale integral behind.
class MotionController {
   public:
    // Retuning keeps the integrators (accelerations, so the force they hold
    // scales with a new mass) and the disturbance estimate.
    void set_model(const VehicleModel& model);
    void set_gains(const ControlGains& gains);
    const VehicleModel& model() const { return model_; }

    // Zeroes the integrators and the disturbance observer.
    void reset();

    // Turns the world-frame integrators by `yaw` about z, for when the world
    // frame itself turns (a pose reset zeroes the yaw) and the forces they hold
    // do not.
    void rotate_world(double yaw);

    // The body wrench for this cycle. `active` axes have feedback; the others
    // only get feedforward and keep their integrators frozen (no estimate of
    // that axis, or no thrusters left to move it). `altitude` replaces the
    // measured z (positive up) when z tracks altitude.
    Vector6d update(const VehicleState& state, const ControlReference& reference,
                    const std::array<bool, NUM_DOF>& active, double dt, double altitude = 0.0,
                    bool use_altitude = false);

    // What the thrusters produced of the last update(). Feeds the integrators'
    // anti-windup and the disturbance observer.
    void achieved(const Vector6d& wrench, double dt);

    // Position/attitude and velocity/rate errors of the last update(), body FLU.
    const Vector6d& position_error() const { return position_error_; }
    const Vector6d& velocity_error() const { return velocity_error_; }
    // Integral action in body axes [m/s^2, rad/s^2].
    Vector6d integral() const;
    // Disturbance observer estimate, body [N, N m].
    const Vector6d& disturbance() const { return disturbance_; }

   private:
    void clamp_integrals();

    VehicleModel model_;
    ControlGains gains_;

    // Translational integral in world axes: buoyancy errors and currents are
    // fixed in the world, so a yaw turn must not leave them pointing the old way.
    Eigen::Vector3d integral_world_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d integral_body_rotation_{Eigen::Vector3d::Zero()};

    // Last update(), kept for achieved().
    Vector6d command_{Vector6d::Zero()};
    std::array<bool, NUM_DOF> active_{};
    Eigen::Matrix3d rotation_{Eigen::Matrix3d::Identity()};
    Vector6d position_error_{Vector6d::Zero()};
    Vector6d velocity_error_{Vector6d::Zero()};

    // Momentum disturbance observer: d_hat converges on the unmodelled force
    // with the observer bandwidth, using only measured velocity (no derivative).
    Vector6d disturbance_{Vector6d::Zero()};
    Vector6d observer_integral_{Vector6d::Zero()};
    Vector6d momentum_start_{Vector6d::Zero()};
    Vector6d momentum_{Vector6d::Zero()};
    Vector6d known_rate_{Vector6d::Zero()};  // model terms of the momentum rate this cycle, less thrust
    bool observer_started_{false};
};

#endif  // SUB_CONTROL_CONTROLLER_HPP_
