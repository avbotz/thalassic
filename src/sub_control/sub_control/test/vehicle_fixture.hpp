#ifndef SUB_CONTROL_TEST_VEHICLE_FIXTURE_HPP_
#define SUB_CONTROL_TEST_VEHICLE_FIXTURE_HPP_

// Marlin's thruster layout and a small 6-DOF rigid-body plant for closed-loop
// tests. The plant is deliberately not the controller's model: the tests
// check that the loop holds up against the error a pool-identified model has.

#include "sub_control/allocator.hpp"
#include "sub_control/controller.hpp"
#include "sub_control/reference_generator.hpp"
#include "sub_control/utils.hpp"

#include <cmath>
#include <numbers>

// config/vehicles/marlin_v3.yaml, in base_link (FLU): 0-3 vertical (mounted +X
// down), 4-7 vectored horizontal; 1, 2, 5, 6 have left-hand propellers.
inline ThrusterGeometries marlin_thrusters() {
    const double s = std::numbers::sqrt2 / 2.0;
    ThrusterGeometries t;
    t[0] = {{0.22, 0.23, 0.0}, {0.0, 0.0, -1.0}, false};
    t[1] = {{0.22, -0.23, 0.0}, {0.0, 0.0, -1.0}, true};
    t[2] = {{-0.22, 0.23, 0.0}, {0.0, 0.0, -1.0}, true};
    t[3] = {{-0.22, -0.23, 0.0}, {0.0, 0.0, -1.0}, false};
    t[4] = {{0.315, 0.285, 0.08}, {s, -s, 0.0}, false};
    t[5] = {{0.315, -0.285, 0.08}, {s, s, 0.0}, true};
    t[6] = {{-0.315, 0.285, 0.08}, {s, s, 0.0}, true};
    t[7] = {{-0.315, -0.285, 0.08}, {s, -s, 0.0}, false};
    return t;
}

inline std::array<bool, NUM_THRUSTERS> marlin_reversed() {
    std::array<bool, NUM_THRUSTERS> reversed{};
    const ThrusterGeometries t = marlin_thrusters();
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        reversed[i] = t[i].reversed;
    }
    return reversed;
}

// The controller's idea of the vehicle.
inline VehicleModel nominal_model() {
    VehicleModel m;
    m.mass = {45.0, 55.0, 60.0};
    m.inertia = {1.2, 1.6, 1.4};
    m.net_buoyancy = 15.0;
    m.buoyancy = 400.0;
    m.center_of_gravity = {0.0, 0.0, 0.08};
    m.center_of_buoyancy = {0.0, 0.0, 0.10};
    m.linear_drag << 20, 30, 40, 4, 5, 5;
    m.quadratic_drag << 60, 80, 90, 2, 3, 3;
    return m;
}

inline ControlGains nominal_gains() {
    ControlGains g;
    const Vector6d w = (Vector6d() << 1.0, 1.0, 1.2, 2.5, 2.5, 2.0).finished();
    g.kp = 3.0 * w.cwiseProduct(w);
    g.kd = 3.0 * w;
    g.ki = w.cwiseProduct(w).cwiseProduct(w);
    return g;
}

// Rigid body with diagonal mass and drag, buoyancy and a disturbance force.
class Plant {
   public:
    VehicleModel truth{nominal_model()};
    Eigen::Vector3d center_of_gravity{0.0, 0.0, 0.06};
    Eigen::Vector3d external_force_world{Eigen::Vector3d::Zero()};
    ThrusterArray effectiveness{1, 1, 1, 1, 1, 1, 1, 1};
    VehicleState state;

    Plant() {
        // What a pool session gets wrong: heavier, draggier, trimmed differently.
        truth.mass *= 1.25;
        truth.inertia *= 1.4;
        truth.linear_drag *= 1.6;
        truth.quadratic_drag *= 0.7;
        truth.net_buoyancy = 22.0;
        truth.center_of_buoyancy = {0.004, -0.002, 0.11};  // trimmed to within a few degrees
        truth.center_of_gravity = center_of_gravity;
    }

    // Integrates `command` (normalized, per thruster) for dt.
    void step(const ThrusterArray& command, double dt) {
        const ThrusterAllocator geometry(marlin_thrusters(), center_of_gravity);
        const std::array<bool, NUM_THRUSTERS> reversed = marlin_reversed();
        ThrusterArray force{};
        for (int i = 0; i < NUM_THRUSTERS; ++i) {
            force[i] = effectiveness[i] * thruster_force(command[i], reversed[i]);
        }
        const Vector6d thrust = geometry.wrench_from_forces(force);

        constexpr int SUBSTEPS = 10;
        const double h = dt / SUBSTEPS;
        for (int k = 0; k < SUBSTEPS; ++k) {
            const Eigen::Matrix3d R = state.orientation.toRotationMatrix();
            Vector6d nu;
            nu << state.linear_velocity, state.angular_velocity;
            Vector6d wrench = thrust + truth.restoring(state.orientation) - truth.drag(nu);
            wrench.head<3>() += R.transpose() * external_force_world;
            const Vector6d inertia = truth.inertia_vector();
            const Eigen::Vector3d momentum = inertia.head<3>().cwiseProduct(state.linear_velocity);
            const Eigen::Vector3d spin = inertia.tail<3>().cwiseProduct(state.angular_velocity);
            wrench.head<3>() -= state.angular_velocity.cross(momentum);
            wrench.tail<3>() -= state.angular_velocity.cross(spin);
            const Vector6d acceleration = wrench.cwiseQuotient(inertia);

            state.linear_velocity += acceleration.head<3>() * h;
            state.angular_velocity += acceleration.tail<3>() * h;
            state.position += R * state.linear_velocity * h;
            const Eigen::Vector3d turn = state.angular_velocity * h;
            if (turn.norm() > 0.0) {
                state.orientation =
                    state.orientation * Eigen::Quaterniond(Eigen::AngleAxisd(turn.norm(), turn.normalized()));
            }
            state.orientation.normalize();
        }
    }

    Vector6d pose() const {
        Vector6d p;
        p << state.position, rpy_from_quaternion(state.orientation);
        return p;
    }
};

// The per-cycle pipeline of the sub_control node, without ROS.
struct ClosedLoop {
    Plant plant;
    ReferenceGenerator generator;
    MotionController controller;
    ThrusterAllocator allocator{marlin_thrusters(), nominal_model().center_of_gravity};
    ThrusterSettings settings;
    ReferenceLimits limits;
    ThrusterArray command{};
    std::array<bool, NUM_DOF> active{true, true, true, true, true, true};
    double time{0.0};
    double dt{0.02};
    ThrusterOutput last;

    ClosedLoop() {
        controller.set_model(nominal_model());
        controller.set_gains(nominal_gains());
        settings.reversed = marlin_reversed();
        settings.axis_weights << 1, 1, 100, 100, 100, 10;
        generator.reset(plant.pose());
    }

    void apply(const MotionCommand& command_in) { generator.apply(command_in, plant.pose(), NAN); }

    void run(double seconds) {
        const int steps = static_cast<int>(std::round(seconds / dt));
        for (int k = 0; k < steps; ++k) {
            generator.update(time, dt, plant.pose(), limits);
            ControlReference reference{generator.position(), generator.velocity(), generator.acceleration()};
            const Vector6d wrench = controller.update(plant.state, reference, active, dt);
            last = drive_thrusters(allocator, wrench, settings, command, dt);
            controller.achieved(last.achieved, dt);
            command = last.command;
            plant.step(command, dt);
            time += dt;
        }
    }
};

inline MotionCommand position_command(const Vector6d& pose) {
    MotionCommand c;
    c.mode.fill(AxisMode::POSITION);
    c.position = pose;
    return c;
}

#endif  // SUB_CONTROL_TEST_VEHICLE_FIXTURE_HPP_
