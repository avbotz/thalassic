#include "vehicle_fixture.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace {

constexpr double DEG = std::numbers::pi / 180.0;

double tilt(const Plant& plant) {
    const Vector6d pose = plant.pose();
    return std::max(std::fabs(pose[ROLL]), std::fabs(pose[PITCH]));
}

}  // namespace

TEST(ClosedLoop, HoldsStillAgainstModelError) {
    ClosedLoop loop;
    loop.run(30.0);
    const Vector6d pose = loop.plant.pose();
    EXPECT_LT(pose.head<3>().norm(), 0.03);
    EXPECT_LT(tilt(loop.plant), 1.0 * DEG);
    EXPECT_LT(std::fabs(pose[YAW]), 1.0 * DEG);
}

TEST(ClosedLoop, DepthStepSettlesWithoutOvershoot) {
    ClosedLoop loop;
    Vector6d goal = Vector6d::Zero();
    goal[Z] = -1.0;
    loop.apply(position_command(goal));
    double deepest = 0.0;
    for (int k = 0; k < 30 * 50; ++k) {
        loop.run(loop.dt);
        deepest = std::min(deepest, loop.plant.state.position.z());
    }
    EXPECT_NEAR(loop.plant.state.position.z(), -1.0, 0.02);
    EXPECT_GT(deepest, -1.05);
}

TEST(ClosedLoop, LongTransitStaysOnLineAndLevel) {
    ClosedLoop loop;
    loop.run(15.0);  // trim out first
    Vector6d goal = Vector6d::Zero();
    goal[X] = 4.0;
    goal[Y] = 1.0;
    goal[Z] = -0.5;
    loop.apply(position_command(goal));
    double worst_tilt = 0.0;
    for (int k = 0; k < 40 * 50; ++k) {
        loop.run(loop.dt);
        worst_tilt = std::max(worst_tilt, tilt(loop.plant));
    }
    EXPECT_LT((loop.plant.state.position - goal.head<3>()).norm(), 0.05);
    EXPECT_LT(worst_tilt, 3.0 * DEG);
}

TEST(ClosedLoop, HalfTurnStaysLevel) {
    ClosedLoop loop;
    loop.run(15.0);
    Vector6d goal = Vector6d::Zero();
    goal[YAW] = 3.0;
    loop.apply(position_command(goal));
    double worst_tilt = 0.0;
    for (int k = 0; k < 20 * 50; ++k) {
        loop.run(loop.dt);
        worst_tilt = std::max(worst_tilt, tilt(loop.plant));
    }
    EXPECT_NEAR(wrap_angle(loop.plant.pose()[YAW] - 3.0), 0.0, 1.0 * DEG);
    EXPECT_LT(worst_tilt, 3.0 * DEG);
    EXPECT_LT(loop.plant.state.position.norm(), 0.05);
}

TEST(ClosedLoop, SurgesAtSpeedWhileHoldingDepthAndHeading) {
    ClosedLoop loop;
    Vector6d goal = Vector6d::Zero();
    goal[Z] = -1.0;
    loop.apply(position_command(goal));
    loop.run(20.0);

    MotionCommand surge;
    surge.mode[X] = surge.mode[Y] = AxisMode::VELOCITY;
    surge.velocity[X] = 0.3;
    loop.apply(surge);
    loop.run(10.0);
    double worst_depth = 0.0;
    for (int k = 0; k < 10 * 50; ++k) {
        loop.run(loop.dt);
        worst_depth = std::max(worst_depth, std::fabs(loop.plant.state.position.z() + 1.0));
    }
    EXPECT_NEAR(loop.plant.state.linear_velocity.x(), 0.3, 0.02);
    EXPECT_LT(worst_depth, 0.05);
    EXPECT_LT(std::fabs(loop.plant.pose()[YAW]), 2.0 * DEG);
}

TEST(ClosedLoop, RidesThroughAnUndeclaredVerticalThrusterFailure) {
    ClosedLoop loop;
    Vector6d goal = Vector6d::Zero();
    goal[Z] = -1.0;
    loop.apply(position_command(goal));
    loop.run(20.0);
    loop.plant.effectiveness[0] = 0.0;  // dies; nobody tells the controller
    double worst_tilt = 0.0;
    for (int k = 0; k < 30 * 50; ++k) {
        loop.run(loop.dt);
        worst_tilt = std::max(worst_tilt, tilt(loop.plant));
    }
    // Passively: the integrators carry the vehicle while the allocator still
    // leans on the dead thruster. Declaring the failure does better (below).
    EXPECT_NEAR(loop.plant.state.position.z(), -1.0, 0.05);
    EXPECT_LT(tilt(loop.plant), 3.0 * DEG);
    EXPECT_LT(worst_tilt, 10.0 * DEG);
}

TEST(ClosedLoop, DeclaredFailureIsAllocatedAround) {
    ClosedLoop loop;
    Vector6d goal = Vector6d::Zero();
    goal[Z] = -1.0;
    loop.apply(position_command(goal));
    loop.run(20.0);
    loop.plant.effectiveness[5] = 0.0;
    loop.settings.health[5] = 0.0;
    goal[X] = 2.0;
    goal[YAW] = 1.0;
    loop.apply(position_command(goal));
    loop.run(40.0);
    EXPECT_LT((loop.plant.state.position - goal.head<3>()).norm(), 0.05);
    EXPECT_NEAR(wrap_angle(loop.plant.pose()[YAW] - 1.0), 0.0, 2.0 * DEG);
    EXPECT_EQ(loop.last.command[5], 0.0);
}

TEST(ClosedLoop, KeepsDepthAfterLosingBothFrontVerticals) {
    ClosedLoop loop;
    Vector6d goal = Vector6d::Zero();
    goal[Z] = -1.0;
    loop.apply(position_command(goal));
    loop.run(20.0);
    for (const int t : {0, 1}) {
        loop.plant.effectiveness[t] = 0.0;
        loop.settings.health[t] = 0.0;
    }
    // What the node does on a health change: drop the axis that is gone.
    const AxisFlags controllable = loop.allocator.controllable_axes({false, false, true, true, true, true, true, true},
                                                                    {Z, YAW, PITCH, X, Y, ROLL});
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        if (!controllable[axis]) {
            loop.active[axis] = false;
            loop.settings.axis_weights[axis] = 1e-6;
        }
    }
    ASSERT_FALSE(loop.active[PITCH]);
    loop.run(30.0);
    EXPECT_NEAR(loop.plant.state.position.z(), -1.0, 0.1);
    EXPECT_LT(std::fabs(loop.plant.pose()[ROLL]), 3.0 * DEG);
}

TEST(ClosedLoop, SaturationDoesNotWindUp) {
    ClosedLoop loop;
    loop.settings.power_limit = 0.3;  // far less thrust than the trajectory asks for
    loop.limits.horizontal = {1.0, 0.5, 1.0};
    Vector6d goal = Vector6d::Zero();
    goal[X] = 6.0;
    loop.apply(position_command(goal));
    double furthest = 0.0;
    for (int k = 0; k < 90 * 50; ++k) {
        loop.run(loop.dt);
        furthest = std::max(furthest, loop.plant.state.position.x());
    }
    EXPECT_LT(furthest, 6.15);
    EXPECT_NEAR(loop.plant.state.position.x(), 6.0, 0.05);
}

TEST(ClosedLoop, DisturbanceObserverTakesOutABuoyancyStep) {
    for (const bool observer : {false, true}) {
        ClosedLoop loop;
        if (observer) {
            ControlGains gains = nominal_gains();
            gains.observer_bandwidth << 1.0, 1.0, 1.5, 0.0, 0.0, 1.0;
            gains.ki *= 0.2;
            loop.controller.set_gains(gains);
        }
        Vector6d goal = Vector6d::Zero();
        goal[Z] = -1.0;
        loop.apply(position_command(goal));
        loop.run(20.0);
        loop.plant.external_force_world = {0.0, 0.0, -15.0};  // picks up a 1.5 kg object
        double worst = 0.0;
        for (int k = 0; k < 20 * 50; ++k) {
            loop.run(loop.dt);
            worst = std::max(worst, std::fabs(loop.plant.state.position.z() + 1.0));
        }
        EXPECT_NEAR(loop.plant.state.position.z(), -1.0, 0.02) << "observer " << observer;
        EXPECT_LT(worst, 0.15) << "observer " << observer;
        if (observer) {
            // Everything the model leaves out: the step plus its own buoyancy error.
            const double expected = -15.0 + loop.plant.truth.net_buoyancy - nominal_model().net_buoyancy;
            EXPECT_NEAR(loop.controller.disturbance()[Z], expected, 1.0);
        }
    }
}

TEST(ClosedLoop, RetuningTheObserverKeepsItsEstimate) {
    ClosedLoop loop;
    ControlGains gains = nominal_gains();
    gains.observer_bandwidth << 1.0, 1.0, 1.5, 0.0, 0.0, 1.0;
    loop.controller.set_gains(gains);
    Vector6d goal = Vector6d::Zero();
    goal[Z] = -1.0;
    loop.apply(position_command(goal));
    loop.run(30.0);
    const double before = loop.controller.disturbance()[Z];
    ASSERT_GT(std::fabs(before), 5.0);  // the model's buoyancy error
    gains.observer_bandwidth[Z] = 3.0;
    loop.controller.set_gains(gains);
    loop.run(loop.dt);
    // A faster observer converges faster; it does not rescale what it has.
    EXPECT_NEAR(loop.controller.disturbance()[Z], before, 0.05 * std::fabs(before));
}

TEST(ClosedLoop, RetuningTheMassMidSurgeKeepsTheObserverEstimate) {
    ClosedLoop loop;
    ControlGains gains = nominal_gains();
    gains.observer_bandwidth << 1.0, 1.0, 1.5, 0.0, 0.0, 1.0;
    loop.controller.set_gains(gains);
    MotionCommand surge;
    surge.mode[X] = surge.mode[Y] = AxisMode::VELOCITY;
    surge.velocity[X] = 0.3;
    loop.apply(surge);
    loop.run(15.0);
    const double before = loop.controller.disturbance()[X];
    VehicleModel heavier = nominal_model();
    heavier.mass *= 1.5;  // a step of K dM v = 6.75 N if the observer's momentum is not rebased
    loop.controller.set_model(heavier);
    loop.run(loop.dt);
    EXPECT_NEAR(loop.controller.disturbance()[X], before, 0.5);
}

TEST(ClosedLoop, PoseResetKeepsHoldingAgainstACurrent) {
    // Held at yaw 90° against a current along world x until the integrators
    // carry it, then x, y and yaw are re-zeroed where the vehicle is, as
    // reset_pose does. The world-frame integral has to turn with the frame, or
    // it pushes against a current that is no longer there.
    std::array<double, 2> drift{};
    for (const bool rotate : {false, true}) {
        ClosedLoop loop;
        loop.plant.external_force_world = {8.0, 0.0, 0.0};
        Vector6d goal = Vector6d::Zero();
        goal[Z] = -1.0;
        goal[YAW] = 90.0 * DEG;
        loop.apply(position_command(goal));
        loop.run(40.0);

        const double yaw = loop.plant.pose()[YAW];
        const Eigen::AngleAxisd turn(-yaw, Eigen::Vector3d::UnitZ());
        loop.plant.state.position = {0.0, 0.0, loop.plant.state.position.z()};
        loop.plant.state.orientation = turn * loop.plant.state.orientation;
        loop.plant.external_force_world = turn * loop.plant.external_force_world;
        loop.generator.reset(loop.plant.pose());
        if (rotate) {
            loop.controller.rotate_world(-yaw);
        }

        for (int k = 0; k < 20 * 50; ++k) {
            loop.run(loop.dt);
            drift[rotate] = std::max(drift[rotate], loop.plant.state.position.head<2>().norm());
        }
        EXPECT_LT(std::fabs(loop.plant.pose()[YAW]), 1.0 * DEG) << "rotate " << rotate;
    }
    EXPECT_LT(drift[1], 0.03);
    EXPECT_GT(drift[0], 3.0 * drift[1]);  // the test can tell the difference
}
