#include "sub_control/trajectory.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr double DT = 0.02;

struct Trace {
    double max_velocity{0.0};
    double max_acceleration{0.0};
    double max_jerk{0.0};
    double overshoot{0.0};
};

Trace move(double distance, const ShaperLimits& limits, double seconds, double& final_position) {
    KinematicShaper<1> shaper;
    shaper.reset(Eigen::Matrix<double, 1, 1>(0.0));
    Trace trace;
    double previous_acceleration = 0.0;
    for (int k = 0; k < static_cast<int>(seconds / DT); ++k) {
        shaper.update_position(Eigen::Matrix<double, 1, 1>(distance), limits, DT);
        trace.max_velocity = std::max(trace.max_velocity, std::fabs(shaper.velocity()(0)));
        trace.max_acceleration = std::max(trace.max_acceleration, std::fabs(shaper.acceleration()(0)));
        trace.max_jerk = std::max(trace.max_jerk, std::fabs(shaper.acceleration()(0) - previous_acceleration) / DT);
        trace.overshoot = std::max(trace.overshoot, std::copysign(1.0, distance) * (shaper.position()(0) - distance));
        previous_acceleration = shaper.acceleration()(0);
    }
    final_position = shaper.position()(0);
    return trace;
}

}  // namespace

class ShaperStep : public ::testing::TestWithParam<double> {};

TEST_P(ShaperStep, ArrivesWithinLimitsWithoutOvershoot) {
    const ShaperLimits limits{0.5, 0.25, 0.5};
    const double distance = GetParam();
    double final_position = 0.0;
    const Trace trace = move(distance, limits, 20.0 + 3.0 * std::fabs(distance), final_position);
    EXPECT_LE(trace.max_velocity, limits.max_velocity + 1e-9);
    EXPECT_LE(trace.max_acceleration, limits.max_acceleration + 1e-9);
    EXPECT_LE(trace.max_jerk, limits.max_jerk + 1e-6);
    EXPECT_LE(trace.overshoot, 0.01 * std::fabs(distance) + 1e-3);
    EXPECT_NEAR(final_position, distance, 1e-3);
}

INSTANTIATE_TEST_SUITE_P(Distances, ShaperStep, ::testing::Values(0.02, 0.3, 1.0, 3.0, 10.0, -2.0));

TEST(Shaper, DiagonalMoveIsStraight) {
    const ShaperLimits limits{0.5, 0.25, 0.5};
    KinematicShaper<2> shaper;
    shaper.reset(Eigen::Vector2d::Zero());
    const Eigen::Vector2d target{3.0, 1.0};
    const Eigen::Vector2d along = target.normalized();
    double worst = 0.0;
    for (int k = 0; k < 2000; ++k) {
        shaper.update_position(target, limits, DT);
        const Eigen::Vector2d p = shaper.position();
        worst = std::max(worst, std::fabs(p.x() * along.y() - p.y() * along.x()));
        EXPECT_LE(shaper.velocity().norm(), limits.max_velocity + 1e-9);
    }
    EXPECT_LT(worst, 1e-9);
    EXPECT_TRUE(shaper.position().isApprox(target, 1e-3));
}

TEST(Shaper, ReachesVelocityTarget) {
    const ShaperLimits limits{0.5, 0.25, 0.5};
    KinematicShaper<1> shaper;
    for (int k = 0; k < 500; ++k) {
        shaper.update_velocity(Eigen::Matrix<double, 1, 1>(0.3), limits, DT);
        EXPECT_LE(std::fabs(shaper.acceleration()(0)), limits.max_acceleration + 1e-9);
    }
    EXPECT_NEAR(shaper.velocity()(0), 0.3, 1e-4);
    // A target above the limit is clipped to it.
    for (int k = 0; k < 500; ++k) {
        shaper.update_velocity(Eigen::Matrix<double, 1, 1>(2.0), limits, DT);
    }
    EXPECT_NEAR(shaper.velocity()(0), limits.max_velocity, 1e-4);
}

TEST(Shaper, RetargetingMidMoveStaysWithinLimits) {
    const ShaperLimits limits{0.5, 0.25, 0.5};
    KinematicShaper<1> shaper;
    double previous_acceleration = 0.0;
    for (int k = 0; k < 1500; ++k) {
        const double target = k < 300 ? 5.0 : -1.0;  // reverse while cruising
        shaper.update_position(Eigen::Matrix<double, 1, 1>(target), limits, DT);
        EXPECT_LE(std::fabs(shaper.acceleration()(0)), limits.max_acceleration + 1e-9);
        EXPECT_LE(std::fabs(shaper.acceleration()(0) - previous_acceleration) / DT, limits.max_jerk + 1e-6);
        previous_acceleration = shaper.acceleration()(0);
    }
    EXPECT_NEAR(shaper.position()(0), -1.0, 1e-3);
}

TEST(SqrtController, LinearNearZeroAndSquareRootFar) {
    EXPECT_NEAR(sqrt_controller(0.01, 1.0, 0.5, 0.0), 0.01, 1e-12);
    // Far away it brakes at the given deceleration: v = sqrt(2 a (e - a/(2 p^2))).
    EXPECT_NEAR(sqrt_controller(4.0, 1.0, 0.5, 0.0), std::sqrt(2.0 * 0.5 * (4.0 - 0.25)), 1e-12);
    EXPECT_NEAR(sqrt_controller(-4.0, 1.0, 0.5, 0.0), -std::sqrt(2.0 * 0.5 * (4.0 - 0.25)), 1e-12);
    // Never more than closes the error in one step.
    EXPECT_NEAR(sqrt_controller(0.001, 100.0, 0.5, 0.1), 0.01, 1e-12);
}
