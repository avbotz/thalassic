#include "sub_control/utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

TEST(ThrustCurve, CommandRoundTripsForBothPropellerHands) {
    for (const bool reversed : {false, true}) {
        for (double force = -39.0; force <= 51.0; force += 0.5) {
            const double command = thruster_command(force, reversed);
            EXPECT_NEAR(thruster_force(command, reversed), force, 0.05)
                << "force " << force << " reversed " << reversed;
        }
    }
}

TEST(ThrustCurve, LeftHandPropellerPushesBackOnPositiveCommand) {
    // A mirrored propeller spun forward pushes the other way, with the curve's
    // weaker (reverse) half.
    EXPECT_NEAR(thruster_force(0.6, true), norm_to_force(-0.6), 1e-9);
    EXPECT_LT(thruster_force(0.6, true), 0.0);
    EXPECT_GT(thruster_command(30.0, true), -1.0);
    EXPECT_LT(thruster_command(30.0, true), 0.0);
}

TEST(ThrustCurve, NaNIsOffNotFullScale) {
    for (const bool reversed : {false, true}) {
        EXPECT_EQ(thruster_command(NAN, reversed), 0.0) << "reversed " << reversed;
        EXPECT_EQ(thruster_force(NAN, reversed), 0.0) << "reversed " << reversed;
    }
}

TEST(Attitude, ErrorIsBodyRotationVector) {
    const Eigen::Quaterniond level = quaternion_from_rpy({0.0, 0.0, 0.0});
    EXPECT_NEAR(attitude_error(level, level).norm(), 0.0, 1e-12);

    const Eigen::Vector3d yaw90 = attitude_error(quaternion_from_rpy({0.0, 0.0, std::numbers::pi / 2}), level);
    EXPECT_NEAR(yaw90.z(), std::numbers::pi / 2, 1e-9);
    EXPECT_NEAR(yaw90.head<2>().norm(), 0.0, 1e-9);
}

TEST(Attitude, ErrorTakesTheShortWayAcrossTheWrap) {
    const Eigen::Vector3d e = attitude_error(quaternion_from_rpy({0.0, 0.0, 170.0 * std::numbers::pi / 180.0}),
                                             quaternion_from_rpy({0.0, 0.0, -170.0 * std::numbers::pi / 180.0}));
    EXPECT_NEAR(e.z(), -20.0 * std::numbers::pi / 180.0, 1e-9);
}

TEST(Attitude, ErrorIsInTheCurrentBodyFrame) {
    // Pitched 90 deg nose down, a world yaw error appears about body x.
    const Eigen::Quaterniond current = quaternion_from_rpy({0.0, std::numbers::pi / 2 - 1e-6, 0.0});
    const Eigen::Quaterniond target = Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())) * current;
    const Eigen::Vector3d e = attitude_error(target, current);
    EXPECT_NEAR(std::fabs(e.x()), 0.3, 1e-4);
    EXPECT_NEAR(e.z(), 0.0, 1e-4);
}

TEST(Attitude, RpyRoundTrip) {
    const Eigen::Vector3d rpy{0.2, -0.4, 2.9};
    EXPECT_TRUE(rpy_from_quaternion(quaternion_from_rpy(rpy)).isApprox(rpy, 1e-9));
}

TEST(Attitude, EulerRatesAreBodyRatesWhenLevel) {
    const Eigen::Vector3d rates{0.1, -0.2, 0.3};
    EXPECT_TRUE(euler_rates_to_body({0.0, 0.0, 1.0}, rates).isApprox(rates, 1e-12));
    // Rolled 90 deg, a yaw rate is a body pitch rate.
    const Eigen::Vector3d rolled = euler_rates_to_body({std::numbers::pi / 2, 0.0, 0.0}, {0.0, 0.0, 1.0});
    EXPECT_NEAR(rolled.y(), 1.0, 1e-9);
    EXPECT_NEAR(rolled.z(), 0.0, 1e-9);
}

TEST(Attitude, WrapAngle) {
    EXPECT_NEAR(wrap_angle(3.5), 3.5 - 2 * std::numbers::pi, 1e-12);
    EXPECT_NEAR(wrap_angle(-3.5), -3.5 + 2 * std::numbers::pi, 1e-12);
    EXPECT_NEAR(wrap_angle(0.5), 0.5, 1e-12);
}
