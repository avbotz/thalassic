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

TEST(ThrustCurve, MatchesTheMeasuredCurveAtSixteenVolts) {
    // BlueRobotics' data for the T200 at 16 V: 5.25 kgf forward and 4.07 kgf in
    // reverse at full command, and nothing within 7% of zero.
    EXPECT_EQ(thruster_voltage(), DEFAULT_THRUSTER_VOLTAGE);
    EXPECT_NEAR(norm_to_force(1.0), 51.44, 0.01);
    EXPECT_NEAR(norm_to_force(-1.0), -39.91, 0.01);
    EXPECT_EQ(norm_to_force(0.07), 0.0);
    EXPECT_EQ(norm_to_force(-0.07), 0.0);
    EXPECT_GT(norm_to_force(0.08), 0.0);
    EXPECT_LT(norm_to_force(-0.08), 0.0);
}

TEST(ThrustCurve, FollowsTheSupplyVoltage) {
    const double at_16 = norm_to_force(0.6);
    set_thruster_voltage(12.0);
    EXPECT_EQ(thruster_voltage(), 12.0);
    const double at_12 = norm_to_force(0.6);
    EXPECT_LT(at_12, 0.8 * at_16);
    // The same force needs a larger command on a lower battery.
    EXPECT_GT(force_to_norm(10.0), 0.3);
    for (double force = -25.0; force <= 30.0; force += 0.5) {
        EXPECT_NEAR(norm_to_force(force_to_norm(force)), force, 0.1) << "force " << force;
    }
    // Between two measured voltages the curve is interpolated, and outside
    // their range it is the nearest one.
    set_thruster_voltage(14.0);
    const double at_14 = norm_to_force(0.6);
    set_thruster_voltage(13.0);
    EXPECT_NEAR(norm_to_force(0.6), 0.5 * (at_12 + at_14), 1e-9);
    set_thruster_voltage(5.0);
    EXPECT_EQ(thruster_voltage(), 10.0);
    set_thruster_voltage(DEFAULT_THRUSTER_VOLTAGE);
    EXPECT_NEAR(norm_to_force(0.6), at_16, 1e-12);
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
