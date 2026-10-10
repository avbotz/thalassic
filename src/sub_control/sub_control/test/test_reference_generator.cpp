#include "sub_control/reference_generator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace {

constexpr double DT = 0.02;

// Follows the reference exactly, as a perfect vehicle would.
struct Tracker {
    ReferenceGenerator generator;
    ReferenceLimits limits;
    double time{0.0};

    Tracker() { generator.reset(Vector6d::Zero()); }

    void run(double seconds) {
        for (int k = 0; k < static_cast<int>(std::round(seconds / DT)); ++k) {
            generator.update(time, DT, generator.position(), limits);
            time += DT;
        }
    }
};

MotionCommand only(int axis, AxisMode mode, double value) {
    MotionCommand c;
    c.mode[axis] = mode;
    (mode == AxisMode::VELOCITY ? c.velocity : c.position)[axis] = value;
    return c;
}

}  // namespace

TEST(ReferenceGenerator, ResetHoldsPoseAndLevels) {
    ReferenceGenerator g;
    Vector6d pose;
    pose << 1, 2, -0.5, 0.1, -0.1, 2.0;
    g.reset(pose);
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        EXPECT_EQ(g.mode(axis), AxisMode::POSITION);
    }
    EXPECT_DOUBLE_EQ(g.target()[ROLL], 0.0);
    EXPECT_DOUBLE_EQ(g.target()[PITCH], 0.0);
    EXPECT_DOUBLE_EQ(g.target()[YAW], 2.0);
    EXPECT_TRUE(g.position().isApprox(pose));
}

TEST(ReferenceGenerator, XAndYMustShareAMode) {
    ReferenceGenerator g;
    g.reset(Vector6d::Zero());
    EXPECT_NE(g.apply(only(X, AxisMode::VELOCITY, 0.3), Vector6d::Zero(), NAN), "");
    MotionCommand both = only(X, AxisMode::VELOCITY, 0.3);
    both.mode[Y] = AxisMode::VELOCITY;
    EXPECT_EQ(g.apply(both, Vector6d::Zero(), NAN), "");
}

TEST(ReferenceGenerator, KeepLeavesOtherAxesAlone) {
    Tracker t;
    t.generator.apply(only(Z, AxisMode::POSITION, -1.0), Vector6d::Zero(), NAN);
    MotionCommand surge;
    surge.mode[X] = surge.mode[Y] = AxisMode::VELOCITY;
    surge.velocity[X] = 0.3;
    t.generator.apply(surge, Vector6d::Zero(), NAN);
    t.run(20.0);
    EXPECT_EQ(t.generator.mode(Z), AxisMode::POSITION);
    EXPECT_NEAR(t.generator.position()[Z], -1.0, 1e-3);
    EXPECT_NEAR(t.generator.velocity()[X], 0.3, 1e-3);
}

TEST(ReferenceGenerator, HeadingFrameVelocityFollowsYaw) {
    Tracker t;
    t.generator.apply(only(YAW, AxisMode::POSITION, std::numbers::pi / 2), Vector6d::Zero(), NAN);
    t.run(10.0);
    MotionCommand surge;
    surge.mode[X] = surge.mode[Y] = AxisMode::VELOCITY;
    surge.velocity[X] = 0.3;
    t.generator.apply(surge, t.generator.position(), NAN);
    t.run(10.0);
    EXPECT_NEAR(t.generator.velocity()[X], 0.0, 1e-3);
    EXPECT_NEAR(t.generator.velocity()[Y], 0.3, 1e-3);
}

TEST(ReferenceGenerator, VelocityTimesOutIntoAHold) {
    Tracker t;
    MotionCommand c = only(Z, AxisMode::VELOCITY, -0.2);
    c.expires = 2.0;
    t.generator.apply(c, Vector6d::Zero(), NAN);
    t.run(1.9);
    EXPECT_EQ(t.generator.mode(Z), AxisMode::VELOCITY);
    t.run(6.0);
    EXPECT_EQ(t.generator.mode(Z), AxisMode::POSITION);  // stopped and latched
    EXPECT_NEAR(t.generator.velocity()[Z], 0.0, 0.02);
    const double latched = t.generator.target()[Z];
    t.run(5.0);
    EXPECT_NEAR(t.generator.position()[Z], latched, 1e-3);
}

TEST(ReferenceGenerator, HoldStopsSmoothlyThenHoldsThere) {
    Tracker t;
    MotionCommand surge;
    surge.mode[X] = surge.mode[Y] = AxisMode::VELOCITY;
    surge.velocity[X] = 0.4;
    t.generator.apply(surge, Vector6d::Zero(), NAN);
    t.run(8.0);
    MotionCommand hold;
    hold.mode[X] = hold.mode[Y] = AxisMode::HOLD;
    t.generator.apply(hold, t.generator.position(), NAN);
    double previous = t.generator.position()[X];
    for (int k = 0; k < 600; ++k) {
        t.run(DT);
        EXPECT_GE(t.generator.position()[X], previous - 1e-9);  // never pulls back
        previous = t.generator.position()[X];
    }
    EXPECT_EQ(t.generator.mode(X), AxisMode::POSITION);
    EXPECT_NEAR(t.generator.target()[X], t.generator.position()[X], 1e-3);
}

TEST(ReferenceGenerator, YawTakesTheShortWayAcrossTheWrap) {
    ReferenceGenerator g;
    Vector6d pose = Vector6d::Zero();
    pose[YAW] = 3.0;
    g.reset(pose);
    g.apply(only(YAW, AxisMode::POSITION, -3.0), pose, NAN);
    ReferenceLimits limits;
    double time = 0.0;
    for (int k = 0; k < 1000; ++k) {
        g.update(time, DT, g.position(), limits);
        time += DT;
        EXPECT_GE(g.velocity()[YAW], -1e-6);  // only ever turns positive
    }
    EXPECT_NEAR(wrap_angle(g.position()[YAW] - (-3.0)), 0.0, 1e-3);
    EXPECT_NEAR(g.position()[YAW], 2 * std::numbers::pi - 3.0, 1e-3);  // continuous, not wrapped
}

TEST(ReferenceGenerator, LeashKeepsReferenceNearAStuckVehicle) {
    ReferenceGenerator g;
    g.reset(Vector6d::Zero());
    g.apply(only(Z, AxisMode::POSITION, -5.0), Vector6d::Zero(), NAN);
    ReferenceLimits limits;
    for (int k = 0; k < 2000; ++k) {
        g.update(k * DT, DT, Vector6d::Zero(), limits);  // the vehicle never moves
        EXPECT_LE(std::fabs(g.position()[Z]), limits.translation_leash + 1e-9);
    }
}

TEST(ReferenceGenerator, AltitudeRebasesWithoutAJump) {
    Tracker t;
    t.generator.apply(only(Z, AxisMode::POSITION, -1.0), Vector6d::Zero(), NAN);
    t.run(15.0);
    MotionCommand c = only(Z, AxisMode::POSITION, 1.5);
    c.altitude = true;
    EXPECT_NE(t.generator.apply(c, t.generator.position(), NAN), "");  // no altitude, no altitude hold
    EXPECT_EQ(t.generator.apply(c, t.generator.position(), 2.0), "");
    EXPECT_TRUE(t.generator.altitude());
    EXPECT_NEAR(t.generator.position()[Z], 2.0, 1e-9);  // now in altitude coordinates, where the vehicle is
    t.run(15.0);
    EXPECT_NEAR(t.generator.position()[Z], 1.5, 1e-3);
    t.generator.drop_altitude(-1.4);
    EXPECT_FALSE(t.generator.altitude());
    EXPECT_NEAR(t.generator.position()[Z], -1.4, 1e-9);
    EXPECT_EQ(t.generator.mode(Z), AxisMode::HOLD);
}

TEST(ReferenceGenerator, AltitudeBelowTheBottomIsRejected) {
    ReferenceGenerator g;
    g.reset(Vector6d::Zero());
    MotionCommand c = only(Z, AxisMode::POSITION, -0.5);
    c.altitude = true;
    EXPECT_NE(g.apply(c, Vector6d::Zero(), 2.0), "");
    EXPECT_FALSE(g.altitude());
    EXPECT_DOUBLE_EQ(g.target()[Z], 0.0);
    c.altitude = false;  // as a depth it is fine
    EXPECT_EQ(g.apply(c, Vector6d::Zero(), 2.0), "");
}

TEST(ReferenceGenerator, PassiveAxisFollowsTheVehicleAndResumesTheTarget) {
    ReferenceGenerator g;
    g.reset(Vector6d::Zero());
    MotionCommand c;
    c.mode[X] = c.mode[Y] = AxisMode::POSITION;
    c.position[X] = 4.0;
    g.apply(c, Vector6d::Zero(), NAN);
    ReferenceLimits limits;
    g.set_passive(X, true);
    g.set_passive(Y, true);
    Vector6d drifting = Vector6d::Zero();
    drifting[X] = 0.7;
    g.update(0.0, DT, drifting, limits);
    EXPECT_NEAR(g.position()[X], 0.7, 1e-12);
    EXPECT_NEAR(g.velocity()[X], 0.0, 1e-12);
    g.set_passive(X, false);
    g.set_passive(Y, false);
    for (int k = 0; k < 3000; ++k) {
        g.update(k * DT, DT, g.position(), limits);
    }
    EXPECT_NEAR(g.position()[X], 4.0, 1e-3);
}
