#include "sub_control/safety.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {

constexpr double INF = std::numeric_limits<double>::infinity();

ArmingInputs fresh() { return {true, 0.02, 0.02, 0.1}; }

}  // namespace

TEST(MessageAge, MissingAndFarFutureStampsAreStale) {
    EXPECT_EQ(message_age(std::nullopt, 100.0), INF);
    // The clock went back (simulator restarted) or the sender's clock is ahead.
    EXPECT_EQ(message_age(101.0, 100.0), INF);
}

TEST(MessageAge, NearFutureStampsAreFresh) {
    // The simulator's /clock trails its sensor stamps by a few milliseconds.
    EXPECT_DOUBLE_EQ(message_age(100.004, 100.0), 0.0);
    EXPECT_DOUBLE_EQ(message_age(100.0 + FUTURE_STAMP_TOLERANCE, 100.0), 0.0);
}

TEST(MessageAge, PastStampsAgeByTheirStamp) {
    EXPECT_NEAR(message_age(99.25, 100.0), 0.75, 1e-12);
    // A DVL clock that lags the vehicle's by minutes looks like no DVL at all.
    EXPECT_NEAR(message_age(-200.0, 100.0), 300.0, 1e-12);
}

TEST(ArmingBlockers, NothingBlocksAFreshVehicleAfterAKill) {
    EXPECT_TRUE(arming_blockers(fresh(), ArmingLimits{}).empty());
}

TEST(ArmingBlockers, AReleaseWithoutAKillSinceStartupBlocksUnlessNotRequired) {
    ArmingInputs inputs = fresh();
    inputs.kill_cycle_seen = false;
    EXPECT_EQ(arming_blockers(inputs, ArmingLimits{}).size(), 1u);
    ArmingLimits limits;
    limits.require_kill_cycle = false;
    EXPECT_TRUE(arming_blockers(inputs, limits).empty());
}

TEST(ArmingBlockers, EachStaleSensorBlocks) {
    for (double ArmingInputs::*age : {&ArmingInputs::state_age, &ArmingInputs::imu_age, &ArmingInputs::depth_age}) {
        ArmingInputs inputs = fresh();
        inputs.*age = 5.0;
        EXPECT_EQ(arming_blockers(inputs, ArmingLimits{}).size(), 1u);
        inputs.*age = INF;
        EXPECT_EQ(arming_blockers(inputs, ArmingLimits{}).size(), 1u);
    }
    ArmingInputs inputs = fresh();
    inputs.kill_cycle_seen = false;
    inputs.state_age = inputs.imu_age = inputs.depth_age = INF;
    EXPECT_EQ(arming_blockers(inputs, ArmingLimits{}).size(), 4u);
}

TEST(ArmingBlockers, NanAgeBlocks) {
    ArmingInputs inputs = fresh();
    inputs.imu_age = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(arming_blockers(inputs, ArmingLimits{}).size(), 1u);
}
