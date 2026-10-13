#include "vehicle_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <initializer_list>

namespace {

ThrusterAllocator marlin() { return ThrusterAllocator(marlin_thrusters(), nominal_model().center_of_gravity); }

std::array<bool, NUM_THRUSTERS> usable_without(std::initializer_list<int> failed) {
    std::array<bool, NUM_THRUSTERS> usable;
    usable.fill(true);
    for (const int t : failed) {
        usable[t] = false;
    }
    return usable;
}

constexpr std::array<int, NUM_DOF> PRIORITY{Z, YAW, PITCH, X, Y, ROLL};

int count(const AxisFlags& flags) { return static_cast<int>(std::count(flags.begin(), flags.end(), true)); }

}  // namespace

TEST(Allocator, VerticalsAndHorizontalsSplitTheAxes) {
    // With the horizontals level with the centre of gravity, the verticals do
    // heave/roll/pitch and the horizontals surge/sway/yaw.
    const auto B = marlin().matrix();
    for (int t = 0; t < 4; ++t) {
        EXPECT_NEAR(B(X, t), 0.0, 1e-12);
        EXPECT_NEAR(B(Y, t), 0.0, 1e-12);
        EXPECT_NEAR(B(YAW, t), 0.0, 1e-12);
        EXPECT_NEAR(B(Z, t), -1.0, 1e-12);  // mounted +X points down
    }
    for (int t = 4; t < 8; ++t) {
        EXPECT_NEAR(B(Z, t), 0.0, 1e-12);
        EXPECT_NEAR(B(ROLL, t), 0.0, 1e-12);
        EXPECT_NEAR(B(PITCH, t), 0.0, 1e-12);
    }
}

TEST(Allocator, ReachableWrenchIsProducedExactly) {
    const ThrusterAllocator a = marlin();
    ThrusterArray lower;
    ThrusterArray upper;
    lower.fill(-30.0);
    upper.fill(30.0);
    Vector6d wrench;
    wrench << 20, -10, 35, 2, -3, 5;
    const ThrusterArray f = a.allocate(wrench, lower, upper, Vector6d::Ones());
    EXPECT_TRUE(a.wrench_from_forces(f).isApprox(wrench, 1e-6));
}

TEST(Allocator, AnySingleFailureKeepsEveryAxis) {
    const ThrusterAllocator a = marlin();
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        EXPECT_EQ(count(a.controllable_axes(usable_without({t}), PRIORITY)), NUM_DOF) << "thruster " << t;
    }
}

TEST(Allocator, OneVerticalAndOneHorizontalKeepEveryAxis) {
    const ThrusterAllocator a = marlin();
    for (int v = 0; v < 4; ++v) {
        for (int h = 4; h < 8; ++h) {
            EXPECT_EQ(count(a.controllable_axes(usable_without({v, h}), PRIORITY)), NUM_DOF) << v << "," << h;
        }
    }
}

TEST(Allocator, TwoInOneGroupLoseTheLowestPriorityAxis) {
    const ThrusterAllocator a = marlin();
    // Front verticals: heave and pitch now move together; heave is kept.
    AxisFlags front = a.controllable_axes(usable_without({0, 1}), PRIORITY);
    EXPECT_EQ(count(front), 5);
    EXPECT_TRUE(front[Z]);
    EXPECT_FALSE(front[PITCH]);
    // Diagonal verticals: roll and pitch couple; pitch outranks roll.
    AxisFlags diagonal = a.controllable_axes(usable_without({0, 3}), PRIORITY);
    EXPECT_EQ(count(diagonal), 5);
    EXPECT_FALSE(diagonal[ROLL]);
    // Left horizontals: surge and yaw couple; yaw is kept.
    AxisFlags left = a.controllable_axes(usable_without({4, 6}), PRIORITY);
    EXPECT_EQ(count(left), 5);
    EXPECT_TRUE(left[YAW]);
    EXPECT_FALSE(left[X]);
}

TEST(Allocator, AnyTwoInOneGroupLoseOneAxisButKeepDepthAndHeading) {
    const ThrusterAllocator a = marlin();
    for (const int group : {0, 4}) {
        for (int i = group; i < group + 4; ++i) {
            for (int j = i + 1; j < group + 4; ++j) {
                const AxisFlags kept = a.controllable_axes(usable_without({i, j}), PRIORITY);
                EXPECT_EQ(count(kept), NUM_DOF - 1) << i << "," << j;
                EXPECT_TRUE(kept[Z]) << i << "," << j;
                EXPECT_TRUE(kept[YAW]) << i << "," << j;
            }
        }
    }
}

TEST(Allocator, TieredWeightsKeepDepthAndAttitudeWhenSurgeSaturates) {
    const ThrusterAllocator a = marlin();
    ThrusterArray lower;
    ThrusterArray upper;
    lower.fill(norm_to_force(-1.0));
    upper.fill(norm_to_force(1.0));
    Vector6d demand;
    demand << 180, 0, 60, 0, 0, 15;  // more surge than four horizontals have

    Vector6d tiered;
    tiered << 1, 1, 100, 100, 100, 10;
    const Vector6d kept = a.wrench_from_forces(a.allocate(demand, lower, upper, tiered));
    EXPECT_NEAR(kept[Z], 60.0, 0.5);
    EXPECT_NEAR(kept[ROLL], 0.0, 0.1);
    EXPECT_NEAR(kept[PITCH], 0.0, 0.1);
    EXPECT_GT(kept[YAW], 0.85 * 15.0);
    EXPECT_NEAR(kept[Y], 0.0, 1.0);

    Vector6d flat;
    flat << 1, 1, 2, 2, 2, 1.5;
    const Vector6d lost = a.wrench_from_forces(a.allocate(demand, lower, upper, flat));
    EXPECT_LT(lost[YAW], kept[YAW]);
}

TEST(Allocator, DriveThrustersHonoursHealthAndHandedness) {
    const ThrusterAllocator a = marlin();
    ThrusterSettings settings;
    settings.reversed = marlin_reversed();
    settings.health[0] = 0.0;
    settings.health[5] = 0.5;
    Vector6d wrench;
    wrench << 15, 5, 30, 0, 0, 3;
    const ThrusterOutput out = drive_thrusters(a, wrench, settings, {}, 0.02);

    EXPECT_EQ(out.command[0], 0.0);
    EXPECT_EQ(out.force[0], 0.0);
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        EXPECT_LE(std::fabs(out.command[t]), settings.power_limit + 1e-12);
        const double health = settings.health[t];
        EXPECT_NEAR(out.force[t], health * thruster_force(out.command[t], settings.reversed[t]), 1e-9);
    }
    EXPECT_TRUE(out.achieved.isApprox(wrench, 2e-2));
}

TEST(Allocator, DriveThrustersSlewLimits) {
    const ThrusterAllocator a = marlin();
    ThrusterSettings settings;
    settings.reversed = marlin_reversed();
    settings.max_command_rate = 2.0;
    Vector6d wrench;
    wrench << 0, 0, 60, 0, 0, 0;
    const ThrusterOutput out = drive_thrusters(a, wrench, settings, {}, 0.02);
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        EXPECT_LE(std::fabs(out.command[t]), 0.04 + 1e-12);
    }
    EXPECT_LT(out.achieved[Z], 60.0);  // the shortfall is reported, not hidden
}

TEST(Allocator, SlewingDownStaysWithinALoweredPowerLimit) {
    const ThrusterAllocator a = marlin();
    ThrusterSettings settings;
    settings.reversed = marlin_reversed();
    settings.max_command_rate = 1.0;
    settings.power_limit = 0.2;  // just lowered from 0.6
    ThrusterArray previous;
    previous.fill(0.6);
    const ThrusterOutput out = drive_thrusters(a, Vector6d::Zero(), settings, previous, 0.02);
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        // Capped at once, yet still slewing toward the 0 the wrench asks for.
        EXPECT_DOUBLE_EQ(out.command[t], settings.power_limit) << "thruster " << t;
    }
}

TEST(Allocator, PowerLimitCannotExceedTheHardCeiling) {
    const ThrusterAllocator a = marlin();
    ThrusterSettings settings;
    settings.reversed = marlin_reversed();
    settings.power_limit = 1.0;
    Vector6d wrench;
    wrench << 200, 0, 200, 0, 0, 0;  // far beyond reach: every thruster saturates
    const ThrusterOutput out = drive_thrusters(a, wrench, settings, {}, 0.02);
    double largest = 0.0;
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        largest = std::max(largest, std::fabs(out.command[t]));
    }
    EXPECT_NEAR(largest, MAX_POWER_LIMIT, 1e-12);
}

TEST(Allocator, NaNInputsTurnThrustersOffInsteadOfFullOn) {
    const ThrusterAllocator a = marlin();
    ThrusterSettings settings;
    settings.reversed = marlin_reversed();
    Vector6d wrench;
    wrench << 15, 5, 30, 0, 0, 3;

    Vector6d broken = wrench;
    broken[YAW] = NAN;
    ThrusterOutput out = drive_thrusters(a, broken, settings, {}, 0.02);
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        EXPECT_EQ(out.command[t], 0.0) << "thruster " << t;
    }
    EXPECT_TRUE(out.achieved.allFinite());

    ThrusterSettings no_limit = settings;
    no_limit.power_limit = NAN;
    out = drive_thrusters(a, wrench, no_limit, {}, 0.02);
    for (int t = 0; t < NUM_THRUSTERS; ++t) {
        EXPECT_EQ(out.command[t], 0.0) << "thruster " << t;
    }

    // A NaN health is a failed thruster, allocated around like one.
    settings.health[2] = NAN;
    out = drive_thrusters(a, wrench, settings, {}, 0.02);
    EXPECT_EQ(out.command[2], 0.0);
    EXPECT_EQ(out.force[2], 0.0);
    EXPECT_TRUE(out.achieved.isApprox(wrench, 2e-2));
}
