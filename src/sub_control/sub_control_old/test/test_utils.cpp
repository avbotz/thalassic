#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "sub_control/utils.hpp"

TEST(ThrusterAllocator, ReconstructsUnsaturatedWrench) {
    ThrusterAllocator allocator;
    for (int axis = 0; axis < NUM_DOF; ++axis) {
        std::array<double, NUM_DOF> requested{};
        requested[axis] = 1.0;
        const auto forces = allocator.allocate(requested, 1000.0);
        const auto actual = allocator.wrench_from_forces(forces);
        for (int measured_axis = 0; measured_axis < NUM_DOF; ++measured_axis) {
            EXPECT_NEAR(actual[measured_axis], requested[measured_axis], 1e-5);
        }
    }
}

TEST(ThrusterAllocator, RespectsForceBounds) {
    ThrusterAllocator allocator;
    const std::array<double, NUM_DOF> requested{200.0, -200.0, 200.0, 20.0, -20.0, 20.0};
    const auto forces = allocator.allocate(requested, 10.0);
    for (double force : forces) {
        EXPECT_LE(std::fabs(force), 10.0 + 1e-9);
    }
}

TEST(ThrusterAllocator, AxisWeightsChangeSaturatedPriority) {
    ThrusterAllocator allocator;
    const std::array<double, NUM_DOF> requested{100.0, 0.0, 0.0, 0.0, 0.0, 20.0};
    const std::array<double, NUM_DOF> uniform_weights{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    const std::array<double, NUM_DOF> yaw_priority{1.0, 1.0, 1.0, 1.0, 1.0, 10.0};

    const auto uniform =
        allocator.wrench_from_forces(allocator.allocate(requested, 10.0, uniform_weights));
    const auto prioritized =
        allocator.wrench_from_forces(allocator.allocate(requested, 10.0, yaw_priority));

    EXPECT_GT(std::fabs(prioritized[5]), std::fabs(uniform[5]) + 0.5);
}

TEST(ThrusterCurve, PreservesForceSign) {
    EXPECT_DOUBLE_EQ(force_to_norm(0.0), 0.0);
    EXPECT_GT(force_to_norm(10.0), 0.0);
    EXPECT_LT(force_to_norm(-10.0), 0.0);
}

TEST(ThrusterCurve, RoundTripsCommands) {
    for (double command : {-1.0, -0.6, -0.2, 0.0, 0.2, 0.6, 1.0}) {
        const double force = norm_to_force(command);
        if (std::fabs(force) > 1e-9) {
            EXPECT_NEAR(force_to_norm(force), command, 1e-9);
        }
    }
}

TEST(AttitudeError, ReturnsBodyRotationVector) {
    const std::array<double, 3> current{0.0, 0.0, 0.0};
    const std::array<double, 3> target{0.0, 0.0, M_PI / 2.0};
    const auto error = attitude_error(target, current);
    EXPECT_NEAR(error[0], 0.0, 1e-9);
    EXPECT_NEAR(error[1], 0.0, 1e-9);
    EXPECT_NEAR(error[2], M_PI / 2.0, 1e-9);
}
