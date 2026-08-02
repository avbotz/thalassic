#include "sub_control/feedforward_core.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace ff = sub_control::feedforward;

TEST(ThrusterAllocator6D, MarlinMatrixMatchesWorkingMcuSigns) {
    const ff::ThrusterAllocator6D allocator(ff::marlin_v2_thruster_geometry(25.0, 19.0));
    const auto & matrix = allocator.matrix();

    const std::array<std::array<int, ff::kThrusters>, ff::kDof> expected_signs{{
    {{0, 0, 0, 0, 1, -1, -1, 1}},
    {{0, 0, 0, 0, -1, -1, -1, -1}},
    {{-1, 1, 1, -1, 0, 0, 0, 0}},
    {{-1, -1, 1, 1, 0, 0, 0, 0}},
    {{1, -1, 1, -1, 0, 0, 0, 0}},
    {{0, 0, 0, 0, -1, -1, 1, 1}},
  }};

    for (std::size_t axis = 0; axis < ff::kDof; ++axis) {
    for (std::size_t thruster = 0; thruster < ff::kThrusters; ++thruster) {
      const int sign = matrix[axis][thruster] >
        1e-9 ? 1 : (matrix[axis][thruster] < -1e-9 ? -1 : 0);
      EXPECT_EQ(sign,
        expected_signs[axis][thruster]) << "axis=" << axis << " thruster=" << thruster;
    }
    }
}

TEST(ThrusterAllocator6D, ReproducesUnsaturatedWrench) {
    const ff::ThrusterAllocator6D allocator(ff::marlin_v2_thruster_geometry(25.0, 19.0));
    const ff::Vector6 desired{3.0, -2.0, 4.0, 0.4, -0.3, 0.5};
    const auto result = allocator.allocate(desired);

    for (std::size_t axis = 0; axis < ff::kDof; ++axis) {
    EXPECT_NEAR(result.achieved_wrench[axis], desired[axis], 1e-4) << "axis=" << axis;
    EXPECT_NEAR(result.residual_wrench[axis], 0.0, 1e-4) << "axis=" << axis;
    }
}

TEST(ThrusterAllocator6D, RespectsAsymmetricThrusterLimits) {
    const ff::ThrusterAllocator6D allocator(ff::marlin_v2_thruster_geometry(2.0, 1.0));
    const auto result = allocator.allocate({100.0, -100.0, 100.0, 50.0, 50.0, 50.0});

    bool any_saturated = false;
    for (std::size_t thruster = 0; thruster < ff::kThrusters; ++thruster) {
    EXPECT_LE(result.thrust_n[thruster], 2.0 + 1e-9);
    EXPECT_GE(result.thrust_n[thruster], -1.0 - 1e-9);
    any_saturated = any_saturated || result.saturated[thruster];
    }
    EXPECT_TRUE(any_saturated);
}

TEST(ThrusterAllocator6D, PureAxisCommandsHaveExpectedChannelDirections) {
    const ff::ThrusterAllocator6D allocator(ff::marlin_v2_thruster_geometry(25.0, 19.0));
    const std::array<std::array<int, ff::kThrusters>, ff::kDof> expected{{
    {{0, 0, 0, 0, 1, -1, -1, 1}},
    {{0, 0, 0, 0, -1, -1, -1, -1}},
    {{-1, 1, 1, -1, 0, 0, 0, 0}},
    {{-1, -1, 1, 1, 0, 0, 0, 0}},
    {{1, -1, 1, -1, 0, 0, 0, 0}},
    {{0, 0, 0, 0, -1, -1, 1, 1}},
  }};

    for (std::size_t axis = 0; axis < ff::kDof; ++axis) {
    ff::Vector6 desired{};
    desired[axis] = axis < 3 ? 2.0 : 0.25;
    const auto result = allocator.allocate(desired);
    EXPECT_NEAR(result.achieved_wrench[axis], desired[axis], 1e-4);
    for (std::size_t thruster = 0; thruster < ff::kThrusters; ++thruster) {
      const int sign = result.thrust_n[thruster] >
        1e-6 ? 1 : (result.thrust_n[thruster] < -1e-6 ? -1 : 0);
      EXPECT_EQ(sign, expected[axis][thruster]) << "axis=" << axis << " thruster=" << thruster;
    }
    }
}

TEST(ThrusterAllocator6D, RandomizedAllocationMaintainsBoundsAndResidualIdentity) {
    constexpr double forward_limit = 25.0;
    constexpr double reverse_limit = 19.0;
    const ff::ThrusterAllocator6D allocator(
    ff::marlin_v2_thruster_geometry(forward_limit, reverse_limit));
    std::mt19937 generator(20260714);
    std::uniform_real_distribution<double> request(-200.0, 200.0);

    for (int sample = 0; sample < 500; ++sample) {
    ff::Vector6 desired{};
    for (double & value : desired) {
      value = request(generator);
    }
    const auto result = allocator.allocate(desired);
    EXPECT_EQ(result.achieved_wrench, allocator.wrench_from_thrust(result.thrust_n));
    for (std::size_t thruster = 0; thruster < ff::kThrusters; ++thruster) {
      EXPECT_LE(result.thrust_n[thruster], forward_limit + 1e-9);
      EXPECT_GE(result.thrust_n[thruster], -reverse_limit - 1e-9);
      EXPECT_TRUE(std::isfinite(result.thrust_n[thruster]));
    }
    for (std::size_t axis = 0; axis < ff::kDof; ++axis) {
      EXPECT_NEAR(result.residual_wrench[axis], desired[axis] - result.achieved_wrench[axis],
        1e-12);
    }
    }
}

TEST(ThrusterAllocator6D, RejectsInvalidGeometryAndRequests) {
    auto geometry = ff::marlin_v2_thruster_geometry(25.0, 19.0);
    geometry[0].positive_direction_flu = {};
    EXPECT_THROW(ff::ThrusterAllocator6D{geometry}, std::invalid_argument);

    const ff::ThrusterAllocator6D allocator(ff::marlin_v2_thruster_geometry(25.0, 19.0));
    ff::Vector6 desired{};
    desired[2] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(allocator.allocate(desired), std::invalid_argument);
    ff::Vector6 weights{};
    weights.fill(1.0);
    weights[4] = 0.0;
    EXPECT_THROW(allocator.allocate({}, weights), std::invalid_argument);
}

TEST(DiagonalMarineModel, ComputesMassDragAndTrim) {
    ff::DiagonalMarineModel model;
    model.effective_mass.fill(2.0);
    model.linear_drag.fill(3.0);
    model.quadratic_drag.fill(4.0);
    model.trim.fill(0.5);

    const auto wrench = model.feedforward({2.0, -2.0, 0.0, 1.0, -1.0, 0.5},
      {1.0, -1.0, 0.0, 0.5, -0.5, 0.25});
    EXPECT_DOUBLE_EQ(wrench[0], 2.0 + 6.0 + 16.0 + 0.5);
    EXPECT_DOUBLE_EQ(wrench[1], -2.0 - 6.0 - 16.0 + 0.5);
    EXPECT_DOUBLE_EQ(wrench[2], 0.5);
}

TEST(DiagonalMarineModel, AddsConfigurationRestoringWrench) {
    ff::DiagonalMarineModel model;
    model.restoring_stiffness = {0.0, 0.0, 0.0, 50.0, 60.0, 0.0};
    ff::Vector6 configuration{};
    configuration[3] = 0.2;
    configuration[4] = -0.1;

    const auto wrench = model.feedforward({}, {}, configuration);

    EXPECT_NEAR(wrench[3], 50.0 * std::sin(0.2), 1e-9);
    EXPECT_NEAR(wrench[4], 60.0 * std::sin(-0.1), 1e-9);
}

TEST(DiagonalMarineModel, DragOpposesTheRequiredMotionInBothDirections) {
    ff::DiagonalMarineModel model;
    model.linear_drag.fill(3.0);
    model.quadratic_drag.fill(4.0);

    const auto positive = model.feedforward({2.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {});
    const auto negative = model.feedforward({-2.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {});
    EXPECT_DOUBLE_EQ(positive[0], 22.0);
    EXPECT_DOUBLE_EQ(negative[0], -22.0);
}

TEST(DiagonalMarineModel, RejectsNonFiniteInputs) {
    ff::DiagonalMarineModel model;
    ff::Vector6 velocity{};
    velocity[0] = std::numeric_limits<double>::infinity();
    EXPECT_THROW(model.feedforward(velocity, {}), std::invalid_argument);
    model.trim[1] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(model.feedforward({}, {}), std::invalid_argument);
}

TEST(VelocityReferenceLimiter, ProducesBoundedAcceleration) {
    ff::VelocityReferenceLimiter limiter({1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    const auto first = limiter.update({10.0, -10.0, 0.1, 0.0, 0.0, 0.0}, 0.1);
    EXPECT_NEAR(first.velocity[0], 0.1, 1e-12);
    EXPECT_NEAR(first.velocity[1], -0.2, 1e-12);
    EXPECT_NEAR(first.velocity[2], 0.1, 1e-12);
    EXPECT_NEAR(first.acceleration[0], 1.0, 1e-12);
    EXPECT_NEAR(first.acceleration[1], -2.0, 1e-12);

    limiter.reset({1.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    const auto second = limiter.update({1.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.1);
    EXPECT_DOUBLE_EQ(second.acceleration[0], 0.0);
}

TEST(VelocityReferenceLimiter, LandsOnTargetWithoutOvershoot) {
    ff::Vector6 acceleration_limit{};
    acceleration_limit.fill(1.0);
    ff::VelocityReferenceLimiter limiter(acceleration_limit);
    ff::Vector6 target{};
    target[0] = 0.25;

    const auto first = limiter.update(target, 0.2);
    const auto second = limiter.update(target, 0.2);
    const auto third = limiter.update(target, 0.2);
    EXPECT_DOUBLE_EQ(first.velocity[0], 0.2);
    EXPECT_DOUBLE_EQ(second.velocity[0], 0.25);
    EXPECT_DOUBLE_EQ(third.velocity[0], 0.25);
    EXPECT_NEAR(second.acceleration[0], 0.25, 1e-12);
    EXPECT_DOUBLE_EQ(third.acceleration[0], 0.0);
}

TEST(VelocityReferenceLimiter, RejectsInvalidLimitsTargetsAndTime) {
    ff::Vector6 limits{};
    limits.fill(1.0);
    limits[3] = 0.0;
    EXPECT_THROW(ff::VelocityReferenceLimiter{limits}, std::invalid_argument);

    limits[3] = 1.0;
    ff::VelocityReferenceLimiter limiter(limits);
    EXPECT_THROW(limiter.update({}, 0.0), std::invalid_argument);
    ff::Vector6 target{};
    target[5] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(limiter.update(target, 0.1), std::invalid_argument);
    EXPECT_THROW(limiter.reset(target), std::invalid_argument);
}

TEST(FeedbackPI6D, UsesAuthorityRemainingAfterFeedforward) {
    ff::Vector6 kp{};
    ff::Vector6 ki{};
    ff::Vector6 integral_limit{};
    ff::Vector6 positive_limit{};
    ff::Vector6 negative_limit{};
    kp.fill(2.0);
    ki.fill(1.0);
    integral_limit.fill(10.0);
    positive_limit.fill(5.0);
    negative_limit.fill(5.0);
    ff::FeedbackPI6D controller(kp, ki, integral_limit);

    ff::Vector6 error{};
    ff::Vector6 feedforward{};
    error[0] = 10.0;
    feedforward[0] = 4.0;
    const auto result = controller.update(error, feedforward, positive_limit, negative_limit, 1.0);

    EXPECT_DOUBLE_EQ(result.effort[0], 1.0);
    EXPECT_DOUBLE_EQ(result.integral[0], 0.0);
}

TEST(FeedbackPI6D, BackCalculatesAllocatorResidual) {
    ff::Vector6 kp{};
    ff::Vector6 ki{};
    ff::Vector6 integral_limit{};
    ff::Vector6 limit{};
    ki.fill(1.0);
    integral_limit.fill(10.0);
    limit.fill(100.0);
    ff::FeedbackPI6D controller(kp, ki, integral_limit);

    ff::Vector6 error{};
    error[0] = 2.0;
    controller.update(error, {}, limit, limit, 1.0);

    ff::Vector6 residual{};
    residual[0] = 1.0;
    controller.apply_allocation_residual(residual, 1.0, 0.5);
    const auto result = controller.update({}, {}, limit, limit, 1.0);
    EXPECT_DOUBLE_EQ(result.integral[0], 1.5);
}

TEST(FeedbackPI6D, IntegralIsBoundedAndCanUnwindFromSaturation) {
    ff::Vector6 kp{};
    ff::Vector6 ki{};
    ff::Vector6 integral_limit{};
    ff::Vector6 authority{};
    ki.fill(1.0);
    integral_limit.fill(2.0);
    authority.fill(100.0);
    ff::FeedbackPI6D controller(kp, ki, integral_limit);
    ff::Vector6 error{};
    error[0] = 10.0;

    const auto charged = controller.update(error, {}, authority, authority, 1.0);
    EXPECT_DOUBLE_EQ(charged.integral[0], 2.0);
    error[0] = -1.0;
    const auto unwound = controller.update(error, {}, authority, authority, 1.0);
    EXPECT_DOUBLE_EQ(unwound.integral[0], 1.0);
}

TEST(FeedbackPI6D, HandlesNegativeAuthorityAndFeedforwardSaturation) {
    ff::Vector6 kp{};
    ff::Vector6 ki{};
    ff::Vector6 integral_limit{};
    ff::Vector6 positive_limit{};
    ff::Vector6 negative_limit{};
    kp.fill(2.0);
    ki.fill(1.0);
    integral_limit.fill(10.0);
    positive_limit.fill(8.0);
    negative_limit.fill(4.0);
    ff::FeedbackPI6D controller(kp, ki, integral_limit);
    ff::Vector6 error{};
    ff::Vector6 feedforward{};
    error[0] = -10.0;
    feedforward[0] = -3.0;

    const auto saturated = controller.update(error, feedforward, positive_limit, negative_limit,
    1.0);
    EXPECT_DOUBLE_EQ(saturated.effort[0], -1.0);
    EXPECT_DOUBLE_EQ(saturated.integral[0], 0.0);

    error[0] = 1.0;
    const auto recovery = controller.update(error, feedforward, positive_limit, negative_limit,
    1.0);
    EXPECT_GT(recovery.integral[0], 0.0);
}

TEST(FeedbackPI6D, RejectsInvalidConfigurationAndUpdates) {
    ff::Vector6 kp{};
    ff::Vector6 ki{};
    ff::Vector6 integral_limit{};
    kp[0] = -1.0;
    EXPECT_THROW(ff::FeedbackPI6D(kp, ki, integral_limit), std::invalid_argument);

    kp[0] = 0.0;
    ff::FeedbackPI6D controller(kp, ki, integral_limit);
    ff::Vector6 authority{};
    authority.fill(1.0);
    EXPECT_THROW(controller.update({}, {}, authority, authority, 0.0), std::invalid_argument);
    authority[2] = 0.0;
    EXPECT_THROW(controller.update({}, {}, authority, authority, 0.1), std::invalid_argument);
    EXPECT_THROW(controller.apply_allocation_residual({}, 0.1, -1.0), std::invalid_argument);
}
