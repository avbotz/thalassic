#include <gtest/gtest.h>

#include "sub_control/controller.hpp"

TEST(StateFeedbackController, SlewsVelocitySetpoint) {
    ControllerConfig config;
    config.position_gain = {10.0, 10.0, 10.0};
    config.max_velocity = {2.0, 2.0, 2.0};
    config.max_linear_acceleration = {1.0, 1.0, 1.0};
    StateFeedbackController controller(config);

    ControlInput input;
    input.position_control = true;
    input.position_error = {10.0, 0.0, 0.0};

    const auto output = controller.update(input, 0.1);
    EXPECT_NEAR(output.velocity_setpoint[0], 0.1, 1e-9);
}

TEST(StateFeedbackController, UsesDirectVelocityMode) {
    ControllerConfig config;
    config.max_linear_acceleration = {100.0, 100.0, 100.0};
    StateFeedbackController controller(config);

    ControlInput input;
    input.position_control = false;
    input.velocity_command = {0.4, -0.3, 0.2};

    const auto output = controller.update(input, 0.1);
    EXPECT_NEAR(output.velocity_setpoint[0], 0.4, 1e-9);
    EXPECT_NEAR(output.velocity_setpoint[1], -0.3, 1e-9);
    EXPECT_NEAR(output.velocity_setpoint[2], 0.2, 1e-9);
}

TEST(StateFeedbackController, ClampsWrenchAndPreventsWindup) {
    ControllerConfig config;
    config.velocity_kp = {100.0, 100.0, 100.0};
    config.velocity_ki = {50.0, 50.0, 50.0};
    config.max_force = {5.0, 5.0, 5.0};
    config.max_linear_acceleration = {100.0, 100.0, 100.0};
    StateFeedbackController controller(config);

    ControlInput input;
    input.velocity_command = {1.0, 0.0, 0.0};
    for (int i = 0; i < 100; ++i) {
        const auto output = controller.update(input, 0.02);
        EXPECT_LE(output.wrench[0], 5.0);
    }

    input.velocity_command = {0.0, 0.0, 0.0};
    const auto recovered = controller.update(input, 0.02);
    EXPECT_NEAR(recovered.wrench[0], 0.0, 1e-9);
}

TEST(StateFeedbackController, AngularRateFeedbackOpposesRotation) {
    ControllerConfig config;
    config.angular_rate_kp = {3.0, 3.0, 4.0};
    config.angular_rate_ki = {0.0, 0.0, 0.0};
    config.max_angular_acceleration = {100.0, 100.0, 100.0};
    StateFeedbackController controller(config);

    ControlInput input;
    input.attitude_control = false;
    input.angular_rate = {0.25, -0.2, 0.15};

    const auto output = controller.update(input, 0.02);
    EXPECT_LT(output.wrench[3], 0.0);
    EXPECT_GT(output.wrench[4], 0.0);
    EXPECT_LT(output.wrench[5], 0.0);
}

TEST(StateFeedbackController, AbsolutePositionRejectsVelocityBias) {
    ControllerConfig config;
    config.position_gain = {0.8, 0.8, 0.8};
    config.position_integral_gain = {0.18, 0.18, 0.18};
    config.velocity_kp = {30.0, 30.0, 30.0};
    config.velocity_ki = {3.0, 3.0, 3.0};
    config.max_force = {30.0, 30.0, 30.0};
    config.max_linear_acceleration = {0.9, 0.9, 0.9};
    StateFeedbackController controller(config);

    double position = 0.0;
    double velocity = 0.0;
    constexpr double dt = 0.02;
    constexpr double velocity_bias = 0.05;

    for (int step = 0; step < 3000; ++step) {
        ControlInput input;
        input.position_control = true;
        input.position_error = {0.5 - position, 0.0, 0.0};
        input.velocity = {velocity + velocity_bias, 0.0, 0.0};
        const auto output = controller.update(input, dt);

        const double acceleration = (output.wrench[0] - 4.0 * velocity) / 20.0;
        velocity += acceleration * dt;
        position += velocity * dt;
    }

    EXPECT_NEAR(position, 0.5, 0.01);
    EXPECT_NEAR(velocity, 0.0, 0.01);
}
