#include "sub_control/sub_control_ff.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sub_control_interfaces/msg/setpoint.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <format>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{

std::vector<double> six(double value = 0.0) {return std::vector<double>(6, value);}

rclcpp::NodeOptions controller_options()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("control_rate_hz", 100.0);
  options.append_parameter_override("power_limit", 0.6);
  options.append_parameter_override("odom_timeout_s", 0.2);
  options.append_parameter_override("cmd_vel_timeout_s", 2.0);
  options.append_parameter_override("reference.position_kp", std::vector<double>{1.0, 1.0, 1.0});
  options.append_parameter_override("reference.attitude_kp", std::vector<double>{1.0, 1.0, 1.0});
  options.append_parameter_override("reference.max_velocity", six(2.0));
  options.append_parameter_override("reference.max_acceleration", six(20.0));
  options.append_parameter_override("model.effective_mass", six());
  options.append_parameter_override("model.linear_drag", six());
  options.append_parameter_override("model.quadratic_drag", six());
  options.append_parameter_override("model.trim", six());
  options.append_parameter_override("model.restoring_stiffness", six());
  options.append_parameter_override("feedback.kp", six());
  options.append_parameter_override("feedback.ki", six());
  options.append_parameter_override("feedback.integral_limit", six(10.0));
  return options;
}

nav_msgs::msg::Odometry odometry(double yaw = 0.0)
{
  nav_msgs::msg::Odometry message;
  message.pose.pose.orientation.z = std::sin(yaw / 2.0);
  message.pose.pose.orientation.w = std::cos(yaw / 2.0);
  return message;
}

class Harness {
public:
  explicit Harness(const rclcpp::NodeOptions & options)
  : controller(std::make_shared<SubControlFF>(options)),
    driver(std::make_shared<rclcpp::Node>("ff_test_driver"))
  {
    latest_thrusters.fill(std::numeric_limits<double>::quiet_NaN());
    executor.add_node(controller);
    executor.add_node(driver);

    odom_pub = driver->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 10);
    kill_pub = driver->create_publisher<std_msgs::msg::Bool>("kill_switch",
        rclcpp::QoS(1).transient_local());
    cmd_vel_pub = driver->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pos_pub = driver->create_publisher<sub_control_interfaces::msg::Setpoint>("pos_setpoint", 10);
    set_pose_service = driver->create_service<robot_localization::srv::SetPose>(
            "set_pose", [this](const std::shared_ptr<robot_localization::srv::SetPose::Request>,
      std::shared_ptr<robot_localization::srv::SetPose::Response>) {++set_pose_requests;});

    for (std::size_t index = 0; index < latest_thrusters.size(); ++index) {
      thruster_subs[index] = driver->create_subscription<std_msgs::msg::Float64>(
                std::format("control/thruster_{}", index), 10,
        [this, index](const std_msgs::msg::Float64::SharedPtr message) {
          latest_thrusters[index] = message->data;
          ++thruster_updates[index];
                });
    }
    feedforward_sub = driver->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "control/feedforward_wrench", 10,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr message) {
        feedforward = *message;
        ++feedforward_updates;
            });
    command_sub = driver->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "control/commanded_wrench", 10,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr message) {
        command = *message;
        ++command_updates;
            });
    residual_sub = driver->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "control/allocation_residual", 10,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr message) {
        residual = *message;
        ++residual_updates;
            });
    reference_sub = driver->create_subscription<geometry_msgs::msg::TwistStamped>(
            "control/reference_velocity", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr message) {
        reference = *message;
        ++reference_updates;
            });
    pump(100ms);
  }

  ~Harness()
  {
    executor.remove_node(driver);
    executor.remove_node(controller);
  }

  template<typename Publish>
  void pump(std::chrono::milliseconds duration, Publish publish)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      publish();
      executor.spin_some();
      std::this_thread::sleep_for(2ms);
    }
    executor.spin_some();
  }

  void pump(std::chrono::milliseconds duration) {pump(duration, [] {});}

  void publish_odometry_for(
    const nav_msgs::msg::Odometry & message,
    std::chrono::milliseconds duration)
  {
    pump(duration, [this, &message] {odom_pub->publish(message);});
  }

  void release_kill()
  {
    std_msgs::msg::Bool message;
    message.data = false;
    pump(50ms, [this, &message] {kill_pub->publish(message);});
  }

  void arm_with_fresh_odometry(const nav_msgs::msg::Odometry & message = odometry())
  {
    release_kill();
    publish_odometry_for(message, 100ms);
    ASSERT_GT(feedforward_updates, 0U);
  }

  bool thrusters_are_zero(double tolerance = 1e-9) const
  {
    for (std::size_t index = 0; index < latest_thrusters.size(); ++index) {
      if (thruster_updates[index] == 0 || std::abs(latest_thrusters[index]) > tolerance) {
        return false;
      }
    }
    return true;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<SubControlFF> controller;
  rclcpp::Node::SharedPtr driver;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kill_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<sub_control_interfaces::msg::Setpoint>::SharedPtr pos_pub;
  rclcpp::Service<robot_localization::srv::SetPose>::SharedPtr set_pose_service;
  std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, 8> thruster_subs;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr feedforward_sub;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr command_sub;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr residual_sub;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr reference_sub;
  std::array<double, 8> latest_thrusters{};
  std::array<std::size_t, 8> thruster_updates{};
  geometry_msgs::msg::WrenchStamped feedforward;
  geometry_msgs::msg::WrenchStamped command;
  geometry_msgs::msg::WrenchStamped residual;
  geometry_msgs::msg::TwistStamped reference;
  std::size_t feedforward_updates{0};
  std::size_t command_updates{0};
  std::size_t residual_updates{0};
  std::size_t reference_updates{0};
  std::size_t set_pose_requests{0};
};

class FeedforwardNodeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(FeedforwardNodeTest, RequiresFreshOdometryAfterEveryKillRelease) {
    auto options = controller_options();
    options.append_parameter_override("feedforward_enabled", true);
    options.append_parameter_override("model.trim",
      std::vector<double>{0.0, 0.0, -5.0, 0.0, 0.0, 0.0});
    Harness harness(options);

    harness.publish_odometry_for(odometry(), 50ms);
    harness.release_kill();
    harness.pump(100ms);
    EXPECT_TRUE(harness.thrusters_are_zero());
    EXPECT_EQ(harness.feedforward_updates, 0U);

    harness.publish_odometry_for(odometry(), 100ms);
    EXPECT_GT(harness.feedforward_updates, 0U);
    EXPECT_NEAR(harness.feedforward.wrench.force.z, -5.0, 1e-9);
    EXPECT_FALSE(harness.thrusters_are_zero());
    EXPECT_GT(harness.set_pose_requests, 0U);
}

TEST_F(FeedforwardNodeTest, ProducesExpectedSurgeFeedforwardAndThrusterDirections) {
    auto options = controller_options();
    options.append_parameter_override("feedforward_enabled", true);
    options.append_parameter_override("model.linear_drag",
      std::vector<double>{10.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    Harness harness(options);
    harness.arm_with_fresh_odometry();

    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = 0.2;
    harness.pump(150ms, [&] {
      harness.cmd_vel_pub->publish(velocity);
      harness.odom_pub->publish(odometry());
    });

    ASSERT_GT(harness.command_updates, 0U);
    ASSERT_GT(harness.reference_updates, 0U);
    EXPECT_NEAR(harness.reference.twist.linear.x, 0.2, 1e-9);
    EXPECT_NEAR(harness.feedforward.wrench.force.x, 2.0, 1e-9);
    EXPECT_NEAR(harness.command.wrench.force.x, 2.0, 1e-9);
    for (std::size_t index = 0; index < 4; ++index) {
    EXPECT_NEAR(harness.latest_thrusters[index], 0.0, 1e-9);
    }
    EXPECT_GT(harness.latest_thrusters[4], 0.0);
    EXPECT_LT(harness.latest_thrusters[5], 0.0);
    EXPECT_LT(harness.latest_thrusters[6], 0.0);
    EXPECT_GT(harness.latest_thrusters[7], 0.0);
}

TEST_F(FeedforwardNodeTest, OdomWatchdogZerosThrustersAndRejectsInvalidRefreshes) {
    auto options = controller_options();
    options.append_parameter_override("feedforward_enabled", true);
    options.append_parameter_override("odom_timeout_s", 0.05);
    options.append_parameter_override("model.trim",
      std::vector<double>{4.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    Harness harness(options);
    harness.arm_with_fresh_odometry();
    ASSERT_FALSE(harness.thrusters_are_zero());

    auto invalid = odometry();
    invalid.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
    harness.publish_odometry_for(invalid, 100ms);
    EXPECT_TRUE(harness.thrusters_are_zero());
}

TEST_F(FeedforwardNodeTest, SaturatedAllocationNeverExceedsConfiguredPower) {
    auto options = controller_options();
    options.append_parameter_override("feedforward_enabled", true);
    options.append_parameter_override("power_limit", 0.2);
    options.append_parameter_override("model.trim", six(10000.0));
    Harness harness(options);
    harness.arm_with_fresh_odometry();

    bool saturated = false;
    for (double value : harness.latest_thrusters) {
    EXPECT_TRUE(std::isfinite(value));
    EXPECT_LE(std::abs(value), 0.2 + 1e-9);
    saturated = saturated || std::abs(value) > 0.199;
    }
    EXPECT_TRUE(saturated);
    EXPECT_GT(std::abs(harness.residual.wrench.force.x) +
      std::abs(harness.residual.wrench.force.y) +
                  std::abs(harness.residual.wrench.force.z),
              1.0);
}

TEST_F(FeedforwardNodeTest, PositionGuidanceRotatesWorldVelocityIntoBodyFrame) {
    auto options = controller_options();
    options.append_parameter_override("feedforward_enabled", false);
    Harness harness(options);
    const auto yaw_left = odometry(std::numbers::pi / 2.0);
    harness.arm_with_fresh_odometry(yaw_left);

    sub_control_interfaces::msg::Setpoint position;
    position.setpoint.x = 1.0;
    harness.pump(150ms, [&] {
      harness.pos_pub->publish(position);
      harness.odom_pub->publish(yaw_left);
    });

    EXPECT_NEAR(harness.reference.twist.linear.x, 0.0, 1e-6);
    EXPECT_NEAR(harness.reference.twist.linear.y, -1.0, 1e-6);
    EXPECT_NEAR(harness.reference.twist.linear.z, 0.0, 1e-6);
}

TEST_F(FeedforwardNodeTest, RuntimeParameterValidationRejectsUnsafeValues) {
    Harness harness(controller_options());

    EXPECT_FALSE(harness.controller->set_parameter(rclcpp::Parameter("power_limit",
      0.0)).successful);
    EXPECT_FALSE(harness.controller
    ->set_parameter(rclcpp::Parameter("reference.max_acceleration", six(0.0)))
    .successful);
    auto negative_mass = six(1.0);
    negative_mass[2] = -1.0;
    EXPECT_FALSE(harness.controller
    ->set_parameter(rclcpp::Parameter("model.effective_mass", negative_mass))
    .successful);
    auto invalid_trim = six();
    invalid_trim[0] = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(harness.controller->set_parameter(rclcpp::Parameter("model.trim",
      invalid_trim)).successful);
    EXPECT_FALSE(harness.controller->set_parameter(rclcpp::Parameter("antiwindup_gain",
      -1.0)).successful);
}

}  // namespace
