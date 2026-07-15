#ifndef SUB_CONTROL_SUB_CONTROL_FF_HPP_
#define SUB_CONTROL_SUB_CONTROL_FF_HPP_

#include "sub_control/feedforward_core.hpp"
#include "sub_control_interfaces/action/control_setpoint.hpp"
#include "sub_control_interfaces/msg/error.hpp"
#include "sub_control_interfaces/msg/setpoint.hpp"
#include "sub_control_interfaces/msg/spin.hpp"

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class SubControlFF : public rclcpp::Node {
public:
  explicit SubControlFF(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using FF = sub_control::feedforward::DiagonalMarineModel;
  using Vector6 = sub_control::feedforward::Vector6;
  using Allocator = sub_control::feedforward::ThrusterAllocator6D;
  using ReferenceLimiter = sub_control::feedforward::VelocityReferenceLimiter;
  using FeedbackPI = sub_control::feedforward::FeedbackPI6D;
  using ControlSetpoint = sub_control_interfaces::action::ControlSetpoint;
  using GoalHandleControlSetpoint = rclcpp_action::ServerGoalHandle<ControlSetpoint>;

  static constexpr std::size_t kThrusters = sub_control::feedforward::kThrusters;
  static constexpr double kPositionTolerance = 0.25;
  static constexpr double kAttitudeTolerance = 0.0872665;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void altitude_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void kill_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void pos_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg);
  void att_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg);
  void spin_setpoint_callback(const sub_control_interfaces::msg::Spin::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void apply_pos_setpoint(const sub_control_interfaces::msg::Setpoint & msg);
  void apply_att_setpoint(const sub_control_interfaces::msg::Setpoint & msg);
  void run();
  void publish_zero_thrusters();
  void publish_wrench(
    const rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr & publisher,
    const Vector6 & wrench, const rclcpp::Time & stamp) const;
  void publish_reference(
    const sub_control::feedforward::MotionReference & reference,
    const rclcpp::Time & stamp) const;
  void reset_control_state();
  void rebuild_allocator();
  Vector6 desired_body_velocity(sub_control_interfaces::msg::Error & error_msg);

  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & params);
  rclcpp_action::GoalResponse handle_setpoint_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ControlSetpoint::Goal> goal);
  rclcpp_action::CancelResponse handle_setpoint_cancel(
    const std::shared_ptr<GoalHandleControlSetpoint> goal_handle);
  void execute_setpoint_goal(const std::shared_ptr<GoalHandleControlSetpoint> goal_handle);

  double control_rate_hz_{30.0};
  double power_limit_{0.3};
  double odom_timeout_s_{0.5};
  double cmd_vel_timeout_s_{0.5};
  double antiwindup_gain_{1.0};
  double spin_max_yaw_rate_{1.0};
  double spin_done_angle_{0.05};
  double spin_done_rate_{0.1};
  double spin_decel_{0.5};
  bool feedforward_enabled_{false};
  std::string robot_name_;

  std::array<double, 3> position_kp_{};
  std::array<double, 3> attitude_kp_{};
  Vector6 max_velocity_{};
  Vector6 max_acceleration_{};
  Vector6 feedback_kp_{};
  Vector6 feedback_ki_{};
  Vector6 feedback_integral_limit_{};
  Vector6 positive_wrench_limit_{};
  Vector6 negative_wrench_limit_{};
  FF model_{};

  std::unique_ptr<Allocator> allocator_;
  std::unique_ptr<ReferenceLimiter> reference_limiter_;
  std::unique_ptr<FeedbackPI> feedback_controller_;

  std::array<double, 3> position_{};
  std::array<double, 3> velocity_{};
  std::array<double, 3> attitude_{};
  std::array<double, 3> angular_velocity_{};
  double altitude_{0.0};

  std::array<double, 3> position_setpoint_{};
  std::array<double, 3> velocity_setpoint_{};
  std::array<double, 3> attitude_setpoint_{};
  std::array<double, 3> angular_velocity_setpoint_{};
  bool velocity_mode_{false};
  bool angular_velocity_mode_{false};
  bool altitude_mode_{false};

  bool spin_active_{false};
  bool cmd_vel_active_{false};
  bool spin_ack_pending_{false};
  bool have_unwrapped_yaw_{false};
  double previous_wrapped_yaw_{0.0};
  double unwrapped_yaw_{0.0};
  double spin_target_yaw_{0.0};
  double spin_rate_{0.0};

  bool killed_{true};
  bool have_odom_{false};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  std::mutex state_mutex_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr altitude_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub_;
  rclcpp::Subscription<sub_control_interfaces::msg::Setpoint>::SharedPtr pos_setpoint_sub_;
  rclcpp::Subscription<sub_control_interfaces::msg::Setpoint>::SharedPtr att_setpoint_sub_;
  rclcpp::Subscription<sub_control_interfaces::msg::Spin>::SharedPtr spin_setpoint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kThrusters> thruster_pubs_;
  rclcpp::Publisher<sub_control_interfaces::msg::Error>::SharedPtr error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr feedforward_wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr feedback_wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr commanded_wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr allocated_wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr residual_wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr reference_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr reference_acceleration_pub_;

  rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr set_pose_client_;
  rclcpp_action::Server<ControlSetpoint>::SharedPtr setpoint_action_server_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

#endif  // SUB_CONTROL_SUB_CONTROL_FF_HPP_
