#include "sub_control/sub_control_ff.hpp"

#include "sub_control/utils.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <limits>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

namespace
{

template<std::size_t N>
std::vector<double> vector_from_array(const std::array<double, N> & values)
{
  return {values.begin(), values.end()};
}

template<std::size_t N>
bool array_from_parameter(
  const rclcpp::Parameter & parameter, std::array<double, N> & output, std::string & reason,
  bool positive = false, bool nonnegative = false)
{
  if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    reason = parameter.get_name() + " must be a double array";
    return false;
  }
  const auto values = parameter.as_double_array();
  if (values.size() != N) {
    reason = parameter.get_name() + " must contain " + std::to_string(N) + " values";
    return false;
  }
  for (std::size_t i = 0; i < N; ++i) {
    if (!std::isfinite(values[i]) || (positive && values[i] <= 0.0) ||
      (nonnegative && values[i] < 0.0))
    {
      reason = parameter.get_name() + " contains an invalid value";
      return false;
    }
    output[i] = values[i];
  }
  return true;
}

template<std::size_t N>
std::array<double, N> get_array_parameter(
  const rclcpp::Node & node, const std::string & name, bool positive = false,
  bool nonnegative = false)
{
  std::array<double, N> output{};
  std::string reason;
  if (!array_from_parameter(node.get_parameter(name), output, reason, positive, nonnegative)) {
    throw std::invalid_argument(reason);
  }
  return output;
}

template<typename Container>
bool all_finite(const Container & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);});
}

void set_wrench(
  geometry_msgs::msg::Wrench & message,
  const sub_control::feedforward::Vector6 & wrench)
{
  message.force.x = wrench[0];
  message.force.y = wrench[1];
  message.force.z = wrench[2];
  message.torque.x = wrench[3];
  message.torque.y = wrench[4];
  message.torque.z = wrench[5];
}

}  // namespace

SubControlFF::SubControlFF(const rclcpp::NodeOptions & options)
: Node("sub_control_ff", options)
{
  rcl_interfaces::msg::ParameterDescriptor read_only;
  read_only.read_only = true;

  position_kp_ = {0.3, 0.3, 0.4};
  attitude_kp_ = {0.8, 0.8, 1.0};
  max_velocity_ = {0.35, 0.35, 0.3, 0.5, 0.5, 0.8};
  max_acceleration_ = {0.15, 0.15, 0.12, 0.25, 0.25, 0.35};
  feedback_kp_ = {20.0, 20.0, 10.0, 5.0, 5.0, 10.0};
  feedback_ki_ = {1.0, 1.0, 2.0, 0.2, 0.2, 0.5};
  feedback_integral_limit_ = {20.0, 20.0, 20.0, 10.0, 10.0, 10.0};

  this->declare_parameter("control_rate_hz", control_rate_hz_, read_only);
  this->declare_parameter("robot_name", std::string{}, read_only);
  this->declare_parameter("power_limit", power_limit_);
  this->declare_parameter("odom_timeout_s", odom_timeout_s_);
  this->declare_parameter("cmd_vel_timeout_s", cmd_vel_timeout_s_);
  this->declare_parameter("feedforward_enabled", feedforward_enabled_);
  this->declare_parameter("antiwindup_gain", antiwindup_gain_);
  this->declare_parameter("reference.position_kp", vector_from_array(position_kp_));
  this->declare_parameter("reference.attitude_kp", vector_from_array(attitude_kp_));
  this->declare_parameter("reference.max_velocity", vector_from_array(max_velocity_));
  this->declare_parameter("reference.max_acceleration", vector_from_array(max_acceleration_));
  this->declare_parameter("model.effective_mass", vector_from_array(model_.effective_mass));
  this->declare_parameter("model.linear_drag", vector_from_array(model_.linear_drag));
  this->declare_parameter("model.quadratic_drag", vector_from_array(model_.quadratic_drag));
  this->declare_parameter("model.trim", vector_from_array(model_.trim));
  this->declare_parameter("model.restoring_stiffness",
    vector_from_array(model_.restoring_stiffness));
  this->declare_parameter("feedback.kp", vector_from_array(feedback_kp_));
  this->declare_parameter("feedback.ki", vector_from_array(feedback_ki_));
  this->declare_parameter("feedback.integral_limit", vector_from_array(feedback_integral_limit_));
  this->declare_parameter("spin.max_yaw_rate", spin_max_yaw_rate_);
  this->declare_parameter("spin.done_angle", spin_done_angle_);
  this->declare_parameter("spin.done_rate", spin_done_rate_);
  this->declare_parameter("spin.deceleration", spin_decel_);

  this->get_parameter("control_rate_hz", control_rate_hz_);
  this->get_parameter("robot_name", robot_name_);
  this->get_parameter("power_limit", power_limit_);
  this->get_parameter("odom_timeout_s", odom_timeout_s_);
  this->get_parameter("cmd_vel_timeout_s", cmd_vel_timeout_s_);
  this->get_parameter("feedforward_enabled", feedforward_enabled_);
  this->get_parameter("antiwindup_gain", antiwindup_gain_);
  position_kp_ = get_array_parameter<3>(*this, "reference.position_kp", false, true);
  attitude_kp_ = get_array_parameter<3>(*this, "reference.attitude_kp", false, true);
  max_velocity_ = get_array_parameter<6>(*this, "reference.max_velocity", true);
  max_acceleration_ = get_array_parameter<6>(*this, "reference.max_acceleration", true);
  model_.effective_mass = get_array_parameter<6>(*this, "model.effective_mass", false, true);
  model_.linear_drag = get_array_parameter<6>(*this, "model.linear_drag", false, true);
  model_.quadratic_drag = get_array_parameter<6>(*this, "model.quadratic_drag", false, true);
  model_.trim = get_array_parameter<6>(*this, "model.trim");
  model_.restoring_stiffness = get_array_parameter<6>(*this, "model.restoring_stiffness", false,
    true);
  feedback_kp_ = get_array_parameter<6>(*this, "feedback.kp", false, true);
  feedback_ki_ = get_array_parameter<6>(*this, "feedback.ki", false, true);
  feedback_integral_limit_ = get_array_parameter<6>(*this, "feedback.integral_limit", false, true);
  this->get_parameter("spin.max_yaw_rate", spin_max_yaw_rate_);
  this->get_parameter("spin.done_angle", spin_done_angle_);
  this->get_parameter("spin.done_rate", spin_done_rate_);
  this->get_parameter("spin.deceleration", spin_decel_);

  const std::array<double, 9> scalar_parameters{
    control_rate_hz_, power_limit_, odom_timeout_s_, cmd_vel_timeout_s_, antiwindup_gain_,
    spin_max_yaw_rate_, spin_done_angle_, spin_done_rate_, spin_decel_,
  };
  if (!all_finite(scalar_parameters) || control_rate_hz_ <= 0.0 || power_limit_ <= 0.0 ||
    power_limit_ > 1.0 ||
    odom_timeout_s_ <= 0.0 || cmd_vel_timeout_s_ <= 0.0 || antiwindup_gain_ < 0.0 ||
    spin_max_yaw_rate_ <= 0.0 ||
    spin_done_angle_ <= 0.0 || spin_done_rate_ <= 0.0 || spin_decel_ <= 0.0)
  {
    throw std::invalid_argument("invalid controller timing or power parameters");
  }

  reference_limiter_ = std::make_unique<ReferenceLimiter>(max_acceleration_);
  feedback_controller_ = std::make_unique<FeedbackPI>(feedback_kp_, feedback_ki_,
    feedback_integral_limit_);
  rebuild_allocator();

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      odom_callback(msg);
                                                                                                          });
  altitude_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "altitude", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
      altitude_callback(msg);
                                                                                                    });
  kill_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "kill_switch", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {kill_callback(msg);});
  pos_setpoint_sub_ = this->create_subscription<sub_control_interfaces::msg::Setpoint>(
        "pos_setpoint", 10,
    [this](const sub_control_interfaces::msg::Setpoint::SharedPtr msg) {
      pos_setpoint_callback(msg);
    });
  att_setpoint_sub_ = this->create_subscription<sub_control_interfaces::msg::Setpoint>(
        "att_setpoint", 10,
    [this](const sub_control_interfaces::msg::Setpoint::SharedPtr msg) {
      att_setpoint_callback(msg);
    });
  spin_setpoint_sub_ = this->create_subscription<sub_control_interfaces::msg::Spin>(
        "spin_setpoint", 10,
    [this](const sub_control_interfaces::msg::Spin::SharedPtr msg) {spin_setpoint_callback(msg);});
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      cmd_vel_callback(msg);
                                                                                                     });

  for (std::size_t i = 0; i < kThrusters; ++i) {
    thruster_pubs_[i] =
      this->create_publisher<std_msgs::msg::Float64>(std::format("control/thruster_{}", i), 10);
  }
  error_pub_ = this->create_publisher<sub_control_interfaces::msg::Error>("control/error", 10);
  feedforward_wrench_pub_ =
    this->create_publisher<geometry_msgs::msg::WrenchStamped>("control/feedforward_wrench", 10);
  feedback_wrench_pub_ =
    this->create_publisher<geometry_msgs::msg::WrenchStamped>("control/feedback_wrench", 10);
  commanded_wrench_pub_ =
    this->create_publisher<geometry_msgs::msg::WrenchStamped>("control/commanded_wrench", 10);
  allocated_wrench_pub_ =
    this->create_publisher<geometry_msgs::msg::WrenchStamped>("control/allocated_wrench", 10);
  residual_wrench_pub_ =
    this->create_publisher<geometry_msgs::msg::WrenchStamped>("control/allocation_residual", 10);
  reference_velocity_pub_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>("control/reference_velocity", 10);
  reference_acceleration_pub_ =
    this->create_publisher<geometry_msgs::msg::AccelStamped>("control/reference_acceleration", 10);

  set_pose_client_ = this->create_client<robot_localization::srv::SetPose>("set_pose");
  setpoint_action_server_ = rclcpp_action::create_server<ControlSetpoint>(
        this, "control_setpoint",
    [this](const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ControlSetpoint::Goal> goal) {
      return handle_setpoint_goal(uuid, std::move(goal));
        },
    [this](const std::shared_ptr<GoalHandleControlSetpoint> goal_handle) {
      return handle_setpoint_cancel(goal_handle);
        },
    [this](const std::shared_ptr<GoalHandleControlSetpoint> goal_handle) {
      std::thread{[this, goal_handle] {execute_setpoint_goal(goal_handle);}}.detach();
        });

  param_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {return on_parameters_set(params);});
  control_timer_ = this->create_timer(std::chrono::microseconds{static_cast<int64_t>(1e6 /
        control_rate_hz_)},
      [this]() {run();});
}

void SubControlFF::rebuild_allocator()
{
  const double forward_n = std::max(0.01, norm_to_force(power_limit_));
  const double reverse_n = std::max(0.01, std::abs(norm_to_force(-power_limit_)));
  allocator_ =
    std::make_unique<Allocator>(sub_control::feedforward::marlin_v2_thruster_geometry(forward_n,
    reverse_n));

  for (std::size_t axis = 0; axis < sub_control::feedforward::kDof; ++axis) {
    Vector6 request{};
    request[axis] = 1e6;
    positive_wrench_limit_[axis] = std::max(0.01,
      allocator_->allocate(request).achieved_wrench[axis]);
    request[axis] = -1e6;
    negative_wrench_limit_[axis] = std::max(0.01,
      -allocator_->allocate(request).achieved_wrench[axis]);
  }
}

void SubControlFF::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const std::array<double, 13> values{
    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w, msg->twist.twist.linear.x, msg->twist.twist.linear.y,
    msg->twist.twist.linear.z, msg->twist.twist.angular.x, msg->twist.twist.angular.y,
    msg->twist.twist.angular.z,
  };
  if (!all_finite(values)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Ignoring non-finite odometry");
    return;
  }

  tf2::Quaternion quaternion;
  tf2::fromMsg(msg->pose.pose.orientation, quaternion);
  if (!std::isfinite(quaternion.length2()) || quaternion.length2() < 1e-12) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Ignoring invalid odometry quaternion");
    return;
  }
  quaternion.normalize();
  std::array<double, 3> attitude{};
  tf2::Matrix3x3(quaternion).getRPY(attitude[0], attitude[1], attitude[2]);
  if (!all_finite(attitude)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Ignoring invalid odometry attitude");
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  position_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
  velocity_ = {msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
  angular_velocity_ = {msg->twist.twist.angular.x, msg->twist.twist.angular.y,
    msg->twist.twist.angular.z};
  attitude_ = attitude;
  if (!have_unwrapped_yaw_) {
    unwrapped_yaw_ = attitude_[2];
    have_unwrapped_yaw_ = true;
  } else {
    unwrapped_yaw_ += angles::shortest_angular_distance(previous_wrapped_yaw_, attitude_[2]);
  }
  previous_wrapped_yaw_ = attitude_[2];
  have_odom_ = true;
  last_odom_time_ = this->get_clock()->now();
}

void SubControlFF::altitude_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  if (!std::isfinite(msg->data)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Ignoring non-finite altitude");
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  altitude_ = msg->data;
}

void SubControlFF::reset_control_state()
{
  const Vector6 measured{velocity_[0], velocity_[1], velocity_[2],
    angular_velocity_[0], angular_velocity_[1], angular_velocity_[2]};
  reference_limiter_->reset(measured);
  feedback_controller_->reset();
  last_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void SubControlFF::kill_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (killed_ && !msg->data) {
    auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
    request->pose.header.frame_id = robot_name_.empty() ? "odom" : robot_name_ + "/odom";
    request->pose.header.stamp = this->now();
    request->pose.pose.pose.orientation.w = 1.0;
    set_pose_client_->async_send_request(request);

    position_.fill(0.0);
    velocity_.fill(0.0);
    attitude_.fill(0.0);
    angular_velocity_.fill(0.0);
    position_setpoint_.fill(0.0);
    velocity_setpoint_.fill(0.0);
    attitude_setpoint_.fill(0.0);
    angular_velocity_setpoint_.fill(0.0);
    velocity_mode_ = false;
    angular_velocity_mode_ = false;
    altitude_mode_ = false;
    spin_active_ = false;
    cmd_vel_active_ = false;
    have_unwrapped_yaw_ = false;
    have_odom_ = false;
    last_odom_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    reset_control_state();
  }
  if (msg->data) {
    spin_active_ = false;
    reset_control_state();
  }
  killed_ = msg->data;
}

void SubControlFF::apply_pos_setpoint(const sub_control_interfaces::msg::Setpoint & msg)
{
  cmd_vel_active_ = false;
  if (velocity_mode_ != msg.velocity) {
    reset_control_state();
  }
  velocity_mode_ = msg.velocity;
  altitude_mode_ = msg.altitude;
  if (velocity_mode_) {
    velocity_setpoint_ = {msg.setpoint.x, msg.setpoint.y, msg.setpoint.z};
  } else {
    position_setpoint_ = {msg.setpoint.x, msg.setpoint.y, msg.setpoint.z};
  }
}

void SubControlFF::apply_att_setpoint(const sub_control_interfaces::msg::Setpoint & msg)
{
  cmd_vel_active_ = false;
  if (angular_velocity_mode_ != msg.velocity || spin_active_) {
    reset_control_state();
  }
  spin_active_ = false;
  angular_velocity_mode_ = msg.velocity;
  if (angular_velocity_mode_) {
    angular_velocity_setpoint_ = {msg.setpoint.roll, msg.setpoint.pitch, msg.setpoint.yaw};
  } else {
    attitude_setpoint_ = {angles::normalize_angle(msg.setpoint.roll),
      angles::normalize_angle(msg.setpoint.pitch),
      angles::normalize_angle(msg.setpoint.yaw)};
  }
}

void SubControlFF::pos_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg)
{
  const std::array<double, 3> values{msg->setpoint.x, msg->setpoint.y, msg->setpoint.z};
  if (!all_finite(values)) {
    RCLCPP_WARN(this->get_logger(), "Ignoring non-finite position/velocity setpoint");
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  apply_pos_setpoint(*msg);
}

void SubControlFF::att_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg)
{
  const std::array<double, 3> values{msg->setpoint.roll, msg->setpoint.pitch, msg->setpoint.yaw};
  if (!all_finite(values)) {
    RCLCPP_WARN(this->get_logger(), "Ignoring non-finite attitude/rate setpoint");
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  apply_att_setpoint(*msg);
}

void SubControlFF::spin_setpoint_callback(const sub_control_interfaces::msg::Spin::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (killed_ || !have_odom_ || !std::isfinite(msg->yaw) || std::abs(msg->yaw) < 1e-9) {
    return;
  }
  const double base = spin_active_ ?
    spin_target_yaw_ :
    unwrapped_yaw_ + angles::shortest_angular_distance(attitude_[2], attitude_setpoint_[2]);
  spin_target_yaw_ = base + msg->yaw;
  spin_rate_ = std::min(msg->max_rate > 0.0 ? msg->max_rate : spin_max_yaw_rate_,
    spin_max_yaw_rate_);
  attitude_setpoint_[2] = angles::normalize_angle(spin_target_yaw_);
  angular_velocity_mode_ = false;
  spin_active_ = true;
  spin_ack_pending_ = true;
  reset_control_state();
}

void SubControlFF::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const std::array<double, 6> command{msg->linear.x, msg->linear.y, msg->linear.z,
    msg->angular.x, msg->angular.y, msg->angular.z};
  if (!std::all_of(command.begin(), command.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    RCLCPP_WARN(this->get_logger(), "Ignoring non-finite cmd_vel command");
    return;
  }
  const bool mode_changed = !velocity_mode_ || !angular_velocity_mode_ || spin_active_;
  velocity_mode_ = true;
  angular_velocity_mode_ = true;
  spin_active_ = false;
  velocity_setpoint_ = {msg->linear.x, msg->linear.y, msg->linear.z};
  angular_velocity_setpoint_ = {msg->angular.x, msg->angular.y, msg->angular.z};
  cmd_vel_active_ = true;
  last_cmd_vel_time_ = this->get_clock()->now();
  if (mode_changed) {
    RCLCPP_INFO(this->get_logger(), "cmd_vel took control of all six velocity axes");
  }
  if (mode_changed) {
    reset_control_state();
  }
}

SubControlFF::Vector6 SubControlFF::desired_body_velocity(
  sub_control_interfaces::msg::Error & error_msg)
{
  Vector6 desired{};
  if (velocity_mode_) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      desired[axis] = std::clamp(velocity_setpoint_[axis], -max_velocity_[axis],
        max_velocity_[axis]);
    }
  } else {
    const double vertical = altitude_mode_ ? altitude_ : position_[2];
    const std::array<double, 3> error{position_setpoint_[0] - position_[0],
      position_setpoint_[1] - position_[1],
      position_setpoint_[2] - vertical};
    tf2::Vector3 world_velocity;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      error_msg.pos_error[axis] = error[axis];
      world_velocity[axis] =
        std::clamp(position_kp_[axis] * error[axis], -max_velocity_[axis], max_velocity_[axis]);
    }
    tf2::Quaternion orientation;
    orientation.setRPY(attitude_[0], attitude_[1], attitude_[2]);
    const tf2::Vector3 body_velocity = tf2::quatRotate(orientation.inverse(), world_velocity);
    desired[0] = std::clamp(body_velocity.x(), -max_velocity_[0], max_velocity_[0]);
    desired[1] = std::clamp(body_velocity.y(), -max_velocity_[1], max_velocity_[1]);
    desired[2] = std::clamp(body_velocity.z(), -max_velocity_[2], max_velocity_[2]);
  }

  if (angular_velocity_mode_) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      desired[axis + 3] =
        std::clamp(angular_velocity_setpoint_[axis], -max_velocity_[axis + 3],
        max_velocity_[axis + 3]);
    }
    return desired;
  }

  std::array<double, 3> attitude_error_value{};
  if (spin_active_) {
    const double remaining = spin_target_yaw_ - unwrapped_yaw_;
    attitude_error_value = attitude_error({attitude_setpoint_[0], attitude_setpoint_[1],
          attitude_[2]}, attitude_);
    attitude_error_value[2] = remaining;
    const double braking_rate = std::sqrt(std::max(0.0, 2.0 * spin_decel_ * std::abs(remaining)));
    const double yaw_limit = std::min({spin_rate_, max_velocity_[5], braking_rate});
    desired[5] = std::clamp(attitude_kp_[2] * remaining, -yaw_limit, yaw_limit);
  } else {
    attitude_error_value = attitude_error(attitude_setpoint_, attitude_);
    desired[5] = std::clamp(attitude_kp_[2] * attitude_error_value[2], -max_velocity_[5],
      max_velocity_[5]);
  }
  for (std::size_t axis = 0; axis < 2; ++axis) {
    desired[axis + 3] = std::clamp(attitude_kp_[axis] * attitude_error_value[axis],
      -max_velocity_[axis + 3],
                                       max_velocity_[axis + 3]);
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    error_msg.att_error[axis] = attitude_error_value[axis];
  }
  return desired;
}

void SubControlFF::publish_zero_thrusters()
{
  std_msgs::msg::Float64 message;
  message.data = 0.0;
  for (auto & publisher : thruster_pubs_) {
    publisher->publish(message);
  }
}

void SubControlFF::publish_wrench(
  const rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr & publisher,
  const Vector6 & wrench, const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::WrenchStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = robot_name_.empty() ? "base_link" : robot_name_ + "/base_link";
  set_wrench(message.wrench, wrench);
  publisher->publish(message);
}

void SubControlFF::publish_reference(
  const sub_control::feedforward::MotionReference & reference,
  const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::TwistStamped velocity_message;
  velocity_message.header.stamp = stamp;
  velocity_message.header.frame_id = robot_name_.empty() ? "base_link" : robot_name_ + "/base_link";
  velocity_message.twist.linear.x = reference.velocity[0];
  velocity_message.twist.linear.y = reference.velocity[1];
  velocity_message.twist.linear.z = reference.velocity[2];
  velocity_message.twist.angular.x = reference.velocity[3];
  velocity_message.twist.angular.y = reference.velocity[4];
  velocity_message.twist.angular.z = reference.velocity[5];
  reference_velocity_pub_->publish(velocity_message);

  geometry_msgs::msg::AccelStamped acceleration_message;
  acceleration_message.header = velocity_message.header;
  acceleration_message.accel.linear.x = reference.acceleration[0];
  acceleration_message.accel.linear.y = reference.acceleration[1];
  acceleration_message.accel.linear.z = reference.acceleration[2];
  acceleration_message.accel.angular.x = reference.acceleration[3];
  acceleration_message.accel.angular.y = reference.acceleration[4];
  acceleration_message.accel.angular.z = reference.acceleration[5];
  reference_acceleration_pub_->publish(acceleration_message);
}

void SubControlFF::run()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const rclcpp::Time now = this->get_clock()->now();
  if (killed_ || !have_odom_ || (now - last_odom_time_).seconds() > odom_timeout_s_) {
    publish_zero_thrusters();
    reset_control_state();
    return;
  }
  if (last_update_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
    last_update_time_ = now;
    return;
  }
  const double dt = std::clamp((now - last_update_time_).seconds(), 1e-4, 0.2);
  last_update_time_ = now;

  if (cmd_vel_active_ && (now - last_cmd_vel_time_).seconds() > cmd_vel_timeout_s_) {
    velocity_setpoint_.fill(0.0);
    angular_velocity_setpoint_.fill(0.0);
    cmd_vel_active_ = false;
  }

  if (spin_active_ && !spin_ack_pending_) {
    const double remaining = spin_target_yaw_ - unwrapped_yaw_;
    if (std::abs(remaining) <= spin_done_angle_ &&
      std::abs(angular_velocity_[2]) <= spin_done_rate_)
    {
      spin_active_ = false;
    }
  }

  sub_control_interfaces::msg::Error error_msg;
  error_msg.header.stamp = now;
  const Vector6 desired_velocity = desired_body_velocity(error_msg);
  const auto reference = reference_limiter_->update(desired_velocity, dt);

  Vector6 feedforward{};
  if (feedforward_enabled_) {
    const Vector6 configuration{position_[0], position_[1], position_[2], attitude_[0],
      attitude_[1], attitude_[2]};
    feedforward = model_.feedforward(reference.velocity, reference.acceleration, configuration);
  }
  const Vector6 measured{velocity_[0], velocity_[1], velocity_[2],
    angular_velocity_[0], angular_velocity_[1], angular_velocity_[2]};
  Vector6 velocity_error{};
  for (std::size_t axis = 0; axis < sub_control::feedforward::kDof; ++axis) {
    velocity_error[axis] = reference.velocity[axis] - measured[axis];
    if (axis < 3) {
      error_msg.vel_error[axis] = velocity_error[axis];
    } else {
      error_msg.angvel_error[axis - 3] = velocity_error[axis];
    }
  }
  const auto feedback =
    feedback_controller_->update(velocity_error, feedforward, positive_wrench_limit_,
    negative_wrench_limit_, dt);
  Vector6 command{};
  for (std::size_t axis = 0; axis < sub_control::feedforward::kDof; ++axis) {
    command[axis] = std::clamp(feedforward[axis] + feedback.effort[axis],
      -negative_wrench_limit_[axis],
                                   positive_wrench_limit_[axis]);
  }
  const auto allocation = allocator_->allocate(command);
  feedback_controller_->apply_allocation_residual(allocation.residual_wrench, dt, antiwindup_gain_);

  for (std::size_t thruster = 0; thruster < kThrusters; ++thruster) {
    std_msgs::msg::Float64 message;
    message.data = std::clamp(force_to_norm(allocation.thrust_n[thruster]), -power_limit_,
      power_limit_);
    thruster_pubs_[thruster]->publish(message);
  }

  error_msg.spin_active = spin_active_;
  error_pub_->publish(error_msg);
  spin_ack_pending_ = false;
  publish_reference(reference, now);
  publish_wrench(feedforward_wrench_pub_, feedforward, now);
  publish_wrench(feedback_wrench_pub_, feedback.effort, now);
  publish_wrench(commanded_wrench_pub_, command, now);
  publish_wrench(allocated_wrench_pub_, allocation.achieved_wrench, now);
  publish_wrench(residual_wrench_pub_, allocation.residual_wrench, now);
}

rcl_interfaces::msg::SetParametersResult SubControlFF::on_parameters_set(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;

  auto position_kp = position_kp_;
  auto attitude_kp = attitude_kp_;
  auto max_velocity = max_velocity_;
  auto max_acceleration = max_acceleration_;
  auto feedback_kp = feedback_kp_;
  auto feedback_ki = feedback_ki_;
  auto integral_limit = feedback_integral_limit_;
  auto model = model_;
  double power_limit = power_limit_;
  double odom_timeout = odom_timeout_s_;
  double cmd_vel_timeout = cmd_vel_timeout_s_;
  double antiwindup_gain = antiwindup_gain_;
  double max_yaw_rate = spin_max_yaw_rate_;
  double done_angle = spin_done_angle_;
  double done_rate = spin_done_rate_;
  double deceleration = spin_decel_;
  bool feedforward_enabled = feedforward_enabled_;

  for (const auto & parameter : params) {
    const auto & name = parameter.get_name();
    if (name == "reference.position_kp") {
      if (!array_from_parameter(parameter, position_kp, result.reason, false, true)) {
        return result;
      }
    } else if (name == "reference.attitude_kp") {
      if (!array_from_parameter(parameter, attitude_kp, result.reason, false, true)) {
        return result;
      }
    } else if (name == "reference.max_velocity") {
      if (!array_from_parameter(parameter, max_velocity, result.reason, true)) {
        return result;
      }
    } else if (name == "reference.max_acceleration") {
      if (!array_from_parameter(parameter, max_acceleration, result.reason, true)) {
        return result;
      }
    } else if (name == "model.effective_mass") {
      if (!array_from_parameter(parameter, model.effective_mass, result.reason, false, true)) {
        return result;
      }
    } else if (name == "model.linear_drag") {
      if (!array_from_parameter(parameter, model.linear_drag, result.reason, false, true)) {
        return result;
      }
    } else if (name == "model.quadratic_drag") {
      if (!array_from_parameter(parameter, model.quadratic_drag, result.reason, false, true)) {
        return result;
      }
    } else if (name == "model.trim") {
      if (!array_from_parameter(parameter, model.trim, result.reason)) {
        return result;
      }
    } else if (name == "model.restoring_stiffness") {
      if (!array_from_parameter(parameter, model.restoring_stiffness, result.reason, false, true)) {
        return result;
      }
    } else if (name == "feedback.kp") {
      if (!array_from_parameter(parameter, feedback_kp, result.reason, false, true)) {
        return result;
      }
    } else if (name == "feedback.ki") {
      if (!array_from_parameter(parameter, feedback_ki, result.reason, false, true)) {
        return result;
      }
    } else if (name == "feedback.integral_limit") {
      if (!array_from_parameter(parameter, integral_limit, result.reason, false, true)) {
        return result;
      }
    } else if (name == "feedforward_enabled") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.reason = name + " must be bool";
        return result;
      }
      feedforward_enabled = parameter.as_bool();
    } else {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.reason = name + " must be double";
        return result;
      }
      const double value = parameter.as_double();
      if (!std::isfinite(value)) {
        result.reason = name + " must be finite";
        return result;
      }
      if (name == "power_limit") {
        power_limit = value;
      } else if (name == "odom_timeout_s") {
        odom_timeout = value;
      } else if (name == "cmd_vel_timeout_s") {
        cmd_vel_timeout = value;
      } else if (name == "antiwindup_gain") {
        antiwindup_gain = value;
      } else if (name == "spin.max_yaw_rate") {
        max_yaw_rate = value;
      } else if (name == "spin.done_angle") {
        done_angle = value;
      } else if (name == "spin.done_rate") {
        done_rate = value;
      } else if (name == "spin.deceleration") {
        deceleration = value;
      } else {
        result.reason = "unsupported runtime parameter " + name;
        return result;
      }
    }
  }

  if (power_limit <= 0.0 || power_limit > 1.0 || odom_timeout <= 0.0 || cmd_vel_timeout <= 0.0 ||
    antiwindup_gain < 0.0 || max_yaw_rate <= 0.0 || done_angle <= 0.0 || done_rate <= 0.0 ||
    deceleration <= 0.0)
  {
    result.reason = "scalar controller parameters are outside their valid range";
    return result;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  position_kp_ = position_kp;
  attitude_kp_ = attitude_kp;
  max_velocity_ = max_velocity;
  max_acceleration_ = max_acceleration;
  feedback_kp_ = feedback_kp;
  feedback_ki_ = feedback_ki;
  feedback_integral_limit_ = integral_limit;
  model_ = model;
  odom_timeout_s_ = odom_timeout;
  cmd_vel_timeout_s_ = cmd_vel_timeout;
  antiwindup_gain_ = antiwindup_gain;
  spin_max_yaw_rate_ = max_yaw_rate;
  spin_done_angle_ = done_angle;
  spin_done_rate_ = done_rate;
  spin_decel_ = deceleration;
  feedforward_enabled_ = feedforward_enabled;
  reference_limiter_->set_max_acceleration(max_acceleration_);
  feedback_controller_->configure(feedback_kp_, feedback_ki_, feedback_integral_limit_);
  if (power_limit != power_limit_) {
    power_limit_ = power_limit;
    rebuild_allocator();
  }
  result.successful = true;
  return result;
}

rclcpp_action::GoalResponse SubControlFF::handle_setpoint_goal(
  const rclcpp_action::GoalUUID &, const std::shared_ptr<const ControlSetpoint::Goal> goal)
{
  if (goal->command_type > ControlSetpoint::Goal::ATTITUDE) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->command_type == ControlSetpoint::Goal::VELOCITY) {
    const std::array<double, 3> values{
      goal->setpoint.setpoint.x, goal->setpoint.setpoint.y, goal->setpoint.setpoint.z};
    if (!all_finite(values)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SubControlFF::handle_setpoint_cancel(
  const std::shared_ptr<GoalHandleControlSetpoint>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SubControlFF::execute_setpoint_goal(
  const std::shared_ptr<GoalHandleControlSetpoint> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const auto result = std::make_shared<ControlSetpoint::Result>();
  std::array<bool, 3> active_axes{};
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (killed_) {
      result->message = "kill switch is engaged";
      goal_handle->abort(result);
      return;
    }
    if (goal->command_type == ControlSetpoint::Goal::POSITION) {
      active_axes = {std::isfinite(goal->setpoint.setpoint.x),
        std::isfinite(goal->setpoint.setpoint.y),
        std::isfinite(goal->setpoint.setpoint.z)};
      auto command = goal->setpoint;
      command.velocity = false;
      if (!active_axes[0]) {
        command.setpoint.x = position_setpoint_[0];
      }
      if (!active_axes[1]) {
        command.setpoint.y = position_setpoint_[1];
      }
      if (!active_axes[2]) {
        command.setpoint.z = position_setpoint_[2];
      }
      apply_pos_setpoint(command);
    } else if (goal->command_type == ControlSetpoint::Goal::VELOCITY) {
      auto command = goal->setpoint;
      command.velocity = true;
      apply_pos_setpoint(command);
      result->message = "velocity setpoint accepted";
      goal_handle->succeed(result);
      return;
    } else {
      active_axes = {std::isfinite(goal->setpoint.setpoint.roll),
        std::isfinite(goal->setpoint.setpoint.pitch),
        std::isfinite(goal->setpoint.setpoint.yaw)};
      auto command = goal->setpoint;
      command.velocity = false;
      if (!active_axes[0]) {
        command.setpoint.roll = attitude_setpoint_[0];
      }
      if (!active_axes[1]) {
        command.setpoint.pitch = attitude_setpoint_[1];
      }
      if (!active_axes[2]) {
        command.setpoint.yaw = attitude_setpoint_[2];
      }
      apply_att_setpoint(command);
    }
  }

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      result->message = "setpoint canceled";
      goal_handle->canceled(result);
      return;
    }
    std::array<double, 3> error{};
    bool complete = true;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (killed_) {
        result->message = "kill switch is engaged";
        goal_handle->abort(result);
        return;
      }
      if (goal->command_type == ControlSetpoint::Goal::POSITION) {
        error = {position_setpoint_[0] - position_[0], position_setpoint_[1] - position_[1],
          position_setpoint_[2] - (altitude_mode_ ? altitude_ : position_[2])};
        for (std::size_t axis = 0; axis < 3; ++axis) {
          complete &= !active_axes[axis] || std::abs(error[axis]) <= kPositionTolerance;
        }
      } else {
        error = attitude_error(attitude_setpoint_, attitude_);
        for (std::size_t axis = 0; axis < 3; ++axis) {
          complete &= !active_axes[axis] || std::abs(error[axis]) <= kAttitudeTolerance;
        }
      }
    }
    auto feedback = std::make_shared<ControlSetpoint::Feedback>();
    feedback->error = error;
    goal_handle->publish_feedback(feedback);
    if (complete) {
      result->message = "setpoint reached";
      goal_handle->succeed(result);
      return;
    }
    std::this_thread::sleep_for(50ms);
  }
  result->message = "ROS shutdown";
  goal_handle->abort(result);
}
