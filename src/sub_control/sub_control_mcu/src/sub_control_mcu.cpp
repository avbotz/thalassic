#include "sub_control_mcu/sub_control_mcu.hpp"

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <format>
#include <memory>
#include <thread>

// The MCU firmware works in NED/FRD; the workspace speaks REP-103 ENU/FLU.
// The two are related by a pi rotation about x (identifying ENU x with
// "north"), so every vector and Euler triple converts as below. These helpers
// are the only place frames change; everything between them is the MCU
// algorithm untouched.
namespace {
constexpr double POSITION_TOLERANCE = 0.25;
constexpr double ATTITUDE_TOLERANCE = 0.0872665;

// ENU world vector <-> NED world vector, and FLU body vector <-> FRD body
// vector: keep x, negate y and z.
constexpr float flip_y(double v) { return static_cast<float>(-v); }
constexpr float flip_z(double v) { return static_cast<float>(-v); }

// Euler angles and body rates: keep roll, negate pitch and yaw.
constexpr float flip_pitch(double v) { return static_cast<float>(-v); }
constexpr float flip_yaw(double v) { return static_cast<float>(-v); }

}  // namespace

SubControlMcu::SubControlMcu() : Node("sub_control_mcu") {
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;

    // The MCU ran its control block every 100 ms.
    this->declare_parameter("control_rate_hz", 10.0, read_only);
    // MCU 'p' command; the sub was normally run at 0.6.
    this->declare_parameter("power_limit", 0.6);
    // PAUSE_TIME after the kill switch flips to alive. The firmware intended
    // 7 s but compared micros() against a millis() stamp, so the pause it
    // actually ran was ~0 -- hence the 0.0 default.
    this->declare_parameter("startup_pause_s", 0.0, read_only);
    this->declare_parameter("robot_name", "", read_only);

    // Standard-form gains [kp, ti, td] (see mec/pid_controller.h), defaults
    // from the *_controller_init() functions. att/ang axes: x=roll, y=pitch,
    // z=yaw.
    this->declare_parameter("pos_pid.x", std::vector<double>{0.6, 0.0, 0.0});
    this->declare_parameter("pos_pid.y", std::vector<double>{0.6, 0.0, 0.0});
    this->declare_parameter("pos_pid.z", std::vector<double>{0.55, 0.0, 0.0});

    this->declare_parameter("vel_pid.x", std::vector<double>{4.0, 10.0, 0.0});
    this->declare_parameter("vel_pid.y", std::vector<double>{4.0, 10.0, 0.0});
    this->declare_parameter("vel_pid.z", std::vector<double>{4.0, 3.5, 0.03});

    this->declare_parameter("att_pid.x", std::vector<double>{1.2, 0.0, 0.0});
    this->declare_parameter("att_pid.y", std::vector<double>{1.2, 0.0, 0.0});
    this->declare_parameter("att_pid.z", std::vector<double>{1.2, 0.0, 0.05});

    this->declare_parameter("ang_pid.x", std::vector<double>{0.3, 0.0, 0.0});
    this->declare_parameter("ang_pid.y", std::vector<double>{0.3, 0.0, 0.0});
    this->declare_parameter("ang_pid.z", std::vector<double>{0.6, 30.0, 0.0});

    this->get_parameter("control_rate_hz", control_rate_hz_);
    this->get_parameter("power_limit", power_limit_);
    this->get_parameter("startup_pause_s", startup_pause_s_);
    this->get_parameter("robot_name", robot_name_);
    power_limit_ = std::clamp(power_limit_, 0.0, 1.0);

    att_controller_init(&attitude_controller_);
    angvel_controller_init(&angular_velocity_controller_);
    position_controller_init(&pos_controller_);
    velocity_controller_init(&vel_controller_);
    mec_vehicle_position_init(&position_);

    const std::array<std::string, 3> axes = {"x", "y", "z"};
    const std::array<std::string, 4> banks = {"pos_pid", "vel_pid", "att_pid", "ang_pid"};
    for (const auto& bank : banks) {
        for (const auto& axis : axes) {
            const std::string name = bank + "." + axis;
            const std::vector<double> g = this->get_parameter(name).as_double_array();
            if (g.size() == 3) {
                pid_set_gains(pid_for_parameter(name), static_cast<float>(g[0]), static_cast<float>(g[1]),
                              static_cast<float>(g[2]));
            }
        }
    }

    std::memcpy(mix_, sub_mix_data, sizeof(float) * NUM_THRUSTERS * NUM_DOF);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });
    altitude_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "altitude", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { altitude_callback(msg); });
    kill_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "kill_switch", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) { kill_callback(msg); });

    pos_setpoint_sub_ = this->create_subscription<sub_control_interfaces::msg::Setpoint>(
        "pos_setpoint", 10,
        [this](const sub_control_interfaces::msg::Setpoint::SharedPtr msg) { pos_setpoint_callback(msg); });
    att_setpoint_sub_ = this->create_subscription<sub_control_interfaces::msg::Setpoint>(
        "att_setpoint", 10,
        [this](const sub_control_interfaces::msg::Setpoint::SharedPtr msg) { att_setpoint_callback(msg); });
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmd_vel_callback(msg); });

    for (size_t i = 0; i < thruster_pubs_.size(); ++i) {
        thruster_pubs_[i] = this->create_publisher<std_msgs::msg::Float64>(std::format("control/thruster_{}", i), 10);
    }
    error_pub_ = this->create_publisher<sub_control_interfaces::msg::Error>("control/error", 10);

    set_pose_client_ = this->create_client<robot_localization::srv::SetPose>("set_pose");
    reset_state_service_ = this->create_service<std_srvs::srv::Trigger>(
        "reset_state_on_revive",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            reset_state_service_callback(request, response);
        });
    setpoint_action_server_ = rclcpp_action::create_server<ControlSetpoint>(
        this, "control_setpoint",
        [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ControlSetpoint::Goal> goal) {
            return handle_setpoint_goal(uuid, std::move(goal));
        },
        [this](const std::shared_ptr<GoalHandleControlSetpoint> goal_handle) {
            return handle_setpoint_cancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleControlSetpoint> goal_handle) {
            std::thread{[this, goal_handle] { execute_setpoint_goal(goal_handle); }}.detach();
        });

    control_timer_ =
        this->create_timer(std::chrono::microseconds{static_cast<int>(1e6 / control_rate_hz_)}, [this]() { run(); });

    param_cb_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) { return on_parameters_set(params); });
}

struct pid_controller* SubControlMcu::pid_for_parameter(const std::string& name) {
    struct pid_controller* bank = nullptr;
    if (name.starts_with("pos_pid.")) {
        bank = pos_controller_.pid;
    } else if (name.starts_with("vel_pid.")) {
        bank = vel_controller_.pid;
    } else if (name.starts_with("att_pid.")) {
        bank = attitude_controller_.pid;
    } else if (name.starts_with("ang_pid.")) {
        bank = angular_velocity_controller_.pid;
    }
    if (bank == nullptr) {
        return nullptr;
    }

    const std::string axis = name.substr(name.find('.') + 1);
    if (axis == "x") {
        return &bank[0];
    }
    if (axis == "y") {
        return &bank[1];
    }
    if (axis == "z") {
        return &bank[2];
    }
    return nullptr;
}

rcl_interfaces::msg::SetParametersResult SubControlMcu::on_parameters_set(
    const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params) {
        if (param.get_name() == "power_limit") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                result.successful = false;
                result.reason = "power_limit must be a double";
                return result;
            }
            if (param.as_double() < 0.0 || param.as_double() > 1.0) {
                result.successful = false;
                result.reason = "power_limit must be in [0, 1]";
                return result;
            }
            power_limit_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "power_limit set to %f", power_limit_);
        } else if (struct pid_controller* pid = pid_for_parameter(param.get_name())) {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                result.successful = false;
                result.reason = param.get_name() + " must be a double array";
                return result;
            }
            const std::vector<double> gains = param.as_double_array();
            if (gains.size() != 3) {
                result.successful = false;
                result.reason = param.get_name() + " must be [kp, ti, td] (standard form, 3 elements)";
                return result;
            }
            // Like the MCU 'u' command: swap gains without resetting the
            // integral or previous error.
            pid_set_gains(pid, static_cast<float>(gains[0]), static_cast<float>(gains[1]),
                          static_cast<float>(gains[2]));
            RCLCPP_INFO(this->get_logger(), "%s set to [%f, %f, %f]", param.get_name().c_str(), gains[0], gains[1],
                        gains[2]);
        }
    }

    return result;
}

void SubControlMcu::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    position_.north = static_cast<float>(msg->pose.pose.position.x);
    position_.east = flip_y(msg->pose.pose.position.y);
    position_.down = flip_z(msg->pose.pose.position.z);

    // Odometry twist is in the body (child) frame, like the DVL on the MCU.
    velocity_body_.forward_m_s = static_cast<float>(msg->twist.twist.linear.x);
    velocity_body_.right_m_s = flip_y(msg->twist.twist.linear.y);
    velocity_body_.down_m_s = flip_z(msg->twist.twist.linear.z);

    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.pose.orientation, quaternion);
    quaternion.normalize();
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    attitude_.roll = static_cast<float>(roll);
    attitude_.pitch = flip_pitch(pitch);
    attitude_.yaw = flip_yaw(yaw);

    // The MCU differentiated consecutive AHRS readings because it had no rate
    // output; odometry supplies the same signal directly.
    angvel_.roll_rad_s = static_cast<float>(msg->twist.twist.angular.x);
    angvel_.pitch_rad_s = flip_pitch(msg->twist.twist.angular.y);
    angvel_.yaw_rad_s = flip_yaw(msg->twist.twist.angular.z);
}

void SubControlMcu::altitude_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    position_.altitude = static_cast<float>(msg->data);
}

void SubControlMcu::apply_pos_setpoint(const sub_control_interfaces::msg::Setpoint& msg) {
    if (msg.velocity) {
        // MCU 'v' command: body velocity setpoint, bypass the position loop.
        velocity_override_ = true;

        if (velocity_body_sp_.forward_m_s != msg.setpoint.x || velocity_body_sp_.right_m_s != flip_y(msg.setpoint.y) ||
            velocity_body_sp_.down_m_s != flip_z(msg.setpoint.z)) {
            RCLCPP_DEBUG(this->get_logger(), "velocity_setpoint_: [%f, %f, %f]", msg.setpoint.x, msg.setpoint.y,
                         msg.setpoint.z);
        }

        velocity_body_sp_.forward_m_s = static_cast<float>(msg.setpoint.x);
        velocity_body_sp_.right_m_s = flip_y(msg.setpoint.y);
        velocity_body_sp_.down_m_s = flip_z(msg.setpoint.z);

        velocity_controller_update_sp(&vel_controller_, &velocity_body_sp_);
        return;
    }

    // MCU 's' / 'z' / 'b' commands: position setpoint, optionally holding
    // altitude off the floor instead of depth.
    velocity_override_ = false;

    if (position_sp_.north != msg.setpoint.x || position_sp_.east != flip_y(msg.setpoint.y) ||
        (msg.altitude ? position_sp_.altitude : position_sp_.down) != (msg.altitude ? msg.setpoint.z : flip_z(msg.setpoint.z)) ||
        pos_controller_.use_floor_altitude != msg.altitude) {
        RCLCPP_DEBUG(this->get_logger(), "%s_setpoint_: [%f, %f, %f]", msg.altitude ? "altitude" : "position",
                     msg.setpoint.x, msg.setpoint.y, msg.setpoint.z);
    }

    position_sp_.north = static_cast<float>(msg.setpoint.x);
    position_sp_.east = flip_y(msg.setpoint.y);

    pos_controller_.use_floor_altitude = msg.altitude;
    if (msg.altitude) {
        // Altitude is a positive distance to the floor in both conventions.
        position_sp_.altitude = static_cast<float>(msg.setpoint.z);
    } else {
        position_sp_.down = flip_z(msg.setpoint.z);
    }

    position_controller_update_sp(&pos_controller_, &position_sp_);
}

void SubControlMcu::att_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    apply_att_setpoint(*msg);
}

void SubControlMcu::pos_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    apply_pos_setpoint(*msg);
}

void SubControlMcu::apply_att_setpoint(const sub_control_interfaces::msg::Setpoint& msg) {
    if (msg.velocity) {
        // MCU 't' command: angular rate setpoint, bypass the attitude loop.
        angvel_override_ = true;

        if (angvel_sp_.roll_rad_s != msg.setpoint.roll || angvel_sp_.pitch_rad_s != flip_pitch(msg.setpoint.pitch) ||
            angvel_sp_.yaw_rad_s != flip_yaw(msg.setpoint.yaw)) {
            RCLCPP_DEBUG(this->get_logger(), "angvel_setpoint_: [%f, %f, %f]", msg.setpoint.roll,
                         msg.setpoint.pitch, msg.setpoint.yaw);
        }

        angvel_sp_.roll_rad_s = static_cast<float>(msg.setpoint.roll);
        angvel_sp_.pitch_rad_s = flip_pitch(msg.setpoint.pitch);
        angvel_sp_.yaw_rad_s = flip_yaw(msg.setpoint.yaw);

        angvel_controller_update_sp(&angular_velocity_controller_, &angvel_sp_);
        return;
    }

    // MCU 'n' command.
    angvel_override_ = false;

    if (att_sp_.roll != msg.setpoint.roll || att_sp_.pitch != flip_pitch(msg.setpoint.pitch) ||
        att_sp_.yaw != flip_yaw(msg.setpoint.yaw)) {
        RCLCPP_DEBUG(this->get_logger(), "attitude_setpoint_: [%f, %f, %f]", msg.setpoint.roll,
                     msg.setpoint.pitch, msg.setpoint.yaw);
    }

    att_sp_.roll = static_cast<float>(msg.setpoint.roll);
    att_sp_.pitch = flip_pitch(msg.setpoint.pitch);
    att_sp_.yaw = flip_yaw(msg.setpoint.yaw);

    att_controller_update_sp(&attitude_controller_, &att_sp_);
}

rclcpp_action::GoalResponse SubControlMcu::handle_setpoint_goal(
    const rclcpp_action::GoalUUID&, const std::shared_ptr<const ControlSetpoint::Goal> goal) {
    if (goal->command_type > ControlSetpoint::Goal::ATTITUDE) {
        RCLCPP_WARN(this->get_logger(), "Rejecting ControlSetpoint goal with invalid command type %u", goal->command_type);
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SubControlMcu::handle_setpoint_cancel(
    const std::shared_ptr<GoalHandleControlSetpoint>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SubControlMcu::execute_setpoint_goal(const std::shared_ptr<GoalHandleControlSetpoint> goal_handle) {
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
            active_axes = {std::isfinite(goal->setpoint.setpoint.x), std::isfinite(goal->setpoint.setpoint.y),
                           std::isfinite(goal->setpoint.setpoint.z)};
            auto command = goal->setpoint;
            command.velocity = false;
            if (!active_axes[0]) command.setpoint.x = position_sp_.north;
            if (!active_axes[1]) command.setpoint.y = -position_sp_.east;
            if (!active_axes[2]) command.setpoint.z = pos_controller_.use_floor_altitude ? position_sp_.altitude : -position_sp_.down;
            apply_pos_setpoint(command);
        } else if (goal->command_type == ControlSetpoint::Goal::VELOCITY) {
            auto command = goal->setpoint;
            command.velocity = true;
            apply_pos_setpoint(command);
            result->message = "velocity setpoint accepted";
            goal_handle->succeed(result);
            return;
        } else {
            active_axes = {std::isfinite(goal->setpoint.setpoint.roll), std::isfinite(goal->setpoint.setpoint.pitch),
                           std::isfinite(goal->setpoint.setpoint.yaw)};
            auto command = goal->setpoint;
            command.velocity = false;
            if (!active_axes[0]) command.setpoint.roll = att_sp_.roll;
            if (!active_axes[1]) command.setpoint.pitch = -att_sp_.pitch;
            if (!active_axes[2]) command.setpoint.yaw = -att_sp_.yaw;
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
                error = {position_sp_.north - position_.north, -(position_sp_.east - position_.east),
                         pos_controller_.use_floor_altitude ? position_.altitude - position_sp_.altitude
                                                            : -(position_sp_.down - position_.down)};
                for (size_t i = 0; i < error.size(); ++i) complete &= !active_axes[i] || std::fabs(error[i]) <= POSITION_TOLERANCE;
            } else {
                error = {angle_difference(att_sp_.roll, attitude_.roll),
                         -angle_difference(att_sp_.pitch, attitude_.pitch),
                         -angle_difference(att_sp_.yaw, attitude_.yaw)};
                for (size_t i = 0; i < error.size(); ++i) complete &= !active_axes[i] || std::fabs(error[i]) <= ATTITUDE_TOLERANCE;
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
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    result->message = "ROS shutdown";
    goal_handle->abort(result);
}

void SubControlMcu::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    velocity_override_ = true;
    angvel_override_ = true;

    RCLCPP_DEBUG(this->get_logger(), "cmd_vel: linear=[%f, %f, %f] angular=[%f, %f, %f]", msg->linear.x,
                 msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);

    velocity_body_sp_.forward_m_s = static_cast<float>(msg->linear.x);
    velocity_body_sp_.right_m_s = flip_y(msg->linear.y);
    velocity_body_sp_.down_m_s = flip_z(msg->linear.z);
    angvel_sp_.roll_rad_s = static_cast<float>(msg->angular.x);
    angvel_sp_.pitch_rad_s = flip_pitch(msg->angular.y);
    angvel_sp_.yaw_rad_s = flip_yaw(msg->angular.z);

    velocity_controller_update_sp(&vel_controller_, &velocity_body_sp_);
    angvel_controller_update_sp(&angular_velocity_controller_, &angvel_sp_);
}

void SubControlMcu::reset_state_on_revive() {
    // MCU dead->alive transition: zero the local frame, the setpoints, and the
    // overrides, then pause so the ESCs can start up. The EKF is re-zeroed
    // through set_pose, mirroring the MCU resetting its dead-reckoned state.
    auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
    request->pose.header.frame_id = robot_name_.empty() ? "odom" : robot_name_ + "/odom";
    request->pose.header.stamp = this->now();
    request->pose.pose.pose.orientation.w = 1.0;
    set_pose_client_->async_send_request(request);

    position_ = {};
    velocity_body_ = {};
    attitude_ = {};
    angvel_ = {};

    position_sp_ = {};
    velocity_body_sp_ = {};
    att_sp_ = {};
    angvel_sp_ = {};

    velocity_override_ = false;
    angvel_override_ = false;
    pos_controller_.use_floor_altitude = false;

    // The MCU carried PID state across kills; with the EKF teleporting back to
    // the origin a stale integral would be wrong, so reset it here.
    for (int i = 0; i < 3; ++i) {
        pid_reset(&pos_controller_.pid[i]);
        pid_reset(&vel_controller_.pid[i]);
        pid_reset(&attitude_controller_.pid[i]);
        pid_reset(&angular_velocity_controller_.pid[i]);
    }

    position_controller_update_sp(&pos_controller_, &position_sp_);
    velocity_controller_update_sp(&vel_controller_, &velocity_body_sp_);
    att_controller_update_sp(&attitude_controller_, &att_sp_);
    angvel_controller_update_sp(&angular_velocity_controller_, &angvel_sp_);

    if (startup_pause_s_ > 0.0) {
        pause_ = true;
        pause_end_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(startup_pause_s_);
    }
}

void SubControlMcu::reset_state_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    reset_state_on_revive();
    response->success = true;
    response->message = "Control state reset";
    RCLCPP_INFO(this->get_logger(), "Control state reset by service request");
}

void SubControlMcu::kill_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (killed_ != msg->data) {
        RCLCPP_INFO(this->get_logger(), "Kill switch %s", msg->data ? "engaged; zeroing thrusters" : "released; resetting state");
    }
    if (killed_ && !msg->data) {
        reset_state_on_revive();
    }
    killed_ = msg->data;
}

void SubControlMcu::publish_zero_thrusters() {
    std_msgs::msg::Float64 msg;
    msg.data = 0.0;
    for (auto& pub : thruster_pubs_) {
        pub->publish(msg);
    }
}

void SubControlMcu::run() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (killed_) {
        last_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        publish_zero_thrusters();
        return;
    }

    const rclcpp::Time current_time = this->get_clock()->now();

    if (pause_) {
        if (current_time < pause_end_) {
            last_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            publish_zero_thrusters();
            return;
        }
        // Like the MCU resetting motor_time when the pause ends, so the first
        // dt after the pause is one tick long.
        pause_ = false;
    }

    if (last_update_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
        last_update_time_ = current_time;
        return;
    }

    double dt = (current_time - last_update_time_).seconds();
    if (dt <= 0.0) {
        return;
    }
    // The MCU's dt was bounded by its tick; cap ours so a stalled sim clock
    // does not dump one giant step into the integrals.
    dt = std::min(dt, 0.2);
    last_update_time_ = current_time;

    const float dt_f = static_cast<float>(dt);

    sub_control_interfaces::msg::Error error_msg;
    error_msg.header.stamp = current_time;

    // Attitude P loop -> angular velocity setpoint (MCU main.cpp order:
    // rotation cascade first, then translation).
    if (!angvel_override_) {
        att_controller_update(&attitude_controller_, &attitude_, &angvel_sp_, dt_f);
        angvel_controller_update_sp(&angular_velocity_controller_, &angvel_sp_);

        error_msg.att_error[0] = angle_difference(attitude_controller_.att_sp.roll, attitude_.roll);
        error_msg.att_error[1] = -angle_difference(attitude_controller_.att_sp.pitch, attitude_.pitch);
        error_msg.att_error[2] = -angle_difference(attitude_controller_.att_sp.yaw, attitude_.yaw);
    }

    // Angular rate PID -> torque, with anti-reset windup.
    struct mec_torque_setpoint torque_out;
    angvel_controller_update(&angular_velocity_controller_, &angvel_, &torque_out, dt_f);

    error_msg.angvel_error[0] = angular_velocity_controller_.angvel_sp.roll_rad_s - angvel_.roll_rad_s;
    error_msg.angvel_error[1] = -(angular_velocity_controller_.angvel_sp.pitch_rad_s - angvel_.pitch_rad_s);
    error_msg.angvel_error[2] = -(angular_velocity_controller_.angvel_sp.yaw_rad_s - angvel_.yaw_rad_s);

    // Position P loop -> body velocity setpoint.
    if (!velocity_override_) {
        position_controller_update(&pos_controller_, &position_, &attitude_, &velocity_body_sp_, dt_f);
        velocity_controller_update_sp(&vel_controller_, &velocity_body_sp_);

        // Recompute the position error for telemetry. Publish it in the same
        // world ENU frame as odometry and the position setpoint so dashboard
        // target/current/error traces are directly comparable. The controller
        // still performs its own body-frame conversion internally.
        struct mec_vehicle_position ned_error;
        ned_error.north = pos_controller_.position_sp.north - position_.north;
        ned_error.east = pos_controller_.position_sp.east - position_.east;
        if (pos_controller_.use_floor_altitude) {
            ned_error.down = position_.altitude - pos_controller_.position_sp.altitude;
        } else {
            ned_error.down = pos_controller_.position_sp.down - position_.down;
        }
        error_msg.pos_error[0] = ned_error.north;
        error_msg.pos_error[1] = -ned_error.east;
        error_msg.pos_error[2] = -ned_error.down;
    }

    // Body velocity PID -> force, with anti-reset windup.
    struct mec_force_setpoint force_out;
    velocity_controller_update(&vel_controller_, &velocity_body_, &force_out, dt_f);

    error_msg.vel_error[0] = vel_controller_.velocity_sp.forward_m_s - velocity_body_.forward_m_s;
    error_msg.vel_error[1] = -(vel_controller_.velocity_sp.right_m_s - velocity_body_.right_m_s);
    error_msg.vel_error[2] = -(vel_controller_.velocity_sp.down_m_s - velocity_body_.down_m_s);

    error_pub_->publish(error_msg);

    // Map forces and torques to thruster outputs. The outputs are already in
    // thalassic's normalized convention (+/-1 == +/-400 counts), and the mix
    // rows are in thalassic channel order -- see mec/control.h.
    float thruster_outputs[8];
    mec_mix(&force_out, &torque_out, mix_, static_cast<float>(power_limit_), thruster_outputs);

    for (size_t i = 0; i < thruster_pubs_.size(); ++i) {
        std_msgs::msg::Float64 msg;
        msg.data = thruster_outputs[i];
        thruster_pubs_[i]->publish(msg);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubControlMcu>());
    rclcpp::shutdown();
    return 0;
}
