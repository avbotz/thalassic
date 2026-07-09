#include "sub_control/sub_control.hpp"
#include "sub_control/pid_controller.hpp"
#include "sub_control/utils.hpp"
#include "sub_control_interfaces/msg/error.hpp"

#include <angles/angles.h>
#include <rclcpp/rclcpp.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <vector>

using namespace std::chrono_literals;

SubControl::SubControl() : Node("sub_control") {
    this->declare_parameter("control_rate_hz", 30.0);
    this->declare_parameter("power_limit", 0.6);
    this->declare_parameter("robot_name", "");

    this->declare_parameter("pos_pid.x", std::vector<float>{0.8, 0.0, 0.0, 1.0});
    this->declare_parameter("pos_pid.y", std::vector<float>{0.8, 0.0, 0.0, 1.0});
    this->declare_parameter("pos_pid.z", std::vector<float>{0.8, 0.0, 0.0, 1.0});

    this->declare_parameter("vel_pid.x", std::vector<float>{30.0, 3.0, 0.0, 50.0});
    this->declare_parameter("vel_pid.y", std::vector<float>{30.0, 3.0, 0.0, 50.0});
    this->declare_parameter("vel_pid.z", std::vector<float>{30.0, 3.0, 0.0, 50.0});

    this->declare_parameter("att_pid.x", std::vector<float>{0.8, 0.3, 0.0, 5.0});
    this->declare_parameter("att_pid.y", std::vector<float>{0.8, 0.3, 0.0, 5.0});
    this->declare_parameter("att_pid.z", std::vector<float>{0.8, 0.3, 0.0, 5.0});

    this->declare_parameter("ang_pid.x", std::vector<float>{3.5, 0.05, 0.0, 20.0});
    this->declare_parameter("ang_pid.y", std::vector<float>{3.5, 0.05, 0.0, 20.0});
    this->declare_parameter("ang_pid.z", std::vector<float>{3.5, 0.05, 0.0, 20.0});

    this->get_parameter("control_rate_hz", control_rate_hz_);
    this->get_parameter("power_limit", power_limit_);
    this->get_parameter("robot_name", robot_name_);
    power_limit_ = std::clamp(power_limit_, 0.0, 1.0);

    const std::array<std::string, 3> axes = {"x", "y", "z"};

    for (size_t i = 0; i < axes.size(); ++i) {
        position_pid_controllers_[i] = PID_Controller(this->get_parameter("pos_pid." + axes[i]).as_double_array());
        velocity_pid_controllers_[i] = PID_Controller(this->get_parameter("vel_pid." + axes[i]).as_double_array());
        attitude_pid_controllers_[i] = PID_Controller(this->get_parameter("att_pid." + axes[i]).as_double_array());
        angvel_pid_controllers_[i] = PID_Controller(this->get_parameter("ang_pid." + axes[i]).as_double_array());
    }

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });
    altitude_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "altitude", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) { altitude_callback(msg); });
    kill_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "kill_switch", rclcpp::QoS(1).transient_local(), [this](const std_msgs::msg::Bool::SharedPtr msg) { kill_callback(msg); });

    pos_setpoint_sub_ = this->create_subscription<sub_control_interfaces::msg::Setpoint>(
        "pos_setpoint", 10,
        [this](const sub_control_interfaces::msg::Setpoint::SharedPtr msg) { pos_setpoint_callback(msg); });
    att_setpoint_sub_ = this->create_subscription<sub_control_interfaces::msg::Setpoint>(
        "att_setpoint", 10,
        [this](const sub_control_interfaces::msg::Setpoint::SharedPtr msg) { att_setpoint_callback(msg); });
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmd_vel_callback(msg); });

    for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
        thruster_pubs_[i] = this->create_publisher<std_msgs::msg::Float64>(std::format("control/thruster_{}", i), 10);
    }
    error_pub_ = this->create_publisher<sub_control_interfaces::msg::Error>("control/error", 10);

    set_pose_client_ = this->create_client<robot_localization::srv::SetPose>("set_pose");

    control_timer_ =
        this->create_timer(std::chrono::microseconds{static_cast<int>(1e6 / control_rate_hz_)}, [this]() { run(); });

    thruster_allocator_ = ThrusterAllocator();
}

void SubControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    position_[0] = msg->pose.pose.position.x;
    position_[1] = msg->pose.pose.position.y;
    position_[2] = msg->pose.pose.position.z;

    velocity_[0] = msg->twist.twist.linear.x;
    velocity_[1] = msg->twist.twist.linear.y;
    velocity_[2] = msg->twist.twist.linear.z;

    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.pose.orientation, quaternion);
    quaternion.normalize();
    tf2::Matrix3x3(quaternion).getRPY(attitude_[0], attitude_[1], attitude_[2]);

    angvel_[0] = msg->twist.twist.angular.x;
    angvel_[1] = msg->twist.twist.angular.y;
    angvel_[2] = msg->twist.twist.angular.z;
}

void SubControl::altitude_callback(const std_msgs::msg::Float64::SharedPtr msg) { altitude_ = msg->data; }

void SubControl::pos_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg) {
    velocity_control_enabled_ = msg->velocity;
    altitude_control_enabled_ = msg->altitude;
    if (velocity_control_enabled_) {
        if (velocity_setpoint_[0] != msg->setpoint.x || velocity_setpoint_[1] != msg->setpoint.y || velocity_setpoint_[2] != msg->setpoint.z) {
            RCLCPP_INFO(this->get_logger(), "velocity_setpoint_: [%f, %f, %f]", velocity_setpoint_[0], velocity_setpoint_[1], velocity_setpoint_[2]);
        }

        velocity_setpoint_[0] = msg->setpoint.x;
        velocity_setpoint_[1] = msg->setpoint.y;
        velocity_setpoint_[2] = msg->setpoint.z;

        for (auto & pid : velocity_pid_controllers_) {
            pid.reset();
        }
    } else {
        if (position_setpoint_[0] != msg->setpoint.x || position_setpoint_[1] != msg->setpoint.y || position_setpoint_[2] != msg->setpoint.z) {
            RCLCPP_INFO(this->get_logger(), "position_setpoint_: [%f, %f, %f]", position_setpoint_[0], position_setpoint_[1], position_setpoint_[2]);
        }

        position_setpoint_[0] = msg->setpoint.x;
        position_setpoint_[1] = msg->setpoint.y;
        position_setpoint_[2] = msg->setpoint.z;

        for (auto & pid : position_pid_controllers_) {
            pid.reset();
        }
    }
}

void SubControl::att_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg) {
    angvel_control_enabled_ = msg->velocity;
    if (angvel_control_enabled_) {
        if (angvel_setpoint_[0] != msg->setpoint.roll || angvel_setpoint_[1] != msg->setpoint.pitch || angvel_setpoint_[2] != msg->setpoint.yaw) {
            RCLCPP_INFO(this->get_logger(), "angvel_setpoint_: [%f, %f, %f]", angvel_setpoint_[0], angvel_setpoint_[1], angvel_setpoint_[2]);
        }

        angvel_setpoint_[0] = msg->setpoint.roll;
        angvel_setpoint_[1] = msg->setpoint.pitch;
        angvel_setpoint_[2] = msg->setpoint.yaw;

        for (auto & pid : angvel_pid_controllers_) {
            pid.reset();
        }
    } else {
        if (attitude_setpoint_[0] != msg->setpoint.roll || attitude_setpoint_[1] != msg->setpoint.pitch || attitude_setpoint_[2] != msg->setpoint.yaw) {
            RCLCPP_INFO(this->get_logger(), "attitude_setpoint_: [%f, %f, %f]", attitude_setpoint_[0], attitude_setpoint_[1], attitude_setpoint_[2]);
        }

        attitude_setpoint_[0] = angles::normalize_angle(msg->setpoint.roll);
        attitude_setpoint_[1] = angles::normalize_angle(msg->setpoint.pitch);
        attitude_setpoint_[2] = angles::normalize_angle(msg->setpoint.yaw);

        for (auto & pid : attitude_pid_controllers_) {
            pid.reset();
        }
    }
}

void SubControl::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    velocity_control_enabled_ = true;
    angvel_control_enabled_ = true;

    velocity_setpoint_[0] = msg->linear.x;
    velocity_setpoint_[1] = msg->linear.y;
    velocity_setpoint_[2] = msg->linear.z;
    angvel_setpoint_[0] = msg->angular.x;
    angvel_setpoint_[1] = msg->angular.y;
    angvel_setpoint_[2] = msg->angular.z;
}

void SubControl::kill_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (killed_ && !msg->data) {
        auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
        request->pose.header.frame_id = robot_name_.empty() ? "odom" : robot_name_ + "/odom";
        request->pose.header.stamp = this->now();
        request->pose.pose.pose.position.x = 0.0;
        request->pose.pose.pose.position.y = 0.0;
        request->pose.pose.pose.position.z = 0.0;
        request->pose.pose.pose.orientation.x = 0.0;
        request->pose.pose.pose.orientation.y = 0.0;
        request->pose.pose.pose.orientation.z = 0.0;
        request->pose.pose.pose.orientation.w = 1.0;
        set_pose_client_->async_send_request(request);

        for (int i = 0; i < 3; ++i) {
            position_pid_controllers_[i].reset();
            velocity_pid_controllers_[i].reset();
            attitude_pid_controllers_[i].reset();
            angvel_pid_controllers_[i].reset();
        }

        position_setpoint_ = {0.0, 0.0, 0.0};
        velocity_setpoint_ = {0.0, 0.0, 0.0};
        attitude_setpoint_ = {0.0, 0.0, 0.0};
        angvel_setpoint_ = {0.0, 0.0, 0.0};

        position_ = {0.0, 0.0, 0.0};
        velocity_ = {0.0, 0.0, 0.0};
        attitude_ = {0.0, 0.0, 0.0};
        angvel_ = {0.0, 0.0, 0.0};
        altitude_ = 0.0;
    }

    killed_ = msg->data;
}

void SubControl::publish_zero_thrusters() {
    std_msgs::msg::Float64 msg;
    msg.data = 0.0;
    for (auto& pub : thruster_pubs_) {
        pub->publish(msg);
    }
}

void SubControl::run() {
    if (killed_) {
        last_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        publish_zero_thrusters();
        return;
    }

    rclcpp::Time current_time = this->get_clock()->now();

    if (last_update_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
        last_update_time_ = current_time;
        return;
    }

    double dt = std::clamp((current_time - last_update_time_).seconds(), 1e-6, 0.2);
    last_update_time_ = current_time;

    sub_control_interfaces::msg::Error error_msg;
    error_msg.header.stamp = current_time;

    std::array<double, 6> body_force_{};

    tf2::Quaternion orientation;
    orientation.setRPY(attitude_[0], attitude_[1], attitude_[2]);

    if (!velocity_control_enabled_) {
        const double world_ex = position_setpoint_[0] - position_[0];
        const double world_ey = position_setpoint_[1] - position_[1];
        const double vertical = altitude_control_enabled_ ? altitude_ : position_[2];
        const double world_ez = position_setpoint_[2] - vertical;

        tf2::Vector3 world_vel_sp{
            position_pid_controllers_[0].update(position_[0], world_ex, dt),
            position_pid_controllers_[1].update(position_[1], world_ey, dt),
            position_pid_controllers_[2].update(vertical, world_ez, dt)};

        tf2::Vector3 body_vel_sp = tf2::quatRotate(orientation.inverse(), world_vel_sp);
        velocity_setpoint_[0] = body_vel_sp.x();
        velocity_setpoint_[1] = body_vel_sp.y();
        velocity_setpoint_[2] = body_vel_sp.z();

        error_msg.pos_error[0] = world_ex;
        error_msg.pos_error[1] = world_ey;
        error_msg.pos_error[2] = world_ez;
    }

    if (!angvel_control_enabled_) {
        // Geodesic attitude error in the body frame (same frame as the angular
        // rate loop), not per-axis Euler deltas which couple axes and hit a gimbal
        // singularity at pitch = +/-90 deg.
        const std::array<double, 3> att_error = attitude_error(attitude_setpoint_, attitude_);
        for (int i = 0; i < 3; ++i) {
            angvel_setpoint_[i] = attitude_pid_controllers_[i].update(attitude_[i], att_error[i], dt);
            error_msg.att_error[i] = att_error[i];
        }
    }

    for (int i = 0; i < 3; ++i) {
        error_msg.vel_error[i] = velocity_setpoint_[i] - velocity_[i];
        error_msg.angvel_error[i] = angvel_setpoint_[i] - angvel_[i];

        body_force_[i] = velocity_pid_controllers_[i].update(velocity_[i], velocity_setpoint_[i] - velocity_[i], dt);
        body_force_[i + 3] = angvel_pid_controllers_[i].update(angvel_[i], angvel_setpoint_[i] - angvel_[i], dt);
    }

    error_pub_->publish(error_msg);

    const double alloc_max_force = std::min(std::abs(norm_to_force(power_limit_)), std::abs(norm_to_force(-power_limit_)));

    std::array<double, NUM_THRUSTERS> thruster_forces = thruster_allocator_.allocate(body_force_, alloc_max_force);
    for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
        std_msgs::msg::Float64 msg;
        msg.data = std::clamp(force_to_norm(thruster_forces[i]), -power_limit_, power_limit_);
        thruster_pubs_[i]->publish(msg);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubControl>());
    rclcpp::shutdown();
    return 0;
}
