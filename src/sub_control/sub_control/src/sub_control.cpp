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
    // control_rate_hz and robot_name are only used to set up the timer and
    // frame ids at startup, so a runtime change would be silently ignored.
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;

    this->declare_parameter("control_rate_hz", 30.0, read_only);
    this->declare_parameter("power_limit", 0.6);
    this->declare_parameter("robot_name", "", read_only);

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

    this->declare_parameter("spin_max_yaw_rate", 2.0);
    this->declare_parameter("spin_done_angle", 0.05);
    this->declare_parameter("spin_done_rate", 0.1);
    this->declare_parameter("spin_decel", 1.0);

    this->get_parameter("control_rate_hz", control_rate_hz_);
    this->get_parameter("power_limit", power_limit_);
    this->get_parameter("robot_name", robot_name_);
    this->get_parameter("spin_max_yaw_rate", spin_max_yaw_rate_);
    this->get_parameter("spin_done_angle", spin_done_angle_);
    this->get_parameter("spin_done_rate", spin_done_rate_);
    this->get_parameter("spin_decel", spin_decel_);
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
    spin_setpoint_sub_ = this->create_subscription<sub_control_interfaces::msg::Spin>(
        "spin_setpoint", 10,
        [this](const sub_control_interfaces::msg::Spin::SharedPtr msg) { spin_setpoint_callback(msg); });
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

    param_cb_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) { return on_parameters_set(params); });
}

PID_Controller* SubControl::pid_for_parameter(const std::string& name) {
    std::array<PID_Controller, 3>* bank = nullptr;
    if (name.starts_with("pos_pid.")) {
        bank = &position_pid_controllers_;
    } else if (name.starts_with("vel_pid.")) {
        bank = &velocity_pid_controllers_;
    } else if (name.starts_with("att_pid.")) {
        bank = &attitude_pid_controllers_;
    } else if (name.starts_with("ang_pid.")) {
        bank = &angvel_pid_controllers_;
    }
    if (bank == nullptr) {
        return nullptr;
    }

    const std::string axis = name.substr(name.find('.') + 1);
    if (axis == "x") {
        return &(*bank)[0];
    }
    if (axis == "y") {
        return &(*bank)[1];
    }
    if (axis == "z") {
        return &(*bank)[2];
    }
    return nullptr;
}

rcl_interfaces::msg::SetParametersResult SubControl::on_parameters_set(const std::vector<rclcpp::Parameter>& params) {
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
        } else if (param.get_name() == "spin_max_yaw_rate" || param.get_name() == "spin_done_angle" ||
                   param.get_name() == "spin_done_rate" || param.get_name() == "spin_decel") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                result.successful = false;
                result.reason = param.get_name() + " must be a double";
                return result;
            }
            if (param.as_double() <= 0.0) {
                result.successful = false;
                result.reason = param.get_name() + " must be > 0";
                return result;
            }
            if (param.get_name() == "spin_max_yaw_rate") {
                spin_max_yaw_rate_ = param.as_double();
            } else if (param.get_name() == "spin_done_angle") {
                spin_done_angle_ = param.as_double();
            } else if (param.get_name() == "spin_done_rate") {
                spin_done_rate_ = param.as_double();
            } else {
                spin_decel_ = param.as_double();
            }
            RCLCPP_INFO(this->get_logger(), "%s set to %f", param.get_name().c_str(), param.as_double());
        } else if (PID_Controller* pid = pid_for_parameter(param.get_name())) {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                result.successful = false;
                result.reason = param.get_name() + " must be a double array";
                return result;
            }
            const std::vector<double> gains = param.as_double_array();
            if (gains.size() != 3 && gains.size() != 4) {
                result.successful = false;
                result.reason = param.get_name() + " must be [kp, ki, kd] or [kp, ki, kd, output_limit]";
                return result;
            }
            pid->configure(gains);
            RCLCPP_INFO(this->get_logger(), "%s set to [%f, %f, %f, %f]", param.get_name().c_str(), gains[0],
                        gains[1], gains[2], gains.size() == 4 ? gains[3] : 0.0);
        }
    }

    return result;
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

    // Unwrapped yaw: consecutive samples differ by well under pi at EKF rates,
    // so accumulating shortest-path deltas keeps a continuous multi-turn angle.
    if (!have_yaw_unwrapped_) {
        yaw_unwrapped_ = attitude_[2];
        have_yaw_unwrapped_ = true;
    } else {
        yaw_unwrapped_ += angles::shortest_angular_distance(prev_wrapped_yaw_, attitude_[2]);
    }
    prev_wrapped_yaw_ = attitude_[2];
    last_odom_time_ = this->get_clock()->now();

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
            RCLCPP_INFO(this->get_logger(), "velocity_setpoint_: [%f, %f, %f]", msg->setpoint.x, msg->setpoint.y, msg->setpoint.z);
        }

        velocity_setpoint_[0] = msg->setpoint.x;
        velocity_setpoint_[1] = msg->setpoint.y;
        velocity_setpoint_[2] = msg->setpoint.z;

        reset_pid();
    } else {
        if (position_setpoint_[0] != msg->setpoint.x || position_setpoint_[1] != msg->setpoint.y || position_setpoint_[2] != msg->setpoint.z) {
            RCLCPP_INFO(this->get_logger(), "position_setpoint_: [%f, %f, %f]", msg->setpoint.x, msg->setpoint.y, msg->setpoint.z);
        }

        position_setpoint_[0] = msg->setpoint.x;
        position_setpoint_[1] = msg->setpoint.y;
        position_setpoint_[2] = msg->setpoint.z;

        reset_pid();
    }
}

void SubControl::att_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg) {
    spin_active_ = false;
    angvel_control_enabled_ = msg->velocity;
    if (angvel_control_enabled_) {
        if (angvel_setpoint_[0] != msg->setpoint.roll || angvel_setpoint_[1] != msg->setpoint.pitch || angvel_setpoint_[2] != msg->setpoint.yaw) {
            RCLCPP_INFO(this->get_logger(), "angvel_setpoint_: [%f, %f, %f]", msg->setpoint.roll, msg->setpoint.pitch, msg->setpoint.yaw);
        }

        angvel_setpoint_[0] = msg->setpoint.roll;
        angvel_setpoint_[1] = msg->setpoint.pitch;
        angvel_setpoint_[2] = msg->setpoint.yaw;

        reset_pid();
    } else {
        if (attitude_setpoint_[0] != msg->setpoint.roll || attitude_setpoint_[1] != msg->setpoint.pitch || attitude_setpoint_[2] != msg->setpoint.yaw) {
            RCLCPP_INFO(this->get_logger(), "attitude_setpoint_: [%f, %f, %f]", msg->setpoint.roll, msg->setpoint.pitch, msg->setpoint.yaw);
        }

        attitude_setpoint_[0] = angles::normalize_angle(msg->setpoint.roll);
        attitude_setpoint_[1] = angles::normalize_angle(msg->setpoint.pitch);
        attitude_setpoint_[2] = angles::normalize_angle(msg->setpoint.yaw);

        reset_pid();
    }
}

void SubControl::spin_setpoint_callback(const sub_control_interfaces::msg::Spin::SharedPtr msg) {
    if (killed_ || !have_yaw_unwrapped_) {
        RCLCPP_WARN(this->get_logger(), "Ignoring spin command: %s", killed_ ? "killed" : "no odometry yet");
        return;
    }

    // Base the target on the commanded heading projected into unwrapped space
    // so spins compose exactly with prior attitude setpoints; fall back to the
    // measured yaw when coming out of angular-velocity mode. A spin already in
    // flight composes with its own unwrapped target instead: the wrapped
    // attitude setpoint would collapse the remaining multi-turn distance.
    double base = yaw_unwrapped_;
    if (spin_active_) {
        base = yaw_target_unwrapped_;
    } else if (!angvel_control_enabled_) {
        base += angles::shortest_angular_distance(attitude_[2], attitude_setpoint_[2]);
    }
    yaw_target_unwrapped_ = base + msg->yaw;
    spin_rate_ = std::min(msg->max_rate > 0.0 ? msg->max_rate : spin_max_yaw_rate_, spin_max_yaw_rate_);

    angvel_control_enabled_ = false;
    // Commit the final heading up front so every exit path (completion, abort,
    // preemption) collapses into a normal attitude hold.
    attitude_setpoint_[2] = angles::normalize_angle(yaw_target_unwrapped_);
    spin_active_ = true;
    spin_ack_pending_ = true;
    reset_pid();

    RCLCPP_INFO(this->get_logger(), "spin: yaw=%f rate=%f final heading=%f", msg->yaw, spin_rate_,
                attitude_setpoint_[2]);
}

void SubControl::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    spin_active_ = false;
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
    if (msg->data) {
        // Never resume a spin across a kill.
        spin_active_ = false;
    }
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

        // The EKF pose was just re-zeroed; restart the unwrapped-yaw
        // bookkeeping from the next odometry sample.
        spin_active_ = false;
        have_yaw_unwrapped_ = false;
        yaw_unwrapped_ = 0.0;
        prev_wrapped_yaw_ = 0.0;
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
        // spin_ack_pending_ defers the done-check so every accepted spin gets at
        // least one error publish with spin_active = true, even one that starts
        // inside the done window (the mission's acknowledgment depends on it).
        if (spin_active_ && !spin_ack_pending_) {
            const double remaining = yaw_target_unwrapped_ - yaw_unwrapped_;
            if ((current_time - last_odom_time_).seconds() > SPIN_ODOM_TIMEOUT) {
                RCLCPP_ERROR(this->get_logger(), "Odometry stale during spin; aborting and holding final heading");
                spin_active_ = false;
            } else if (std::fabs(remaining) <= spin_done_angle_ && std::fabs(angvel_[2]) <= spin_done_rate_) {
                // Requiring low rate as well means spin_active never drops while
                // still carrying momentum through the target.
                RCLCPP_INFO(this->get_logger(), "Spin complete at yaw=%f", attitude_[2]);
                spin_active_ = false;
            }
        }

        if (spin_active_) {
            const double remaining = yaw_target_unwrapped_ - yaw_unwrapped_;
            // Roll/pitch stay geodesic: with the target yaw pinned to the current
            // yaw the error rotation is pure roll/pitch. The yaw component is the
            // unwrapped remaining angle (which may exceed pi), so the attitude PID
            // saturates at the rate cap, cruises, and decelerates onto the
            // measured heading.
            std::array<double, 3> att_error =
                attitude_error({attitude_setpoint_[0], attitude_setpoint_[1], attitude_[2]}, attitude_);
            att_error[2] = remaining;
            // Brake-limited cap (sqrt profile): never command a yaw rate that a
            // spin_decel_ deceleration cannot shed within the remaining angle.
            // Below linear_region the plain P term alone demands <= spin_decel_
            // of braking, so the cap hands off to it continuously; the result is
            // the same terminal approach no matter how many turns were commanded.
            double rate_cap = spin_rate_;
            const double kp = attitude_pid_controllers_[2].kp();
            if (kp > 0.0) {
                const double linear_region = spin_decel_ / (kp * kp);
                if (std::fabs(remaining) > linear_region) {
                    rate_cap = std::min(
                        rate_cap, std::sqrt(2.0 * spin_decel_ * (std::fabs(remaining) - 0.5 * linear_region)));
                }
            }
            angvel_setpoint_[0] = attitude_pid_controllers_[0].update(attitude_[0], att_error[0], dt);
            angvel_setpoint_[1] = attitude_pid_controllers_[1].update(attitude_[1], att_error[1], dt);
            angvel_setpoint_[2] = attitude_pid_controllers_[2].update(yaw_unwrapped_, remaining, dt, rate_cap);
            for (int i = 0; i < 3; ++i) {
                error_msg.att_error[i] = att_error[i];
            }
        } else {
            // Geodesic attitude error in the body frame (same frame as the angular
            // rate loop), not per-axis Euler deltas which couple axes and hit a gimbal
            // singularity at pitch = +/-90 deg.
            const std::array<double, 3> att_error = attitude_error(attitude_setpoint_, attitude_);
            // Unwrapped yaw as the measurement keeps derivative-on-measurement
            // continuous across the +/-pi crossing and the spin -> hold handoff.
            const std::array<double, 3> att_meas = {attitude_[0], attitude_[1], yaw_unwrapped_};
            for (int i = 0; i < 3; ++i) {
                angvel_setpoint_[i] = attitude_pid_controllers_[i].update(att_meas[i], att_error[i], dt);
                error_msg.att_error[i] = att_error[i];
            }
        }
    }

    const double alloc_max_force = std::min(std::abs(norm_to_force(power_limit_)), std::abs(norm_to_force(-power_limit_)));

    // Cap the force/torque loops at the wrench the thrusters can actually
    // produce, so anti-windup engages at real saturation. A configured limit
    // above physical authority lets the integral wind into force that never
    // materializes and unwind slowly afterwards (e.g. spins overshooting worse
    // the longer they cruise).
    const std::array<double, NUM_DOF> achievable_wrench = thruster_allocator_.max_wrench(alloc_max_force);
    const auto capped_limit = [](const PID_Controller& pid, double achievable) {
        if (achievable <= 0.0) {
            return pid.output_limit();
        }
        return pid.output_limit() > 0.0 ? std::min(pid.output_limit(), achievable) : achievable;
    };

    for (int i = 0; i < 3; ++i) {
        error_msg.vel_error[i] = velocity_setpoint_[i] - velocity_[i];
        error_msg.angvel_error[i] = angvel_setpoint_[i] - angvel_[i];

        body_force_[i] = velocity_pid_controllers_[i].update(velocity_[i], velocity_setpoint_[i] - velocity_[i], dt,
                                                             capped_limit(velocity_pid_controllers_[i], achievable_wrench[i]));
        body_force_[i + 3] = angvel_pid_controllers_[i].update(angvel_[i], angvel_setpoint_[i] - angvel_[i], dt,
                                                               capped_limit(angvel_pid_controllers_[i], achievable_wrench[i + 3]));
    }

    error_msg.spin_active = spin_active_;
    error_pub_->publish(error_msg);
    spin_ack_pending_ = false;

    std::array<double, NUM_THRUSTERS> thruster_forces = thruster_allocator_.allocate(body_force_, alloc_max_force);
    for (size_t i = 0; i < NUM_THRUSTERS; ++i) {
        std_msgs::msg::Float64 msg;
        msg.data = std::clamp(force_to_norm(thruster_forces[i]), -power_limit_, power_limit_);
        thruster_pubs_[i]->publish(msg);
    }
}

void SubControl::reset_pid() {
    for (auto& pid : position_pid_controllers_) {
        pid.reset();
    }
    for (auto& pid : velocity_pid_controllers_) {
        pid.reset();
    }
    for (auto& pid : attitude_pid_controllers_) {
        pid.reset();
    }
    for (auto& pid : angvel_pid_controllers_) {
        pid.reset();
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubControl>());
    rclcpp::shutdown();
    return 0;
}
