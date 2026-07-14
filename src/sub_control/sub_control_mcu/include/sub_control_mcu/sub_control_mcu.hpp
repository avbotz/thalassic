#ifndef SUB_CONTROL_MCU_SUB_CONTROL_MCU_HPP_
#define SUB_CONTROL_MCU_SUB_CONTROL_MCU_HPP_

#include "sub_control_interfaces/msg/error.hpp"
#include "sub_control_interfaces/msg/setpoint.hpp"
#include "sub_control_interfaces/action/control_setpoint.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

extern "C" {
#include <mec/control.h>
#include <mec/estimation.h>
#include <mec/pid_controller.h>
#include <mec/util.h>
}

#include <array>
#include <mutex>
#include <string>
#include <vector>

// Reimplementation of the Nautical-Private MCU control loop (main.cpp run())
// as a ROS 2 node. The mec_* math runs in the MCU's native NED/FRD frames;
// every topic keeps the workspace's REP-103 ENU/FLU convention and is
// converted at the callback/publish boundary. Drop-in alternative to
// sub_control: same topics in and out.
class SubControlMcu : public rclcpp::Node {
   public:
    SubControlMcu();

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void altitude_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void kill_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void pos_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg);
    void att_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void run();

   private:
    using ControlSetpoint = sub_control_interfaces::action::ControlSetpoint;
    using GoalHandleControlSetpoint = rclcpp_action::ServerGoalHandle<ControlSetpoint>;

    void publish_zero_thrusters();
    void reset_state_on_revive();
    void apply_pos_setpoint(const sub_control_interfaces::msg::Setpoint& msg);
    void apply_att_setpoint(const sub_control_interfaces::msg::Setpoint& msg);
    rclcpp_action::GoalResponse handle_setpoint_goal(const rclcpp_action::GoalUUID& uuid,
                                                     std::shared_ptr<const ControlSetpoint::Goal> goal);
    rclcpp_action::CancelResponse handle_setpoint_cancel(const std::shared_ptr<GoalHandleControlSetpoint> goal_handle);
    void execute_setpoint_goal(const std::shared_ptr<GoalHandleControlSetpoint> goal_handle);

    rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter>& params);
    struct pid_controller* pid_for_parameter(const std::string& name);

    double control_rate_hz_{10.0};
    double power_limit_{0.6};
    double startup_pause_s_{0.0};
    std::string robot_name_{""};

    // MCU controller state (all NED/FRD, see mec headers)
    struct att_controller attitude_controller_ {};
    struct angvel_controller angular_velocity_controller_ {};
    struct position_controller pos_controller_ {};
    struct velocity_controller vel_controller_ {};

    struct mec_vehicle_position position_ {};
    struct mec_vehicle_velocity_body velocity_body_ {};
    struct mec_vehicle_attitude attitude_ {};
    struct mec_vehicle_angvel angvel_ {};

    struct mec_vehicle_position position_sp_ {};
    struct mec_vehicle_velocity_body velocity_body_sp_ {};
    struct mec_vehicle_attitude att_sp_ {};
    struct mec_vehicle_angvel angvel_sp_ {};

    // MCU 'v' / 't' command equivalents: skip the outer position/attitude loop
    bool velocity_override_{false};
    bool angvel_override_{false};

    float mix_[8][6];

    bool killed_{true};
    std::mutex state_mutex_;
    bool pause_{false};
    rclcpp::Time pause_end_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<sub_control_interfaces::msg::Setpoint>::SharedPtr pos_setpoint_sub_;
    rclcpp::Subscription<sub_control_interfaces::msg::Setpoint>::SharedPtr att_setpoint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr altitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub_;

    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 8> thruster_pubs_;
    rclcpp::Publisher<sub_control_interfaces::msg::Error>::SharedPtr error_pub_;

    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr set_pose_client_;
    rclcpp_action::Server<ControlSetpoint>::SharedPtr setpoint_action_server_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

#endif  // SUB_CONTROL_MCU_SUB_CONTROL_MCU_HPP_
