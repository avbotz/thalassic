#include "sub_control/pid_controller.hpp"
#include "sub_control_interfaces/msg/setpoint.hpp"
#include "sub_control/utils.hpp"
#include "sub_control_interfaces/msg/error.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <robot_localization/srv/set_pose.hpp>

#include <array>
#include <string>

class SubControl : public rclcpp::Node {
   public:
    SubControl();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void altitude_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void kill_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void pos_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg);
    void att_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void run();

   private:
    static constexpr size_t NUM_THRUSTERS = 8;

    void publish_zero_thrusters();

    double control_rate_hz_{100.0};
    double power_limit_{0.6};
    std::string robot_name_{""};

    std::array<PID_Controller, 3> position_pid_controllers_;
    std::array<PID_Controller, 3> velocity_pid_controllers_;

    std::array<PID_Controller, 3> attitude_pid_controllers_;
    std::array<PID_Controller, 3> angvel_pid_controllers_;

    std::array<double, 3> position_{0.0, 0.0, 0.0};
    std::array<double, 3> velocity_{0.0, 0.0, 0.0};
    std::array<double, 3> attitude_{0.0, 0.0, 0.0};
    std::array<double, 3> angvel_{0.0, 0.0, 0.0};
    double altitude_{0.0};

    std::array<double, 3> position_setpoint_{0.0, 0.0, 0.0};
    std::array<double, 3> velocity_setpoint_{0.0, 0.0, 0.0};
    bool velocity_control_enabled_{false};
    bool altitude_control_enabled_{false};

    std::array<double, 3> attitude_setpoint_{0.0, 0.0, 0.0};
    std::array<double, 3> angvel_setpoint_{0.0, 0.0, 0.0};
    bool angvel_control_enabled_{false};

    bool killed_{true};

    // State subscribers
    rclcpp::Subscription<sub_control_interfaces::msg::Setpoint>::SharedPtr pos_setpoint_sub_;
    rclcpp::Subscription<sub_control_interfaces::msg::Setpoint>::SharedPtr att_setpoint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Setpoint subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr altitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub_;

    // Publishers
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, NUM_THRUSTERS> thruster_pubs_;
    rclcpp::Publisher<sub_control_interfaces::msg::Error>::SharedPtr error_pub_;

    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr set_pose_client_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};

    ThrusterAllocator thruster_allocator_;
};
