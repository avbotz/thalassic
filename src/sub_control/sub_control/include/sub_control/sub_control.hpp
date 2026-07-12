#include "sub_control/pid_controller.hpp"
#include "sub_control_interfaces/msg/setpoint.hpp"
#include "sub_control_interfaces/msg/spin.hpp"
#include "sub_control/utils.hpp"
#include "sub_control_interfaces/msg/error.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <robot_localization/srv/set_pose.hpp>

#include <array>
#include <string>
#include <vector>

class SubControl : public rclcpp::Node {
   public:
    SubControl();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void altitude_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void kill_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void pos_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg);
    void att_setpoint_callback(const sub_control_interfaces::msg::Setpoint::SharedPtr msg);
    void spin_setpoint_callback(const sub_control_interfaces::msg::Spin::SharedPtr msg);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void run();

   private:
    static constexpr size_t NUM_THRUSTERS = 8;

    void publish_zero_thrusters();

    void reset_pid();

    rcl_interfaces::msg::SetParametersResult on_parameters_set(const std::vector<rclcpp::Parameter>& params);
    PID_Controller* pid_for_parameter(const std::string& name);

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

    // Spin (relative multi-turn yaw) mode. Yaw is tracked unwrapped (continuous
    // across +/-pi) so a spin target can be several full turns away.
    bool spin_active_{false};
    // Guarantees at least one control/error publish with spin_active = true per
    // accepted spin, so the mission's acknowledgment never misses a spin that
    // completes on its first control tick.
    bool spin_ack_pending_{false};
    double yaw_unwrapped_{0.0};
    double prev_wrapped_yaw_{0.0};
    bool have_yaw_unwrapped_{false};
    double yaw_target_unwrapped_{0.0};
    double spin_rate_{0.0};
    double spin_max_yaw_rate_{2.0};
    double spin_done_angle_{0.05};
    double spin_done_rate_{0.10};
    double spin_decel_{1.0};
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
    static constexpr double SPIN_ODOM_TIMEOUT = 0.5;

    bool killed_{true};

    // State subscribers
    rclcpp::Subscription<sub_control_interfaces::msg::Setpoint>::SharedPtr pos_setpoint_sub_;
    rclcpp::Subscription<sub_control_interfaces::msg::Setpoint>::SharedPtr att_setpoint_sub_;
    rclcpp::Subscription<sub_control_interfaces::msg::Spin>::SharedPtr spin_setpoint_sub_;
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

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    ThrusterAllocator thruster_allocator_;
};
