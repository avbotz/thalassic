#ifndef SUB_CONTROL_SUB_CONTROL_HPP_
#define SUB_CONTROL_SUB_CONTROL_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sub_control/controller.hpp"
#include "sub_control/utils.hpp"
#include "sub_control_interfaces/msg/error.hpp"
#include "sub_control_interfaces/msg/setpoint.hpp"

// Command and feedback topics follow REP-103: body frame is FLU (x forward,
// y left, z up), positions are relative to the control origin captured at
// kill-switch release, with x along the initial heading. Commands arrive as
// sub_control_interfaces/Setpoint on pos_setpoint (x/y/z) and att_setpoint
// (yaw/pitch/roll); the velocity flag selects rate control instead of
// absolute targets. All NED/FRD math is internal; conversions happen at the
// subscription/publication boundary.
class ThrusterControl : public rclcpp::Node {
   public:
    ThrusterControl();

   private:
    rclcpp::Subscription<sub_control_interfaces::msg::Setpoint>::SharedPtr pos_setpoint_sub_;
    rclcpp::Subscription<sub_control_interfaces::msg::Setpoint>::SharedPtr att_setpoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr altitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, NUM_THRUSTERS> thruster_pubs_;
    rclcpp::Publisher<sub_control_interfaces::msg::Error>::SharedPtr error_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    double control_rate_hz_{50.0};
    double feedback_timeout_s_{0.5};
    double max_force_{35.0};
    double power_level_{0.6};
    static constexpr double MAX_POWER_LEVEL = 1.0;

    std::mutex state_mutex_;
    std::mutex controller_mutex_;
    bool killed_{true};
    bool state_initialized_{false};
    bool have_odom_{false};
    bool have_altitude_{false};
    std::chrono::steady_clock::time_point last_odom_time_{};
    std::chrono::steady_clock::time_point last_altitude_time_{};
    std::chrono::steady_clock::time_point last_control_time_{};
    bool have_control_time_{false};

    std::array<double, CONTROL_AXES> position_ned_{};
    std::array<double, CONTROL_AXES> velocity_frd_{};
    std::array<double, CONTROL_AXES> attitude_ned_{};
    std::array<double, CONTROL_AXES> angular_rate_frd_{};
    std::array<double, CONTROL_AXES> initial_position_ned_{};
    double initial_yaw_{0.0};
    double altitude_{0.0};

    std::array<double, CONTROL_AXES> position_setpoint_{};
    std::array<double, CONTROL_AXES> velocity_setpoint_{};
    std::array<double, CONTROL_AXES> attitude_setpoint_{};
    std::array<double, CONTROL_AXES> angular_rate_setpoint_{};
    bool position_control_{false};
    bool attitude_control_{true};
    bool use_altitude_{false};

    ControllerConfig controller_config_;
    StateFeedbackController controller_;
    ThrusterAllocator allocator_;
    std::array<double, NUM_DOF> allocation_weights_{1.0, 1.0, 2.0, 2.0, 2.0, 1.5};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

    void pos_setpoint_callback(const sub_control_interfaces::msg::Setpoint& msg);
    void att_setpoint_callback(const sub_control_interfaces::msg::Setpoint& msg);
    void odom_callback(const nav_msgs::msg::Odometry& odom);
    void altitude_callback(const std_msgs::msg::Float64& msg);
    void kill_callback(const std_msgs::msg::Bool& msg);
    void control_loop();
    bool feedback_is_fresh(bool require_altitude) const;
    void publish_thrusters(const std::array<double, NUM_THRUSTERS>& forces, double power);
    void publish_zero_thrusters();
    void publish_errors(const std::array<double, CONTROL_AXES>& position_error,
                        const std::array<double, CONTROL_AXES>& attitude_error, const ControlOutput& output);
    void reset_control_state();
    void load_controller_config();
    rcl_interfaces::msg::SetParametersResult update_parameters(const std::vector<rclcpp::Parameter>& parameters);
};

#endif  // SUB_CONTROL_SUB_CONTROL_HPP_
