#ifndef SIM_THRUSTERS_HPP_
#define SIM_THRUSTERS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "stonefish_ros2/msg/thruster_state.hpp"

#include <array>
#include <string>

// Stands in for the ESCs between sub_control and the simulated thrusters:
// control/thruster_<i> -> sim/thruster_setpoints, through a first-order lag
// (the BlueRobotics Basic ESC takes ~0.1 s to respond) and a per-thruster
// `effectiveness` that injects faults (0 dead, 0.5 half thrust; settable at
// runtime). Also reports what each simulated thruster really pushes, on
// thrusters/measured_thrust [N along each thruster's +X], the signal current
// sensing or ESC telemetry would give on the vehicle.
class SimThrusters : public rclcpp::Node {
   public:
    static constexpr int NUM_THRUSTERS = 8;
    // The cap sub_low puts on |normalized command| on the vehicle.
    static constexpr double MAX_COMMAND = 0.6;

    explicit SimThrusters(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    // Why `param` cannot take its value; empty if it can.
    static std::string invalid(const rclcpp::Parameter& param);
    // Takes up a parameter's value, which must be valid.
    void apply(const rclcpp::Parameter& param);
    void publish_setpoints();
    void state_callback(const stonefish_ros2::msg::ThrusterState::SharedPtr msg);

    std::array<double, NUM_THRUSTERS> command_{};
    std::array<double, NUM_THRUSTERS> output_{};
    std::array<double, NUM_THRUSTERS> effectiveness_{};
    double time_constant_{0.1};
    rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};

    std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, NUM_THRUSTERS> command_subs_;
    rclcpp::Subscription<stonefish_ros2::msg::ThrusterState>::SharedPtr state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

#endif  // SIM_THRUSTERS_HPP_
