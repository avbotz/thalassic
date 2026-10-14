#ifndef SIM_THRUSTERS_HPP_
#define SIM_THRUSTERS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sub_control_interfaces/msg/thruster_command.hpp"

#include <array>
#include <optional>
#include <string>

// Stands in for sub_low, the firmware and the ESCs between sub_control and the
// simulated thrusters: control/thrusters -> sim/thruster_setpoints, through a
// per-thruster `effectiveness` that injects faults (0 dead, 0.5 half thrust;
// settable at runtime). Like sub_low on the vehicle, it stops every thruster
// when the commands stop or go stale for `command_timeout`. The ESCs' response
// lag is each Stonefish thruster's rotor dynamics (thruster.j2).
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
    void command_callback(const sub_control_interfaces::msg::ThrusterCommand& msg);
    void publish_setpoints();

    std::array<double, NUM_THRUSTERS> command_{};
    std::array<double, NUM_THRUSTERS> effectiveness_{};
    double command_timeout_{0.5};
    std::optional<rclcpp::Time> command_stamp_;  // of the command in command_; none once stopped

    rclcpp::Subscription<sub_control_interfaces::msg::ThrusterCommand>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr setpoint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

#endif  // SIM_THRUSTERS_HPP_
