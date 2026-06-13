#ifndef SIM_KILL_SWITCH_HPP_
#define SIM_KILL_SWITCH_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// Sub is kept killed for the first `off_delay` seconds, then released (true -> false on /kill_switch).
// After, what is published on /sim/kill_switch is transparently forwarded to /kill_switch.
class SimKillSwitch : public rclcpp::Node {
   public:
    explicit SimKillSwitch(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void sim_callback(const std_msgs::msg::Bool& msg);
    void release();
    void publish(bool killed);

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sim_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr startup_timer_;

    bool startup_done_{false};
};

#endif  // SIM_KILL_SWITCH_HPP_
