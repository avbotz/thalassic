#include "sub_sim_sensors/sim_kill_switch.hpp"

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"


SimKillSwitch::SimKillSwitch(const rclcpp::NodeOptions& options) : Node("sim_kill_switch", options) {
    const double off_delay = this->declare_parameter("off_delay", 5.0);

    // Latched (transient_local) so sub_control picks up the current state even if
    // it subscribes after we publish.
    rclcpp::QoS qos(1);
    qos.transient_local();

    pub_ = this->create_publisher<std_msgs::msg::Bool>("kill_switch", qos);
    sim_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "sim/kill_switch", qos, [this](const std_msgs::msg::Bool& msg) { sim_callback(msg); });

    // Start killed, then release after the startup delay.
    publish(true);
    RCLCPP_INFO(this->get_logger(), "kill switch ON; releasing in %.1fs", off_delay);
    startup_timer_ = this->create_timer(std::chrono::duration<double>(off_delay), [this]() { release(); });
}

void SimKillSwitch::release() {
    startup_timer_->cancel();
    startup_done_ = true;
    publish(false);
    RCLCPP_INFO(this->get_logger(), "kill switch OFF");
}

void SimKillSwitch::sim_callback(const std_msgs::msg::Bool& msg) {
    // Ignore external commands during startup window
    // Afterwards pass them through to kill_switch topic
    if (startup_done_) {
        publish(msg.data);
    }
}

void SimKillSwitch::publish(bool killed) {
    std_msgs::msg::Bool m;
    m.data = killed;
    pub_->publish(m);
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimKillSwitch)
