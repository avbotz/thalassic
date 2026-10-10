#include "sub_sim_sensors/sim_kill_switch.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"

SimKillSwitch::SimKillSwitch(const rclcpp::NodeOptions& options) : Node("sim_kill_switch", options) {
    const double off_delay = this->declare_parameter("off_delay", 5.0);
    esc_startup_ = this->declare_parameter("esc_startup", 3.0);
    // Timer periods: a NaN one is undefined, and a negative esc_startup would throw from
    // sim_callback and take the whole container down.
    if (!std::isfinite(off_delay) || !std::isfinite(esc_startup_) || esc_startup_ < 0.0) {
        throw std::invalid_argument("off_delay must be finite and esc_startup finite and non-negative");
    }

    // Latched (transient_local) so sub_control picks up the current state even if
    // it subscribes after we publish.
    rclcpp::QoS qos(1);
    qos.transient_local();

    pub_ = this->create_publisher<std_msgs::msg::Bool>("kill_switch", qos);
    // Volatile: a transient_local subscription never hears volatile publishers, such as
    // `ros2 topic pub`'s and Foxglove's.
    sim_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "sim/kill_switch", 10, [this](const std_msgs::msg::Bool& msg) { sim_callback(msg); });

    // Start killed, then release after the startup delay; without one, start released.
    if (off_delay <= 0.0) {
        release();
        return;
    }
    publish(true);
    RCLCPP_INFO(this->get_logger(), "kill switch ON; releasing in %.1fs", off_delay);
    startup_timer_ = this->create_timer(std::chrono::duration<double>(off_delay), [this]() { release(); });
}

void SimKillSwitch::release() {
    if (startup_timer_) {
        startup_timer_->cancel();
    }
    startup_done_ = true;
    publish(false);
    RCLCPP_INFO(this->get_logger(), "kill switch OFF");
}

void SimKillSwitch::sim_callback(const std_msgs::msg::Bool& msg) {
    // Ignore external commands during startup window
    // Afterwards pass them through to kill_switch topic
    if (!startup_done_) {
        return;
    }
    if (release_timer_) {
        // A kill (or another release) supersedes a release still waiting.
        release_timer_->cancel();
        release_timer_.reset();
    }
    if (msg.data) {
        publish(true);
        return;
    }
    // On the simulation clock, without holding up the executor.
    release_timer_ = this->create_timer(std::chrono::duration<double>(esc_startup_), [this]() {
        release_timer_->cancel();  // once
        publish(false);
    });
}

void SimKillSwitch::publish(bool killed) {
    std_msgs::msg::Bool m;
    m.data = killed;
    pub_->publish(m);
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimKillSwitch)
