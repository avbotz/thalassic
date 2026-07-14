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

    // Start killed, then release after the startup delay.  A zero delay is
    // useful in simulation: composable nodes are loaded after the launch
    // graph is already running, so relying on a timer to make the first
    // release can leave a mission permanently killed if that executor is
    // still busy finishing startup.
    if (off_delay <= 0.0) {
        // Sim launch requests immediate operation. Avoid publishing an
        // initial latched true that can arrive after the controller's
        // simulation-only startup override and re-kill it.
        release();
    } else {
        publish(true);
        RCLCPP_INFO(this->get_logger(), "kill switch ON; releasing in %.1fs", off_delay);
        startup_timer_ = this->create_timer(std::chrono::duration<double>(off_delay), [this]() { release(); });
    }
}

void SimKillSwitch::release() {
    if (startup_timer_) {
        startup_timer_->cancel();
    }
    startup_done_ = true;
    publish(false);
    // Re-publish the released state for late-starting controller nodes. The
    // transient sample should cover this, but the composable sim startup can
    // race DDS endpoint discovery.
    released_heartbeat_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() { publish(false); });
    RCLCPP_INFO(this->get_logger(), "kill switch OFF");
}

void SimKillSwitch::sim_callback(const std_msgs::msg::Bool& msg) {
    // Ignore external commands during startup window
    // Afterwards pass them through to kill_switch topic
    if (startup_done_) {
        if (!msg.data) {
            // model time waiting for ESCs to initialize before sending unkill command
            rclcpp::sleep_for(std::chrono::seconds(3));
        }
        publish(msg.data);
    }
}

void SimKillSwitch::publish(bool killed) {
    std_msgs::msg::Bool m;
    m.data = killed;
    pub_->publish(m);
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimKillSwitch)
