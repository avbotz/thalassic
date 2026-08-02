#include "sub_sim_sensors/sim_thruster_republisher.hpp"

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

SimThrusterRepublisher::SimThrusterRepublisher(const rclcpp::NodeOptions & options)
: Node("sim_thruster_republisher", options)
{
  publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("sim/thruster_setpoints",
    10);

  for (int i = 0; i < 8; i++) {
    std::string topic_name = std::format("control/thruster_{}", i);
    subscribers_[i] = this->create_subscription<std_msgs::msg::Float64>(
            topic_name, 10, [this, i](const std_msgs::msg::Float64::SharedPtr msg) {
                // Both high-level controllers publish normalized ESC commands.
                // Stonefish's thruster actuator also accepts [-1, 1] directly;
                // multiplying by the real ESC's 400 us half-range saturated
                // every nontrivial simulated command at full thrust.
        thruster_values_[i] = std::clamp(msg->data, -1.0, 1.0);
        std_msgs::msg::Float64MultiArray multi_array_msg;
        multi_array_msg.data = std::vector<double>(thruster_values_.begin(),
        thruster_values_.end());
        publisher_->publish(multi_array_msg);
            });
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimThrusterRepublisher)
