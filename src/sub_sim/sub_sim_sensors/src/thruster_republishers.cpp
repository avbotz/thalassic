#include "sub_sim_sensors/thruster_republishers.hpp"

#include <array>
#include <memory>
#include "rclcpp/rclcpp.hpp"

ThrusterRepublisher::ThrusterRepublisher() : Node("thruster_republisher") {
    this->declare_parameter("robot_name", "");
    robot_name_ = this->get_parameter("robot_name").as_string();

    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(std::format("/{}/sim/thruster_setpoints", robot_name_), 10);

    for (int i=0; i<8; i++) {
        std::string topic_name = std::format("/{}/control/thruster_{}", robot_name_, i);
        subscribers_[i] = this->create_subscription<std_msgs::msg::Float64>(
            topic_name, 10, [this, i](const std_msgs::msg::Float64::SharedPtr msg) {
                thruster_values_[i] = msg->data * 400;
                std_msgs::msg::Float64MultiArray multi_array_msg;
                multi_array_msg.data = std::vector<double>(thruster_values_.begin(), thruster_values_.end());
                publisher_->publish(multi_array_msg);
            }
        );
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterRepublisher>());
    rclcpp::shutdown();
    return 0;
}