#include "sub_sim_sensors/sim_dvl_remapper.hpp"
#include <rclcpp/qos.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "stonefish_ros2/msg/dvl.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

SimDVLRemapper::SimDVLRemapper(const rclcpp::NodeOptions& options) : Node("sim_dvl_remapper", options) {
    this->declare_parameter("robot_name", "");
    robot_name_ = this->get_parameter("robot_name").as_string();

    subscriber_ = this->create_subscription<stonefish_ros2::msg::DVL>(
        "sim/dvl", 10, std::bind(&SimDVLRemapper::dvl_callback, this, _1));

    vel_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/dvl", rclcpp::SystemDefaultsQoS());
}

void SimDVLRemapper::dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg_stonefish) {
    nav_msgs::msg::Odometry odom_msg{};

    odom_msg.header = msg_stonefish->header;
    odom_msg.header.frame_id = robot_name_.empty() ? "dvl_link" : robot_name_ + "/dvl_link";
    odom_msg.child_frame_id = robot_name_.empty() ? "dvl_link" : robot_name_ + "/dvl_link";

    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    odom_msg.twist.twist.linear.x = msg_stonefish->velocity.x;
    odom_msg.twist.twist.linear.y = msg_stonefish->velocity.y;
    odom_msg.twist.twist.linear.z = msg_stonefish->velocity.z;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    vel_publisher_->publish(odom_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimDVLRemapper)
