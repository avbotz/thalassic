#include "sub_sim_sensors/sim_dvl_remapper.hpp"
#include <rclcpp/qos.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/float64.hpp"
#include "stonefish_ros2/msg/dvl.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

SimDVLRemapper::SimDVLRemapper(const rclcpp::NodeOptions& options) : Node("sim_dvl_remapper", options) {
    this->declare_parameter("dvl_link", "dvl_link");
    this->declare_parameter("odom_frame_id", "odom");
    // Depth of the pool floor below the surface (m); mirrors the real
    // waterlinked_dvl_driver, which reports z = altitude - pool_height.
    this->declare_parameter("pool_height", 0.0);
    this->declare_parameter("z_variance", 0.01);
    dvl_link_ = this->get_parameter("dvl_link").as_string();
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    pool_height_ = this->get_parameter("pool_height").as_double();
    z_variance_ = this->get_parameter("z_variance").as_double();

    subscriber_ = this->create_subscription<stonefish_ros2::msg::DVL>(
        "sim/dvl", 10, std::bind(&SimDVLRemapper::dvl_callback, this, _1));

    vel_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/dvl", rclcpp::SystemDefaultsQoS());
    altitude_publisher_ = this->create_publisher<std_msgs::msg::Float64>("altitude", rclcpp::SystemDefaultsQoS());
}

void SimDVLRemapper::dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg_stonefish) {
    nav_msgs::msg::Odometry odom_msg{};

    odom_msg.header = msg_stonefish->header;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = dvl_link_;

    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    // x/y position and orientation are never measured (no dead reckoning);
    // -1 marks them invalid for consumers.
    odom_msg.pose.covariance[0] = -1.0;
    odom_msg.pose.covariance[7] = -1.0;
    odom_msg.pose.covariance[21] = -1.0;
    odom_msg.pose.covariance[28] = -1.0;
    odom_msg.pose.covariance[35] = -1.0;

    // The altitude is the height above the pool floor, which sits pool_height
    // below the surface (world z = 0), so a valid altitude is an absolute
    // measurement of z (ENU, negative underwater). Stonefish reports -1 when
    // there is no bottom lock.
    if (msg_stonefish->altitude >= 0.0) {
        odom_msg.pose.pose.position.z = msg_stonefish->altitude - pool_height_;
        odom_msg.pose.covariance[14] = z_variance_;

        std_msgs::msg::Float64 altitude_msg;
        altitude_msg.data = msg_stonefish->altitude;
        altitude_publisher_->publish(altitude_msg);
    } else {
        odom_msg.pose.covariance[14] = 1e6;
    }

    odom_msg.twist.twist.linear.x = msg_stonefish->velocity.x;
    odom_msg.twist.twist.linear.y = msg_stonefish->velocity.y;
    odom_msg.twist.twist.linear.z = msg_stonefish->velocity.z;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    vel_publisher_->publish(odom_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimDVLRemapper)
