#include "sub_sim_sensors/sim_pressure_to_depth.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

#include <algorithm>

SimPressureToDepth::SimPressureToDepth(const rclcpp::NodeOptions& options) : Node("sim_pressure_to_depth", options) {
    this->declare_parameter("water_density_kg_m3", 997.0);
    this->declare_parameter("gravity_m_s2", 9.80665);
    this->declare_parameter("frame_id", "odom");
    this->declare_parameter("child_frame_id", "depth_link");
    this->declare_parameter("z_variance", 0.01);

    water_density_kg_m3_ = this->get_parameter("water_density_kg_m3").as_double();
    gravity_m_s2_ = this->get_parameter("gravity_m_s2").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();
    child_frame_id_ = this->get_parameter("child_frame_id").as_string();
    z_variance_ = this->get_parameter("z_variance").as_double();

    subscriber_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
        "sim/pressure", 5, [this](const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
            this->pressure_callback(msg);
        });
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/depth", rclcpp::SensorDataQoS());
}

void SimPressureToDepth::pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
    const double depth = std::max(0.0, msg->fluid_pressure / (water_density_kg_m3_ * gravity_m_s2_));

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    // depth is positive down, output ENU z (positive up)
    odom.pose.pose.position.z = -depth;
    odom.pose.pose.orientation.w = 1.0;
    odom.pose.covariance[14] = z_variance_;

    publisher_->publish(odom);
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimPressureToDepth)
