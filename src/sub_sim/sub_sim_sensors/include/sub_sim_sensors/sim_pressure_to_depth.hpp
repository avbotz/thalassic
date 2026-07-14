#ifndef SIM_PRESSURE_TO_DEPTH_HPP_
#define SIM_PRESSURE_TO_DEPTH_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

#include <string>

class SimPressureToDepth : public rclcpp::Node {
   public:
    explicit SimPressureToDepth(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);

   private:
    double water_density_kg_m3_;
    double gravity_m_s2_;
    double surface_pressure_pa_;
    std::string frame_id_;
    std::string child_frame_id_;
    double z_variance_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

#endif  // SIM_PRESSURE_TO_DEPTH_HPP_
