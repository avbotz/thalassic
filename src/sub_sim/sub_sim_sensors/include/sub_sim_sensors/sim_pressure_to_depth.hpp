#ifndef SIM_PRESSURE_TO_DEPTH_HPP_
#define SIM_PRESSURE_TO_DEPTH_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/float64.hpp"

class SimPressureToDepth : public rclcpp::Node {
   public:
    explicit SimPressureToDepth(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);

   private:
    double pressure_to_pascal(double pressure) const;

    std::string pressure_unit_;
    double surface_pressure_pa_;
    double water_density_kg_m3_;
    double gravity_m_s2_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

#endif  // SIM_PRESSURE_TO_DEPTH_HPP_
