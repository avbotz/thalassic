#include "sub_sim_sensors/sim_pressure_to_depth.hpp"

#include <algorithm>
#include <cctype>

#include "rclcpp_components/register_node_macro.hpp"

using std::placeholders::_1;

SimPressureToDepth::SimPressureToDepth(const rclcpp::NodeOptions& options) : Node("sim_pressure_to_depth", options) {
    this->declare_parameter("pressure_unit", "pa");
    this->declare_parameter("pressure_reference", "gauge");
    this->declare_parameter("surface_pressure_pa", 101325.0);
    this->declare_parameter("water_density_kg_m3", 997.0);
    this->declare_parameter("gravity_m_s2", 9.80665);

    pressure_unit_ = this->get_parameter("pressure_unit").as_string();
    std::transform(pressure_unit_.begin(), pressure_unit_.end(), pressure_unit_.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    pressure_reference_ = this->get_parameter("pressure_reference").as_string();
    std::transform(pressure_reference_.begin(), pressure_reference_.end(), pressure_reference_.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    surface_pressure_pa_ = this->get_parameter("surface_pressure_pa").as_double();
    water_density_kg_m3_ = this->get_parameter("water_density_kg_m3").as_double();
    gravity_m_s2_ = this->get_parameter("gravity_m_s2").as_double();

    subscriber_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
        "sim/pressure", 10, std::bind(&SimPressureToDepth::pressure_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("depth", rclcpp::SystemDefaultsQoS());
}

double SimPressureToDepth::pressure_to_pascal(double pressure) const {
    if (pressure_unit_ == "bar") {
        return pressure * 100000.0;
    }
    return pressure;
}

void SimPressureToDepth::pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
    std_msgs::msg::Float64 depth_msg;
    const double pressure_pa = pressure_to_pascal(msg->fluid_pressure);
    double gauge_pressure_pa = pressure_pa;
    if (pressure_reference_ == "absolute") {
        gauge_pressure_pa -= surface_pressure_pa_;
    }
    depth_msg.data = std::max(0.0, gauge_pressure_pa / (water_density_kg_m3_ * gravity_m_s2_));
    publisher_->publish(depth_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimPressureToDepth)
