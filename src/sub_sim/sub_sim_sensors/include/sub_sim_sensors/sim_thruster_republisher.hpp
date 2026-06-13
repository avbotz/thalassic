#ifndef SIM_THRUSTER_REPUBLISHER_HPP_
#define SIM_THRUSTER_REPUBLISHER_HPP_

#include <array>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class SimThrusterRepublisher : public rclcpp::Node {
   public:
    explicit SimThrusterRepublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    std::array<double, 8> thruster_values_;
    std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, 8> subscribers_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    double thrust_to_pwm(double thrust);
};

#endif  // SIM_THRUSTER_REPUBLISHER_HPP_
