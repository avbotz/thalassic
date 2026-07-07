#ifndef SIM_IMU_REMAPPER_HPP_
#define SIM_IMU_REMAPPER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class SimIMURemapper : public rclcpp::Node {
   public:
    explicit SimIMURemapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

   private:
    std::string imu_link_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

#endif  // SIM_IMU_REMAPPER_HPP_
