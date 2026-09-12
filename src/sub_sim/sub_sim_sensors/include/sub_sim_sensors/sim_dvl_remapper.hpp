#ifndef SIM_DVL_REMAPPER_HPP_
#define SIM_DVL_REMAPPER_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "stonefish_ros2/msg/dvl.hpp"

class SimDVLRemapper : public rclcpp::Node {
   public:
    explicit SimDVLRemapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg);

   private:
    std::string dvl_link_;
    rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vel_publisher_;
};

#endif  // SIM_DVL_REMAPPER_HPP_
