#ifndef SIM_DVL_REMAPPER_HPP_
#define SIM_DVL_REMAPPER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "stonefish_ros2/msg/dvl.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SimDVLRemapper : public rclcpp::Node {
   public:
    explicit SimDVLRemapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg);

   private:
    std::string dvl_link_;
    std::string odom_frame_id_;
    double pool_height_;
    double z_variance_;
    rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr altitude_publisher_;
};

#endif  // SIM_DVL_REMAPPER_HPP_
