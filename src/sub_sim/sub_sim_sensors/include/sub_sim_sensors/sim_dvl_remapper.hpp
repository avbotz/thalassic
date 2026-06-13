#ifndef SIM_DVL_REMAPPER_HPP_
#define SIM_DVL_REMAPPER_HPP_

#include "marine_acoustic_msgs/msg/dvl.hpp"
#include "rclcpp/rclcpp.hpp"
#include "stonefish_ros2/msg/dvl.hpp"

using namespace std::chrono_literals;

class SimDVLRemapper : public rclcpp::Node {
   public:
    explicit SimDVLRemapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg);

   private:
    std::string robot_name_;
    rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr subscriber_;
    rclcpp::Publisher<marine_acoustic_msgs::msg::Dvl>::SharedPtr vel_publisher_;
};

#endif  // SIM_DVL_REMAPPER_HPP_
