#ifndef SIM_DVL_REMAPPER_HPP_
#define SIM_DVL_REMAPPER_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "stonefish_ros2/msg/dvl.hpp"

#include <string>

// Stonefish DVL -> what the Water Linked driver publishes on the vehicle:
// odometry/dvl (velocity only, for the EKF) and altitude (DVL range to the
// bottom, for altitude hold). Readings without bottom lock are dropped, as the
// driver drops them on the vehicle.
class SimDVLRemapper : public rclcpp::Node {
   public:
    explicit SimDVLRemapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg);

   private:
    std::string dvl_link_;
    double velocity_variance_;
    double max_altitude_;
    rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vel_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr altitude_publisher_;
};

#endif  // SIM_DVL_REMAPPER_HPP_
