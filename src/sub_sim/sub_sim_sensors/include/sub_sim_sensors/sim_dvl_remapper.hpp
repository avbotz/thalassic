#include <array>
#include <chrono>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "marine_acoustic_msgs/msg/dvl.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "stonefish_ros2/msg/dvl.hpp"

using namespace std::chrono_literals;

std::array<double, 3> offsets_to_frame(std::array<double, 3> xyz, std::array<double, 3> angles, std::array<double, 3>& output);

class SimDVLRemapper : public rclcpp::Node {
   public:
    SimDVLRemapper();
    void dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

   private:
    std::string robot_name_;
    std::array<double, 3> rpy_{0.0f, 0.0f, 0.0f};
    rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr dvl_imu_subscriber_;
    rclcpp::Publisher<marine_acoustic_msgs::msg::Dvl>::SharedPtr vel_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
};

int main(int argc, char* argv[]);
