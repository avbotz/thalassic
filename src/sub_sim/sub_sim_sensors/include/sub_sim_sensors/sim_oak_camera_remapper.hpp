#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sub_sim_sensors/depth_conversion.hpp"

// Bridges Stonefish's color + depth camera output into the topic layout and data
// formats produced by the real OAK-D Pro (depthai_ros_driver), so the sub_vision
// pipeline runs unchanged in sim and on hardware.
//   * color : Stonefish rgb8 -> oak/rgb/image_raw (bgr8)
//   * depth : Stonefish 32FC1 meters -> oak/stereo/image_raw (16UC1 millimeters)
class SimOakCameraRemapper : public rclcpp::Node {
   public:
    SimOakCameraRemapper();

   private:
    void color_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void color_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
};

int main(int argc, char* argv[]);
