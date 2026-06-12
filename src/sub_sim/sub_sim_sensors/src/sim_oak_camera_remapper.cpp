#include "sub_sim_sensors/sim_oak_camera_remapper.hpp"

#include <cmath>
#include <cstdint>

#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/image_encodings.hpp"

using std::placeholders::_1;

SimOakCameraRemapper::SimOakCameraRemapper() : Node("sim_oak_camera_remapper") {
    // Stonefish-side inputs (relative to this node's namespace).
    const std::string color_in = this->declare_parameter("color_in_topic", "sim/front_camera/image_color");
    const std::string color_info_in = this->declare_parameter("color_info_in_topic", "sim/front_camera/camera_info");
    const std::string depth_in = this->declare_parameter("depth_in_topic", "sim/depth_camera/image_depth");
    const std::string depth_info_in = this->declare_parameter("depth_info_in_topic", "sim/depth_camera/camera_info");

    // OAK-D-style outputs.
    const std::string rgb_out = this->declare_parameter("rgb_out_topic", "oak/rgb/image_raw");
    const std::string rgb_info_out = this->declare_parameter("rgb_info_out_topic", "oak/rgb/camera_info");
    const std::string depth_out = this->declare_parameter("depth_out_topic", "oak/stereo/image_raw");
    const std::string depth_info_out = this->declare_parameter("depth_info_out_topic", "oak/stereo/camera_info");

    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(rgb_out, 10);
    rgb_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(rgb_info_out, 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_out, 10);
    depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(depth_info_out, 10);

    color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        color_in, 10, std::bind(&SimOakCameraRemapper::color_callback, this, _1));
    color_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        color_info_in, 10, std::bind(&SimOakCameraRemapper::color_info_callback, this, _1));
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_in, 10, std::bind(&SimOakCameraRemapper::depth_callback, this, _1));
    depth_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_info_in, 10, std::bind(&SimOakCameraRemapper::depth_info_callback, this, _1));
}

void SimOakCameraRemapper::color_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Stonefish publishes rgb8; the real OAK-D rgb stream is bgr8.
    cv_bridge::CvImageConstPtr cv = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    sensor_msgs::msg::Image::SharedPtr out =
        cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, cv->image).toImageMsg();
    rgb_pub_->publish(*out);
}

void SimOakCameraRemapper::color_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    rgb_info_pub_->publish(*msg);
}

void SimOakCameraRemapper::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Stonefish depth is 32FC1 in meters; OAK-D stereo depth is 16UC1 in millimeters.
    cv_bridge::CvImageConstPtr cv = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);

    cv::Mat depth_mm(cv->image.rows, cv->image.cols, CV_16UC1);
    for (int r = 0; r < cv->image.rows; ++r) {
        const float* src = cv->image.ptr<float>(r);
        uint16_t* dst = depth_mm.ptr<uint16_t>(r);
        for (int c = 0; c < cv->image.cols; ++c) {
            dst[c] = depth_meters_to_mm(src[c]);
        }
    }

    sensor_msgs::msg::Image::SharedPtr out =
        cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_16UC1, depth_mm).toImageMsg();
    depth_pub_->publish(*out);
}

void SimOakCameraRemapper::depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    depth_info_pub_->publish(*msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimOakCameraRemapper>());
    rclcpp::shutdown();
    return 0;
}
