#include "sub_drivers_mappings/dvl_odom_remapper.hpp"

#include "marine_acoustic_msgs/msg/dvl.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

DVLOdomRemapper::DVLOdomRemapper() : Node("dvl_odom_remapper") {
    subscriber_ = this->create_subscription<marine_acoustic_msgs::msg::Dvl>("dvl", 10, std::bind(&DVLOdomRemapper::dvl_callback, this, _1));
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("dvl_odom", 10);
    altitude_publisher_ = this->create_publisher<std_msgs::msg::Float64>("dvl_altitude", 10);
}

void DVLOdomRemapper::dvl_callback(const marine_acoustic_msgs::msg::Dvl::SharedPtr msg) {
    nav_msgs::msg::Odometry odom_msg{};
    odom_msg.header = msg->header;
    odom_msg.header.frame_id = "marlin_v2/odom";
    odom_msg.child_frame_id = "marlin_v2/dvl_link";

    odom_msg.twist.twist.linear.x = msg->velocity.x;
    odom_msg.twist.twist.linear.y = msg->velocity.y;
    odom_msg.twist.twist.linear.z = msg->velocity.z;
    odom_msg.twist.covariance[0] = msg->velocity_covar[0];
    odom_msg.twist.covariance[7] = msg->velocity_covar[4];
    odom_msg.twist.covariance[14] = msg->velocity_covar[8];

    odom_publisher_->publish(odom_msg);

    std_msgs::msg::Float64 alt_msg;
    alt_msg.data = msg->altitude;
    altitude_publisher_->publish(alt_msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DVLOdomRemapper>());
    rclcpp::shutdown();
    return 0;
}