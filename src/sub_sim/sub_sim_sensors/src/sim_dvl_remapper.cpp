#include "sub_sim_sensors/sim_dvl_remapper.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "marine_acoustic_msgs/msg/dvl.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "stonefish_ros2/msg/dvl.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

SimDVLRemapper::SimDVLRemapper() : Node("sim_dvl_remapper") {
    this->declare_parameter("robot_name", "");
    robot_name_ = this->get_parameter("robot_name").as_string();

    subscriber_ = this->create_subscription<stonefish_ros2::msg::DVL>(robot_name_ + "/sim/dvl", 10, std::bind(&SimDVLRemapper::dvl_callback, this, _1));
    dvl_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(robot_name_ + "/sim/dvl_imu", 10, std::bind(&SimDVLRemapper::imu_callback, this, _1));

    vel_publisher_ = this->create_publisher<marine_acoustic_msgs::msg::Dvl>(robot_name_ + "/dvl", 10);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(robot_name_ + "/dvl_odom", 10);
}

void SimDVLRemapper::dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg_stonefish) {
    marine_acoustic_msgs::msg::Dvl msg_marine{};

    msg_marine.header = msg_stonefish->header;
    msg_marine.header.frame_id = robot_name_ + "/dvl_link";
    msg_marine.velocity.x = msg_stonefish->velocity.x;
    msg_marine.velocity.y = msg_stonefish->velocity.y;
    msg_marine.velocity.z = msg_stonefish->velocity.z;
    msg_marine.altitude = msg_stonefish->altitude;

    msg_marine.velocity_covar = msg_stonefish->velocity_covariance;

    size_t num_beams{std::min(static_cast<size_t>(4), msg_stonefish->beams.size())};
    for (size_t i = 0; i < num_beams; ++i) {
        const auto& b = msg_stonefish->beams[i];
        msg_marine.range[i] = b.range;
        msg_marine.beam_velocity[i] = b.velocity;
        msg_marine.beam_quality[i] = (b.range >= 0) ? 255.0f : 0.0f;
    }

    msg_marine.beam_ranges_valid = true;
    msg_marine.beam_velocities_valid = true;
    msg_marine.num_good_beams = 4;

    msg_marine.course_gnd = std::atan2(msg_stonefish->velocity.y, msg_stonefish->velocity.x);
    msg_marine.speed_gnd = std::hypot(msg_stonefish->velocity.x, msg_stonefish->velocity.y);

    msg_marine.velocity_mode = marine_acoustic_msgs::msg::Dvl::DVL_MODE_BOTTOM;
    msg_marine.dvl_type = marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PISTON;

    vel_publisher_->publish(msg_marine);


    nav_msgs::msg::Odometry odom_msg{};
    odom_msg.header = msg_marine.header;

    odom_msg.twist.twist.linear.x = msg_marine.velocity.x;
    odom_msg.twist.twist.linear.y = msg_marine.velocity.y;
    odom_msg.twist.twist.linear.z = msg_marine.velocity.z;
    odom_msg.twist.covariance[0] = msg_marine.velocity_covar[0];
    odom_msg.twist.covariance[7] = msg_marine.velocity_covar[4];
    odom_msg.twist.covariance[14] = msg_marine.velocity_covar[8];

    odom_publisher_->publish(odom_msg);
}

void SimDVLRemapper::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    rpy_ = {msg->orientation.x, msg->orientation.y, msg->orientation.z};
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimDVLRemapper>());
    rclcpp::shutdown();
    return 0;
}