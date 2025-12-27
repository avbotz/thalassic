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

std::array<double, 3> offsets_to_frame(std::array<double, 3> xyz, std::array<double, 3> rpy) {
    /*
     * Input offsets at those angles from your frame,
     * Converts those offsets to your frame's offsets.
     * Input = x, y, z relative offsets
     * Angles = roll, pitch, yaw from your frame the input is applied at
     * Output = your frame's x, y, z absolute offsets
     */

    // Pre-compute trig functions.
    double sphi = std::sin(rpy[0]);
    double sthe = std::sin(rpy[1]);
    double spsi = std::sin(rpy[2]);
    double cphi = std::cos(rpy[0]);
    double cthe = std::cos(rpy[1]);
    double cpsi = std::cos(rpy[2]);

    // Calculate rotation matrix for Euler transformation.
    double r11 = cthe * cpsi;
    double r12 = -cphi * spsi + sphi * sthe * cpsi;
    double r13 = -sphi * spsi + cphi * sthe * cpsi;
    double r21 = cthe * spsi;
    double r22 = cphi * cpsi + sphi * sthe * spsi;
    double r23 = sphi * cpsi + cphi * sthe * spsi;
    double r31 = -sthe;
    double r32 = sphi * cthe;
    double r33 = cphi * cthe;

    std::array<double, 3> output;

    // Calculate matrix.
    output[0] = r11 * xyz[0] + r12 * xyz[1] + r13 * xyz[2];
    output[1] = r21 * xyz[0] + r22 * xyz[1] + r23 * xyz[2];
    output[2] = r31 * xyz[0] + r32 * xyz[1] + r33 * xyz[2];

    return output;
}

SimDVLRemapper::SimDVLRemapper() : Node("sim_dvl_remapper"), last_time_{0} {
    this->declare_parameter("robot_name", "");
    robot_name_ = this->get_parameter("robot_name").as_string();

    subscriber_ = this->create_subscription<stonefish_ros2::msg::DVL>(robot_name_ + "/sim/dvl", 10, std::bind(&SimDVLRemapper::dvl_callback, this, _1));
    dvl_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(robot_name_ + "/sim/dvl_imu", 10, std::bind(&SimDVLRemapper::imu_callback, this, _1));

    vel_publisher_ = this->create_publisher<marine_acoustic_msgs::msg::Dvl>(robot_name_ + "/dvl", 10);
    pos_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(robot_name_ + "/dead_reckoning", 10);
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

    if (last_time_.nanoseconds() == 0) {
        last_time_ = this->get_clock()->now();
    }

    rclcpp::Time now = this->get_clock()->now();
    std::array<double, 3> delta_pos{
        msg_marine.velocity.x * (now - last_time_).seconds(),
        msg_marine.velocity.y * (now - last_time_).seconds(),
        msg_marine.velocity.z * (now - last_time_).seconds()};
    std::array<double, 3> rotated_delta = offsets_to_frame(delta_pos, rpy_);
    accumulated_pos_[0] += rotated_delta[0];
    accumulated_pos_[1] += rotated_delta[1];
    accumulated_pos_[2] += rotated_delta[2];
    last_time_ = now;

    nav_msgs::msg::Odometry odom_msg{};
    odom_msg.header = msg_marine.header;
    // odom_msg.header.frame_id = robot_name_ + "/dvl_imu_link";

    odom_msg.twist.twist.linear.x = msg_marine.velocity.x;
    odom_msg.twist.twist.linear.y = msg_marine.velocity.y;
    odom_msg.twist.twist.linear.z = msg_marine.velocity.z;
    odom_msg.twist.covariance[0] = msg_marine.velocity_covar[0];
    odom_msg.twist.covariance[7] = msg_marine.velocity_covar[4];
    odom_msg.twist.covariance[14] = msg_marine.velocity_covar[8];

    // odom_msg.pose.pose.position.x = accumulated_pos_[0];
    // odom_msg.pose.pose.position.y = accumulated_pos_[1];
    // odom_msg.pose.pose.position.z = accumulated_pos_[2];

    // tf2::Quaternion q_tf2;
    // q_tf2.setRPY(rpy_[0], rpy_[1], rpy_[2]);

    // odom_msg.pose.pose.orientation = tf2::toMsg(q_tf2);

    // odom_msg.pose.covariance[0] = msg_marine.velocity_covar[0];
    // odom_msg.pose.covariance[7] = msg_marine.velocity_covar[4];
    // odom_msg.pose.covariance[14] = msg_marine.velocity_covar[8];
    // // Orientation covariance is not provided by the DVL
    // // Set -1 to indicate unknown
    // odom_msg.pose.covariance[21] = 0;
    // odom_msg.pose.covariance[28] = 0;
    // odom_msg.pose.covariance[35] = 0;

    pos_publisher_->publish(odom_msg);

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