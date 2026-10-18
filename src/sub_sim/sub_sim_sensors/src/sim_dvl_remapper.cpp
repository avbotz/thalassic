#include "sub_sim_sensors/sim_dvl_remapper.hpp"
#include <rclcpp/qos.hpp>

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "stonefish_ros2/msg/dvl.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

using std::placeholders::_1;

SimDVLRemapper::SimDVLRemapper(const rclcpp::NodeOptions& options) : Node("sim_dvl_remapper", options) {
    this->declare_parameter("dvl_link", "dvl_link");
    // Least variance reported with each velocity. Stonefish gives zero when no noise
    // is configured, which would tell the EKF the DVL is perfect.
    this->declare_parameter("velocity_variance", 1e-4);
    this->declare_parameter("max_altitude", 50.0);
    dvl_link_ = this->get_parameter("dvl_link").as_string();
    velocity_variance_ = this->get_parameter("velocity_variance").as_double();
    max_altitude_ = this->get_parameter("max_altitude").as_double();
    if (!(velocity_variance_ > 0.0) || !(max_altitude_ > 0.0)) {
        throw std::invalid_argument("velocity_variance and max_altitude must be positive");
    }

    subscriber_ = this->create_subscription<stonefish_ros2::msg::DVL>(
        "sim/dvl", 10, std::bind(&SimDVLRemapper::dvl_callback, this, _1));

    vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "dvl/velocity", rclcpp::SystemDefaultsQoS());
    altitude_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("altitude", rclcpp::SystemDefaultsQoS());
}

void SimDVLRemapper::dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg_stonefish) {
    // Without bottom lock Stonefish reports zero velocity and an altitude of -1.
    if (msg_stonefish->altitude < 0.0 || !std::isfinite(msg_stonefish->velocity.x) ||
        !std::isfinite(msg_stonefish->velocity.y) || !std::isfinite(msg_stonefish->velocity.z)) {
        return;
    }

    geometry_msgs::msg::TwistWithCovarianceStamped velocity_msg{};

    velocity_msg.header = msg_stonefish->header;
    velocity_msg.header.frame_id = dvl_link_;

    velocity_msg.twist.twist.linear.x = msg_stonefish->velocity.x;
    velocity_msg.twist.twist.linear.y = msg_stonefish->velocity.y;
    velocity_msg.twist.twist.linear.z = msg_stonefish->velocity.z;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            velocity_msg.twist.covariance[i * 6 + j] = msg_stonefish->velocity_covariance[i * 3 + j];
        }
        velocity_msg.twist.covariance[i * 7] = std::max(velocity_msg.twist.covariance[i * 7], velocity_variance_);
    }

    vel_publisher_->publish(velocity_msg);

    if (std::isfinite(msg_stonefish->altitude)) {
        sensor_msgs::msg::Range range;
        range.header = velocity_msg.header;
        range.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        range.min_range = 0.05F;
        range.max_range = static_cast<float>(max_altitude_);
        range.range = static_cast<float>(msg_stonefish->altitude);
        altitude_publisher_->publish(range);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimDVLRemapper)
