#pragma once

#include <array>
#include <cmath>
#include <string>

#include "behaviortree_cpp/tree_node.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// Shared helpers for the BT action nodes (node.cpp, vision.cpp): conversion
// from the mission frame (x forward, y right, z down, yaw clockwise-positive)
// to the ENU/FLU command messages sub_control consumes. Keep every mission ->
// control conversion here so the sign flips live in exactly one place.

using PointCmdMsg = geometry_msgs::msg::PointStamped;
using QuaternionCmdMsg = geometry_msgs::msg::QuaternionStamped;
using VectorCmdMsg = geometry_msgs::msg::Vector3Stamped;
using PointCmdPublisher = rclcpp::Publisher<PointCmdMsg>;
using QuaternionCmdPublisher = rclcpp::Publisher<QuaternionCmdMsg>;
using VectorCmdPublisher = rclcpp::Publisher<VectorCmdMsg>;

constexpr double POSITION_TOLERANCE = 0.25;
constexpr double ANGLE_TOLERANCE = 0.0872665;
constexpr const char *ALTITUDE_FRAME = "altitude";

inline double normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

inline bool portProvided(const BT::TreeNode &node, const std::string &port) {
    return node.config().input_ports.find(port) != node.config().input_ports.end();
}

inline QuaternionCmdMsg quaternionCommand(const rclcpp::Clock &clock, const double roll, const double pitch,
                                          const double yaw) {
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    QuaternionCmdMsg msg;
    msg.header.stamp = clock.now();
    msg.header.frame_id = "base_link";
    msg.quaternion.w = cr * cp * cy + sr * sp * sy;
    msg.quaternion.x = sr * cp * cy - cr * sp * sy;
    msg.quaternion.y = cr * sp * cy + sr * cp * sy;
    msg.quaternion.z = cr * cp * sy - sr * sp * cy;
    return msg;
}

inline PointCmdMsg positionCommand(const rclcpp::Clock &clock, const std::array<double, 3> &target,
                                   const bool use_altitude) {
    PointCmdMsg msg;
    msg.header.stamp = clock.now();
    msg.header.frame_id = use_altitude ? ALTITUDE_FRAME : "base_link";
    msg.point.x = target[0];
    msg.point.y = -target[1];
    msg.point.z = use_altitude ? target[2] : -target[2];
    return msg;
}

inline VectorCmdMsg vectorCommand(const rclcpp::Clock &clock, const std::array<double, 3> &target) {
    VectorCmdMsg msg;
    msg.header.stamp = clock.now();
    msg.header.frame_id = "base_link";
    msg.vector.x = target[0];
    msg.vector.y = -target[1];
    msg.vector.z = -target[2];
    return msg;
}
