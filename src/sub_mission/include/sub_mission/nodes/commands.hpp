#pragma once

#include <array>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sub_control_interfaces/msg/setpoint.hpp"

// Shared helpers for the BT action nodes (node.cpp, vision.cpp): conversion
// from the mission frame (x forward, y right, z down, yaw clockwise-positive)
// to the ENU/FLU command messages sub_control consumes. Keep every mission ->
// control conversion here so the sign flips live in exactly one place.

using SetpointMsg = sub_control_interfaces::msg::Setpoint;
using SetpointPublisher = rclcpp::Publisher<SetpointMsg>;
using PointCmdMsg = SetpointMsg;
using QuaternionCmdMsg = SetpointMsg;
using VectorCmdMsg = SetpointMsg;
using PointCmdPublisher = SetpointPublisher;
using QuaternionCmdPublisher = SetpointPublisher;
using VectorCmdPublisher = SetpointPublisher;

constexpr double POSITION_TOLERANCE = 0.25;
constexpr double ANGLE_TOLERANCE = 0.0872665;
constexpr double UNSPECIFIED_PORT = std::numeric_limits<double>::quiet_NaN();

inline double normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

inline SetpointMsg positionCommand(const rclcpp::Clock &, const std::array<double, 3> &target, const bool altitude) {
    SetpointMsg msg;
    msg.velocity = false;
    msg.altitude = altitude;
    msg.setpoint.x = target[0];
    msg.setpoint.y = -target[1];
    msg.setpoint.z = altitude ? target[2] : -target[2];
    return msg;
}

inline SetpointMsg linearVelocityCommand(const rclcpp::Clock &, const std::array<double, 3> &target) {
    SetpointMsg msg;
    msg.velocity = true;
    msg.altitude = false;
    msg.setpoint.x = target[0];
    msg.setpoint.y = -target[1];
    msg.setpoint.z = -target[2];
    return msg;
}

inline SetpointMsg attitudeCommand(const rclcpp::Clock &, const std::array<double, 3> &target) {
    SetpointMsg msg;
    msg.velocity = false;
    msg.altitude = false;
    msg.setpoint.roll = target[0];
    msg.setpoint.pitch = -target[1];
    msg.setpoint.yaw = -target[2];
    return msg;
}

inline SetpointMsg angularVelocityCommand(const rclcpp::Clock &, const std::array<double, 3> &target) {
    SetpointMsg msg;
    msg.velocity = true;
    msg.altitude = false;
    msg.setpoint.roll = target[0];
    msg.setpoint.pitch = -target[1];
    msg.setpoint.yaw = -target[2];
    return msg;
}
