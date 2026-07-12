#pragma once

#include <array>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sub_control_interfaces/msg/setpoint.hpp"
#include "sub_control_interfaces/msg/spin.hpp"

// Shared helpers for the BT action nodes (node.cpp, vision.cpp). The mission
// speaks the workspace's REP-103 convention natively -- world ENU (z up, so
// underwater is negative; yaw counter-clockwise-positive) and body FLU
// (x forward, y left, z up) -- the same frames sub_control consumes, so these
// builders fill the Setpoint messages verbatim. The only frame conversion left
// in sub_mission is camera-optical bearings (positive = right of / below
// center) into ENU/FLU steering, which lives at the vision call sites.

using SetpointMsg = sub_control_interfaces::msg::Setpoint;
using SetpointPublisher = rclcpp::Publisher<SetpointMsg>;
using PointCmdMsg = SetpointMsg;
using QuaternionCmdMsg = SetpointMsg;
using VectorCmdMsg = SetpointMsg;
using PointCmdPublisher = SetpointPublisher;
using QuaternionCmdPublisher = SetpointPublisher;
using VectorCmdPublisher = SetpointPublisher;
using SpinCmdMsg = sub_control_interfaces::msg::Spin;
using SpinCmdPublisher = rclcpp::Publisher<SpinCmdMsg>;

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

inline SetpointMsg positionCommand(const rclcpp::Clock &, const std::array<double, 3> &target) {
    SetpointMsg msg;
    msg.velocity = false;
    msg.setpoint.x = target[0];
    msg.setpoint.y = target[1];
    msg.setpoint.z = target[2];
    return msg;
}

inline SetpointMsg linearVelocityCommand(const rclcpp::Clock &, const std::array<double, 3> &target) {
    SetpointMsg msg;
    msg.velocity = true;
    msg.setpoint.x = target[0];
    msg.setpoint.y = target[1];
    msg.setpoint.z = target[2];
    return msg;
}

inline SetpointMsg attitudeCommand(const rclcpp::Clock &, const std::array<double, 3> &target) {
    SetpointMsg msg;
    msg.velocity = false;
    msg.setpoint.roll = target[0];
    msg.setpoint.pitch = target[1];
    msg.setpoint.yaw = target[2];
    return msg;
}

inline SetpointMsg angularVelocityCommand(const rclcpp::Clock &, const std::array<double, 3> &target) {
    SetpointMsg msg;
    msg.velocity = true;
    msg.setpoint.roll = target[0];
    msg.setpoint.pitch = target[1];
    msg.setpoint.yaw = target[2];
    return msg;
}

inline SpinCmdMsg spinCommand(const rclcpp::Clock &, const double yaw, const double max_rate) {
    SpinCmdMsg msg;
    msg.yaw = yaw;
    msg.max_rate = max_rate;
    return msg;
}
