#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sub_control_interfaces/msg/motion_setpoint.hpp"

// Shared helpers for the BT action nodes (node.cpp, vision.cpp), building the
// MotionSetpoint sub_control consumes. The mission works in the same frames
// as sub_control: positions and yaw in odom (ENU: z up, so underwater is
// negative; yaw counter-clockwise-positive), velocities along the body (FLU:
// x forward, y left, z up). So targets go into the message unchanged; the only
// conversion left is from camera-optical bearings, at the vision call sites.
//
// Every command goes out on the one motion_setpoint topic, so they reach
// sub_control in the order they were sent. Each sets only the axes it is
// given and leaves the rest as they were (KEEP): a forward velocity keeps
// holding depth, a depth change keeps the heading. Axes are given as NaN to
// leave them alone. MissionNode records what was sent (commandPosition etc.).

using SetpointMsg = sub_control_interfaces::msg::MotionSetpoint;
using SetpointPublisher = rclcpp::Publisher<SetpointMsg>;

// How close control/error must come for a move to count as arrived.
constexpr double POSITION_TOLERANCE = 0.25;    // m
constexpr double ANGLE_TOLERANCE = 0.0872665;  // rad, 5 deg
constexpr double UNSPECIFIED_PORT = std::numeric_limits<double>::quiet_NaN();

// The same angle in [-pi, pi]. An infinite angle (a bad port value or
// bearing) gives NaN, which the commands below leave alone.
inline double normalizeAngle(const double angle) { return std::remainder(angle, 2.0 * M_PI); }

// The longest a position or attitude move may take before it counts as stuck
// (a current, a dead thruster, an altitude hold without bottom lock): its size
// at about a third of sub_control's speed limits (0.5 m/s, 0.3 m/s, 0.6 rad/s),
// plus time to settle. Generous, so a slow but healthy move still finishes;
// bounded, so one that cannot finish fails instead of stalling the run.
inline std::chrono::milliseconds moveTimeout(const double horizontal, const double vertical, const double angle) {
    constexpr double SETTLE_S = 20.0;
    constexpr double HORIZONTAL_SPEED = 0.15;  // m/s
    constexpr double VERTICAL_SPEED = 0.1;     // m/s
    constexpr double YAW_RATE = 0.2;           // rad/s
    const double seconds = SETTLE_S + std::fabs(horizontal) / HORIZONTAL_SPEED + std::fabs(vertical) / VERTICAL_SPEED +
                           std::fabs(angle) / YAW_RATE;
    return std::chrono::milliseconds(static_cast<std::int64_t>(seconds * 1000.0));
}

inline SetpointMsg command(const rclcpp::Clock &clock) {
    SetpointMsg msg;
    msg.header.stamp = clock.now();
    msg.mode.fill(SetpointMsg::KEEP);
    return msg;
}

// The axes given ([x, y, z, roll, pitch, yaw]) come to rest and hold wherever
// they stop, ending any velocity or target they had; the rest keep theirs.
// x and y share a mode, so holding either holds both.
inline SetpointMsg holdCommand(const rclcpp::Clock &clock, const std::array<bool, 6> &axes) {
    SetpointMsg msg = command(clock);
    for (std::size_t i = 0; i < axes.size(); ++i) {
        if (axes[i] || (i < 2 && axes[1 - i])) {
            msg.mode[i] = SetpointMsg::HOLD;
        }
    }
    return msg;
}

// x, y, z and yaw come to rest and hold wherever they stop, ending any
// velocity or spin still running; roll and pitch keep holding level.
inline SetpointMsg holdCommand(const rclcpp::Clock &clock) {
    return holdCommand(clock, {true, true, true, false, false, true});
}

// x and y share a mode in sub_control, so give both or neither. `altitude`
// makes z a height above the bottom instead of an odom z.
inline SetpointMsg positionCommand(const rclcpp::Clock &clock, const std::array<double, 3> &target,
                                   const bool altitude) {
    SetpointMsg msg = command(clock);
    for (std::size_t i = 0; i < 3; ++i) {
        if (std::isfinite(target[i])) {
            msg.mode[i] = SetpointMsg::POSITION;
            msg.position[i] = target[i];
        }
    }
    msg.altitude = altitude;
    return msg;
}

// Axes given as NaN keep what they were doing. x and y go together (they share
// a mode in sub_control), so giving either sets both, the other to zero.
// The velocity holds until replaced.
inline SetpointMsg linearVelocityCommand(const rclcpp::Clock &clock, const std::array<double, 3> &target) {
    SetpointMsg msg = command(clock);
    msg.velocity_frame = SetpointMsg::HEADING;
    if (std::isfinite(target[0]) || std::isfinite(target[1])) {
        msg.mode[0] = msg.mode[1] = SetpointMsg::VELOCITY;
        msg.velocity[0] = std::isfinite(target[0]) ? target[0] : 0.0;
        msg.velocity[1] = std::isfinite(target[1]) ? target[1] : 0.0;
    }
    if (std::isfinite(target[2])) {
        msg.mode[2] = SetpointMsg::VELOCITY;
        msg.velocity[2] = target[2];
    }
    return msg;
}

inline SetpointMsg attitudeCommand(const rclcpp::Clock &clock, const std::array<double, 3> &target) {
    SetpointMsg msg = command(clock);
    for (std::size_t i = 0; i < 3; ++i) {
        if (std::isfinite(target[i])) {
            msg.mode[3 + i] = SetpointMsg::POSITION;
            msg.position[3 + i] = target[i];
        }
    }
    return msg;
}

// Axes given as NaN keep what they were doing (e.g. stay level while spinning).
inline SetpointMsg angularVelocityCommand(const rclcpp::Clock &clock, const std::array<double, 3> &target) {
    SetpointMsg msg = command(clock);
    for (std::size_t i = 0; i < 3; ++i) {
        if (std::isfinite(target[i])) {
            msg.mode[3 + i] = SetpointMsg::VELOCITY;
            msg.velocity[3 + i] = target[i];
        }
    }
    return msg;
}
