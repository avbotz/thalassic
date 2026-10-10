#pragma once

#include <sys/types.h>
#include <unistd.h>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// Just enough of a node to watch the kill switch and hand the mission's
// parameters on to the `mission` process it runs. Named apart from that
// process's node, which lives in the same namespace.
class RestartNode : public rclcpp::Node {
   public:
    RestartNode() : rclcpp::Node("mission_restart") {
        mission = this->declare_parameter<std::string>("mission", "");
        role = this->declare_parameter<std::string>("role", "SURVEY");
        groot_port = this->declare_parameter<std::int64_t>("groot_port", 5555);
        // `mission` checks these too, but only once the kill switch is
        // released, in the water.
        if (mission.empty()) {
            throw std::invalid_argument("sub_mission restart needs a mission: -p mission:=<name or path>");
        }
        if (role != "SURVEY" && role != "SEARCH") {
            throw std::invalid_argument("sub_mission role must be SURVEY or SEARCH");
        }
        if (groot_port < 0 || groot_port > 65534) {
            throw std::invalid_argument("sub_mission groot_port must be 0 (off) or a port up to 65534");
        }
        // Deeper than the one state the publisher latches, so that a kill and
        // its release both arrive even if they come between two polls.
        kill_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "kill_switch", rclcpp::QoS(10).transient_local(),
            [this](const std_msgs::msg::Bool &msg) { kill_callback(msg); });
    }

    bool subAlive() const { return !killed_; }
    // Times the kill switch has gone from released to killed.
    std::uint64_t kills() const { return kills_; }

    std::string mission;
    std::string role;
    std::int64_t groot_port;

   private:
    void kill_callback(const std_msgs::msg::Bool &msg) {
        if (msg.data && !killed_) {
            ++kills_;
        }
        killed_ = msg.data;
    }

    bool killed_ = true;
    std::uint64_t kills_ = 0;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub_;
};
