#include "sub_sim_sensors/sim_thrusters.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals;

SimThrusters::SimThrusters(const rclcpp::NodeOptions& options) : Node("sim_thrusters", options) {
    this->declare_parameter("effectiveness", std::vector<double>(NUM_THRUSTERS, 1.0));
    this->declare_parameter("esc_time_constant", time_constant_);
    // As sub_low's: a command older than this, or none for this long, stops every thruster [s].
    this->declare_parameter("command_timeout", command_timeout_);
    for (const char* name : {"effectiveness", "esc_time_constant", "command_timeout"}) {
        const rclcpp::Parameter param = this->get_parameter(name);
        if (const std::string reason = invalid(param); !reason.empty()) {
            throw std::invalid_argument(reason);
        }
        apply(param);
    }

    param_cb_handle_ = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        // All or nothing: a rejected set changes none of them.
        for (const auto& p : params) {
            result.reason = invalid(p);
            if (!result.reason.empty()) {
                result.successful = false;
                return result;
            }
        }
        for (const auto& p : params) {
            apply(p);
            if (p.get_name() == "effectiveness") {
                RCLCPP_WARN(this->get_logger(), "thruster effectiveness now [%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
                            effectiveness_[0], effectiveness_[1], effectiveness_[2], effectiveness_[3],
                            effectiveness_[4], effectiveness_[5], effectiveness_[6], effectiveness_[7]);
            }
        }
        result.successful = true;
        return result;
    });

    command_sub_ = this->create_subscription<sub_control_interfaces::msg::ThrusterCommand>(
        "control/thrusters", 10,
        [this](const sub_control_interfaces::msg::ThrusterCommand::SharedPtr msg) { command_callback(*msg); });

    setpoint_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("sim/thruster_setpoints", 10);
    timer_ = this->create_timer(10ms, [this]() { publish_setpoints(); });
}

std::string SimThrusters::invalid(const rclcpp::Parameter& param) {
    if (param.get_name() == "effectiveness") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY ||
            param.as_double_array().size() != NUM_THRUSTERS ||
            !std::ranges::all_of(param.as_double_array(), [](double e) { return e >= 0.0 && e <= 1.0; })) {
            return std::format("effectiveness must be {} doubles in [0, 1]", NUM_THRUSTERS);
        }
    } else if (param.get_name() == "esc_time_constant") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || !std::isfinite(param.as_double()) ||
            param.as_double() < 0.0) {
            return "esc_time_constant must be a finite, non-negative double";
        }
    } else if (param.get_name() == "command_timeout") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || !std::isfinite(param.as_double()) ||
            param.as_double() <= 0.0) {
            return "command_timeout must be a finite, positive double";
        }
    }
    return "";
}

void SimThrusters::apply(const rclcpp::Parameter& param) {
    if (param.get_name() == "effectiveness") {
        std::ranges::copy(param.as_double_array(), effectiveness_.begin());
    } else if (param.get_name() == "esc_time_constant") {
        time_constant_ = param.as_double();
    } else if (param.get_name() == "command_timeout") {
        command_timeout_ = param.as_double();
    }
}

void SimThrusters::command_callback(const sub_control_interfaces::msg::ThrusterCommand& msg) {
    const rclcpp::Time now = this->now();
    const rclcpp::Time stamp(msg.header.stamp, now.get_clock_type());
    // Like sub_low: a command that took too long to get here is not acted on.
    const double age = (now - stamp).seconds();
    if (age > command_timeout_) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "ignoring a thruster command %.2f s old",
                              age);
        return;
    }
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        // A NaN or inf stops the thruster, as sub_low does on the vehicle; a NaN would
        // also stick in the lag's state for good.
        double command = msg.command[i];
        if (!std::isfinite(command)) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "thruster %d commanded %f: stopping it",
                                  i, command);
            command = 0.0;
        }
        command_[i] = std::clamp(command, -MAX_COMMAND, MAX_COMMAND);
    }
    command_stamp_ = stamp;
}

void SimThrusters::publish_setpoints() {
    const rclcpp::Time now = this->now();
    const double dt = last_publish_.nanoseconds() == 0 ? 0.0 : (now - last_publish_).seconds();
    last_publish_ = now;
    // sub_control publishes every control cycle, killed or not; if it stops,
    // so do the thrusters (sub_low's watchdog, and the firmware's, on the vehicle).
    if (command_stamp_ && (now - *command_stamp_).seconds() > command_timeout_) {
        RCLCPP_ERROR(this->get_logger(), "no thruster commands for %.1f s: stopping all thrusters", command_timeout_);
        command_.fill(0.0);
        command_stamp_.reset();
    }
    const double blend = time_constant_ > 0.0 ? 1.0 - std::exp(-std::max(dt, 0.0) / time_constant_) : 1.0;

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(NUM_THRUSTERS);
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        output_[i] += blend * (command_[i] - output_[i]);
        msg.data[i] = effectiveness_[i] * output_[i];
    }
    setpoint_pub_->publish(msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimThrusters)
