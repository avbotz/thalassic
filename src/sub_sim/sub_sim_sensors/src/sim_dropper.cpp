#include "sub_sim_sensors/sim_dropper.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

SimDropper::SimDropper(const rclcpp::NodeOptions& options) : Node("sim_dropper", options) {
    this->declare_parameter<std::string>("joint_name", "dropper_joint");
    this->declare_parameter<double>("open_position", 1.5);  // rad
    this->declare_parameter<double>("release_angle", 1.0);  // rad

    glue_client_ = this->create_client<std_srvs::srv::SetBool>("sim/dropper_ball/glue");

    joint_setpoint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("sim/joint_setpoints", 10);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "sim/joint_states", 10, [this](const sensor_msgs::msg::JointState& msg) { this->joint_state_callback(msg); });

    set_dropper_srv_ = this->create_service<sub_driver_interfaces::srv::SetDropper>(
        "set_dropper", [this](const std::shared_ptr<sub_driver_interfaces::srv::SetDropper::Request> request,
                              std::shared_ptr<sub_driver_interfaces::srv::SetDropper::Response> response) {
            this->set_dropper_callback(request, response);
        });
}

void SimDropper::set_dropper_callback(const std::shared_ptr<sub_driver_interfaces::srv::SetDropper::Request> request,
                                      std::shared_ptr<sub_driver_interfaces::srv::SetDropper::Response> response) {
    const double position = request->open ? this->get_parameter("open_position").as_double() : 0.0;

    sensor_msgs::msg::JointState setpoint;
    setpoint.header.stamp = this->get_clock()->now();
    setpoint.name.push_back(this->get_parameter("joint_name").as_string());
    setpoint.position.push_back(position);
    joint_setpoint_pub_->publish(setpoint);

    RCLCPP_INFO(this->get_logger(), "Dropper servo commanded to %g rad.", position);
    response->success = true;
    response->message = request->open ? "Dropper opening." : "Dropper closing.";
}

void SimDropper::joint_state_callback(const sensor_msgs::msg::JointState& msg) {
    if (released_ || request_in_flight_) {
        return;
    }

    const std::string joint_name = this->get_parameter("joint_name").as_string();
    const double release_angle = this->get_parameter("release_angle").as_double();

    for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
        if (msg.name[i] != joint_name) {
            continue;
        }

        if (std::abs(msg.position[i]) < release_angle) {
            return;
        }

        if (!glue_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Dropper servo turned but glue service is unavailable.");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false;
        request_in_flight_ = true;

        glue_client_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            request_in_flight_ = false;
            const auto result = future.get();
            if (result->success) {
                released_ = true;
                RCLCPP_INFO(this->get_logger(), "Dropper ball released.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Glue release failed: %s", result->message.c_str());
            }
        });
        return;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimDropper)
