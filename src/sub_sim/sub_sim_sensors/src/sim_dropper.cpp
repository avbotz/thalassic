#include "sub_sim_sensors/sim_dropper.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

SimDropper::SimDropper(const rclcpp::NodeOptions& options) : Node("sim_dropper", options) {
    this->declare_parameter<std::string>("joint_name", "dropper_joint");
    this->declare_parameter<double>("open_position", 1.5);  // rad
    this->declare_parameter<double>("release_angle", 1.0);  // rad

    glue_clients_[0] = this->create_client<std_srvs::srv::SetBool>("sim/dropper_ball/glue");
    glue_clients_[1] = this->create_client<std_srvs::srv::SetBool>("sim/dropper_ball_1/glue");

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
    if (request->dropper_id >= glue_clients_.size()) {
        response->success = false;
        response->message = "dropper_id must be 0 or 1.";
        return;
    }

    if (request->open && released_[request->dropper_id]) {
        response->success = false;
        response->message = "Marker has already been released.";
        return;
    }

    if (request->open && pending_dropper_id_) {
        response->success = false;
        response->message = "Another marker release is still in progress.";
        return;
    }

    const double position = request->open ? this->get_parameter("open_position").as_double() : 0.0;

    sensor_msgs::msg::JointState setpoint;
    setpoint.header.stamp = this->get_clock()->now();
    setpoint.name.push_back(this->get_parameter("joint_name").as_string());
    setpoint.position.push_back(position);
    joint_setpoint_pub_->publish(setpoint);

    if (request->open) {
        pending_dropper_id_ = request->dropper_id;
    }

    RCLCPP_INFO(this->get_logger(), "Dropper %u servo commanded to %g rad.",
                static_cast<unsigned int>(request->dropper_id), position);
    response->success = true;
    response->message = request->open ? "Dropper opening." : "Dropper closing.";
}

void SimDropper::joint_state_callback(const sensor_msgs::msg::JointState& msg) {
    if (!pending_dropper_id_) {
        return;
    }
    const std::uint8_t dropper_id = *pending_dropper_id_;
    if (released_[dropper_id] || request_in_flight_[dropper_id]) {
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

        auto& glue_client = glue_clients_[dropper_id];
        if (!glue_client->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Dropper servo turned but glue service is unavailable.");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false;
        request_in_flight_[dropper_id] = true;

        glue_client->async_send_request(request, [this, dropper_id](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            request_in_flight_[dropper_id] = false;
            const auto result = future.get();
            if (result->success) {
                released_[dropper_id] = true;
                pending_dropper_id_.reset();
                RCLCPP_INFO(this->get_logger(), "Dropper %u marker released.", static_cast<unsigned int>(dropper_id));
            } else {
                RCLCPP_ERROR(this->get_logger(), "Glue release failed: %s", result->message.c_str());
            }
        });
        return;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimDropper)
