#include "sub_sim_sensors/sim_torpedo_launcher.hpp"

#include <chrono>
#include <format>
#include <memory>
#include <string>
#include <sub_driver_interfaces/srv/launch_torpedo.hpp>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

SimTorpedoLauncher::SimTorpedoLauncher(const rclcpp::NodeOptions& options) : Node("torpedo_launcher", options) {
    this->declare_parameter<double>("launch_force", 2.5);    // N
    this->declare_parameter<double>("burst_duration", 0.1);  // s

    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    for (size_t i = 0; i < NUM_TORPEDOES; ++i) {
        torpedoes_[i].force_pub =
            this->create_publisher<std_msgs::msg::Float64>(std::format("sim/torpedo_{}/force", i), 10);
        torpedoes_[i].glue_client = this->create_client<std_srvs::srv::SetBool>(std::format("sim/torpedo_{}/glue", i),
                                                                                rclcpp::ServicesQoS(), cb_group_);
    }

    launch_srv_ = this->create_service<sub_driver_interfaces::srv::LaunchTorpedo>(
        "launch_torpedo",
        [this](const std::shared_ptr<sub_driver_interfaces::srv::LaunchTorpedo::Request> request,
               std::shared_ptr<sub_driver_interfaces::srv::LaunchTorpedo::Response> response) {
            this->launch_callback(request, response);
        },
        rclcpp::ServicesQoS(), cb_group_);
}

void SimTorpedoLauncher::launch_callback(
    const std::shared_ptr<sub_driver_interfaces::srv::LaunchTorpedo::Request> request,
    std::shared_ptr<sub_driver_interfaces::srv::LaunchTorpedo::Response> response) {
    if (!request->open) {
        response->success = true;
        response->message = "Close ignored in sim.";
        return;
    }

    if (request->torpedo_id >= NUM_TORPEDOES) {
        response->success = false;
        response->message = std::format("Invalid thruster id {}.", request->torpedo_id);
        return;
    }

    std::lock_guard<std::mutex> lock(launch_mutex_);
    Torpedo& torpedo = torpedoes_[request->torpedo_id];

    if (torpedo.launched) {
        response->success = false;
        response->message = "Torpedo already launched.";
        return;
    }

    if (!torpedo.glue_client->service_is_ready()) {
        response->success = false;
        response->message = "Glue service unavailable. Is the sim running?";
        return;
    }

    auto glue_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    glue_request->data = false;
    auto future = torpedo.glue_client->async_send_request(glue_request);

    if (future.wait_for(2s) != std::future_status::ready) {
        response->success = false;
        response->message = "Glue release did not respond.";
        return;
    }

    auto result = future.get();
    if (!result->success) {
        response->success = false;
        response->message = "Glue release failed: " + result->message;
        return;
    }

    torpedo.launched = true;

    const double force = this->get_parameter("launch_force").as_double();
    const double burst = this->get_parameter("burst_duration").as_double();

    std_msgs::msg::Float64 force_msg;
    force_msg.data = force;
    torpedo.force_pub->publish(force_msg);

    const auto period = std::chrono::duration<double>(burst);

    torpedo.burst_timer = this->create_timer(
        period,
        [this, &torpedo, torpedo_id = request->torpedo_id]() {
            torpedo.burst_timer->cancel();
            std_msgs::msg::Float64 zero_msg;
            zero_msg.data = 0.0;
            torpedo.force_pub->publish(zero_msg);
            RCLCPP_INFO(this->get_logger(), "%d burst complete.", torpedo_id);
        },
        cb_group_);

    RCLCPP_INFO(this->get_logger(), "%d launched (%g N for %g s).", request->torpedo_id, force, burst);
    response->success = true;
    response->message = "Torpedo launched.";
}

// Blocks on the glue service inside the launch callback, so it must run under a
// multithreaded executor (component_container_mt) with its reentrant callback group.
RCLCPP_COMPONENTS_REGISTER_NODE(SimTorpedoLauncher)
