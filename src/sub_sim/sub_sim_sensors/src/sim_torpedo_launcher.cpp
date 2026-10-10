#include "sub_sim_sensors/sim_torpedo_launcher.hpp"

#include <chrono>
#include <format>

#include "rclcpp_components/register_node_macro.hpp"

// Each torpedo in the scenario (torpedo.j2) is glued in its tube. Launching releases the glue
// and then pushes the torpedo along its axis for a short burst, standing in for the spring.
// The service answers once the glue has let go, and only one launch per torpedo: the sim
// cannot reload.
SimTorpedoLauncher::SimTorpedoLauncher(const rclcpp::NodeOptions& options) : Node("sim_torpedo_launcher", options) {
    // A 0.25 N s impulse. Drag takes the 16 g torpedo from there to about 1.7 m of travel,
    // and range grows only with the log of the impulse.
    this->declare_parameter<double>("launch_force", 2.5);    // N
    this->declare_parameter<double>("burst_duration", 0.1);  // s, on the sim clock

    for (size_t i = 0; i < NUM_TORPEDOES; ++i) {
        torpedoes_[i].force_pub =
            this->create_publisher<std_msgs::msg::Float64>(std::format("sim/torpedo_{}/force", i), 10);
        torpedoes_[i].glue_client = this->create_client<std_srvs::srv::SetBool>(std::format("sim/torpedo_{}/glue", i));
    }

    // Deferred responses: the answer goes out from the glue service's callback.
    launch_srv_ = this->create_service<LaunchTorpedo>(
        "launch_torpedo",
        [this](const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<LaunchTorpedo::Request> request) {
            launch_callback(header, request);
        });
}

void SimTorpedoLauncher::launch_callback(const std::shared_ptr<rmw_request_id_t>& header,
                                         const std::shared_ptr<LaunchTorpedo::Request>& request) {
    if (request->torpedo_id >= NUM_TORPEDOES) {
        respond(*header, false, std::format("Invalid torpedo id {}.", request->torpedo_id));
        return;
    }
    if (!request->open) {
        respond(*header, true, "Close ignored in sim.");
        return;
    }

    Torpedo& torpedo = torpedoes_[request->torpedo_id];
    if (torpedo.launched || torpedo.releasing) {
        respond(*header, false, std::format("Torpedo {} already launched.", request->torpedo_id));
        return;
    }
    if (!torpedo.glue_client->service_is_ready()) {
        respond(*header, false, "Glue service unavailable. Is the sim running?");
        return;
    }

    torpedo.releasing = true;
    auto release = std::make_shared<std_srvs::srv::SetBool::Request>();
    release->data = false;
    torpedo.glue_client->async_send_request(
        release, [this, header, id = request->torpedo_id](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            Torpedo& torpedo = torpedoes_[id];
            torpedo.releasing = false;
            const auto result = future.get();
            if (!result->success) {
                respond(*header, false, "Glue release failed: " + result->message);
                return;
            }
            torpedo.launched = true;
            push(id);
            respond(*header, true, "Torpedo launched.");
        });
}

void SimTorpedoLauncher::push(size_t id) {
    Torpedo& torpedo = torpedoes_[id];
    const double force = this->get_parameter("launch_force").as_double();
    const double burst = this->get_parameter("burst_duration").as_double();

    std_msgs::msg::Float64 msg;
    msg.data = force;
    torpedo.force_pub->publish(msg);

    torpedo.burst_timer = this->create_timer(std::chrono::duration<double>(burst), [this, id]() {
        Torpedo& torpedo = torpedoes_[id];
        torpedo.burst_timer->cancel();
        torpedo.force_pub->publish(std_msgs::msg::Float64());
    });

    RCLCPP_INFO(this->get_logger(), "Torpedo %zu launched (%g N for %g s).", id, force, burst);
}

void SimTorpedoLauncher::respond(rmw_request_id_t header, bool success, const std::string& message) {
    LaunchTorpedo::Response response;
    response.success = success;
    response.message = message;
    launch_srv_->send_response(header, response);
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimTorpedoLauncher)
