#include "sub_sim_sensors/sim_grabber.hpp"

#include <chrono>
#include <format>
#include <limits>
#include <string_view>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace {
// The scenario's grasp glues are <namespace>/sim/grasp/<object>/glue.
constexpr std::string_view GLUE_SUFFIX = "/glue";

tf2::Transform to_transform(const geometry_msgs::msg::Pose& pose) {
    const auto& p = pose.position;
    const auto& q = pose.orientation;
    return tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(p.x, p.y, p.z));
}
}  // anonymous namespace

// Stonefish cannot grip by friction reliably, so closing the jaws with an object between them
// glues the object to the robot where it is, and opening them lets go. Where things are comes
// from Stonefish's ground truth: the robot's (sim/odometry) and the objects' (/sim/objects/
// <object>/odometry, at the middle of each).
SimGrabber::SimGrabber(const rclcpp::NodeOptions& options) : Node("sim_grabber", options) {
    this->declare_parameter<std::string>("left_joint", "left_grabber_joint");
    this->declare_parameter<std::string>("right_joint", "right_grabber_joint");
    // The left jaw closes with a positive angle, the right one with the negative of it.
    this->declare_parameter<double>("open_position", 0.0);    // rad
    this->declare_parameter<double>("closed_position", 0.8);  // rad
    // Where between the jaws an object is held, in base_link_ned, and how far from it the
    // middle of an object may be and still be grabbed.
    this->declare_parameter<std::vector<double>>("grasp_point", {0.0, 0.0, 0.0});  // m
    this->declare_parameter<double>("grasp_radius", 0.05);                         // m

    joint_setpoint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("sim/joint_setpoints", 10);
    vehicle_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "sim/odometry", 10, [this](const nav_msgs::msg::Odometry& msg) { vehicle_ = to_transform(msg.pose.pose); });

    // Deferred responses: a grab or release answers from the glue service's callback.
    set_grabber_srv_ = this->create_service<SetGrabber>(
        "set_grabber",
        [this](const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<SetGrabber::Request> request) {
            set_grabber_callback(header, request);
        });

    // Stonefish creates the glue services once it has loaded the scenario.
    discovery_timer_ = this->create_wall_timer(2s, [this]() { discover(); });
}

void SimGrabber::discover() {
    const std::string prefix = std::string(this->get_namespace()) + "/sim/grasp/";
    for (const auto& [service, types] : this->get_service_names_and_types()) {
        if (!service.starts_with(prefix) || !service.ends_with(GLUE_SUFFIX)) {
            continue;
        }
        const std::string name = service.substr(prefix.size(), service.size() - prefix.size() - GLUE_SUFFIX.size());
        if (name.empty() || name.find('/') != std::string::npos || graspables_.contains(name)) {
            continue;
        }
        Graspable& graspable = graspables_[name];
        graspable.glue = this->create_client<SetBool>(service);
        graspable.odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            std::format("/sim/objects/{}/odometry", name), 10, [this, name](const nav_msgs::msg::Odometry& msg) {
                const auto& p = msg.pose.pose.position;
                graspables_.at(name).position = tf2::Vector3(p.x, p.y, p.z);
            });
        RCLCPP_INFO(this->get_logger(), "%s can be grabbed.", name.c_str());
    }
}

void SimGrabber::set_grabber_callback(const std::shared_ptr<rmw_request_id_t>& header,
                                      const std::shared_ptr<SetGrabber::Request>& request) {
    if (busy_) {
        respond(*header, false, "Grabber busy with the last request.");
        return;
    }
    command_jaws(request->open);

    if (request->open) {
        if (held_.empty()) {
            respond(*header, true, "Grabber opened.");
            return;
        }
        set_glue(held_, false, *header, "Grabber opened, let go of " + held_ + ".");
        return;
    }

    if (!held_.empty()) {
        respond(*header, true, "Grabber closed, still holding " + held_ + ".");
        return;
    }
    const auto [closest, distance] = closest_graspable();
    if (closest.empty()) {
        respond(*header, true, "Grabber closed on nothing; there is nothing to grab.");
        return;
    }
    if (distance > this->get_parameter("grasp_radius").as_double()) {
        respond(*header, true,
                std::format("Grabber closed on nothing; {} is {:.2f} m from the grasp point.", closest, distance));
        return;
    }
    set_glue(closest, true, *header, "Grabber closed on " + closest + ".");
}

void SimGrabber::command_jaws(bool open) {
    const double angle = this->get_parameter(open ? "open_position" : "closed_position").as_double();
    sensor_msgs::msg::JointState setpoint;
    setpoint.header.stamp = this->now();
    setpoint.name = {this->get_parameter("left_joint").as_string(), this->get_parameter("right_joint").as_string()};
    setpoint.position = {angle, -angle};
    joint_setpoint_pub_->publish(setpoint);
}

void SimGrabber::set_glue(const std::string& name, bool hold, const rmw_request_id_t& header,
                          const std::string& message) {
    const Graspable& graspable = graspables_.at(name);
    if (!graspable.glue->service_is_ready()) {
        respond(header, false, "Glue service for " + name + " unavailable.");
        return;
    }
    auto request = std::make_shared<SetBool::Request>();
    request->data = hold;
    busy_ = true;
    graspable.glue->async_send_request(
        request, [this, name, hold, header, message](rclcpp::Client<SetBool>::SharedFuture future) {
            busy_ = false;
            const auto result = future.get();
            if (!result->success) {
                respond(header, false, "Glue for " + name + " failed: " + result->message);
                return;
            }
            held_ = hold ? name : "";
            RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
            respond(header, true, message);
        });
}

std::pair<std::string, double> SimGrabber::closest_graspable() const {
    std::pair<std::string, double> closest{"", std::numeric_limits<double>::infinity()};
    if (!vehicle_) {
        RCLCPP_WARN(this->get_logger(), "No ground truth on sim/odometry yet, so nothing to grab.");
        return closest;
    }
    const auto point = this->get_parameter("grasp_point").as_double_array();
    const tf2::Vector3 grasp_point(point.at(0), point.at(1), point.at(2));
    for (const auto& [name, graspable] : graspables_) {
        if (!graspable.position) {
            continue;
        }
        // In base_link_ned.
        const double distance = (vehicle_->inverse() * *graspable.position).distance(grasp_point);
        if (distance < closest.second) {
            closest = {name, distance};
        }
    }
    return closest;
}

void SimGrabber::respond(rmw_request_id_t header, bool success, const std::string& message) {
    SetGrabber::Response response;
    response.success = success;
    response.message = message;
    set_grabber_srv_->send_response(header, response);
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimGrabber)
