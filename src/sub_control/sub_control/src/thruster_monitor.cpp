// Declares a thruster failed when it stops producing the thrust it is
// commanded to, and publishes thruster_health for sub_control to allocate
// around it (docs/control.md#thruster-failure).
//
// It compares each thruster's command (control/thruster_<i>, through the T200
// curve) with the thrust measured on thrusters/measured_thrust. In simulation
// sim_thrusters provides that from Stonefish; on the vehicle it needs a source
// such as per-thruster current sensing or ESC RPM telemetry, which the Basic
// ESCs do not have. The estimate is a least-squares effectiveness over a
// sliding window, judged only while the thruster is underwater and commanded
// hard enough to tell, and a failure is latched until ~/clear is called: a
// failed thruster is commanded off, so nothing would show it recovering.

#include "sub_control/utils.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <format>
#include <optional>
#include <stdexcept>

using namespace std::chrono_literals;

class ThrusterMonitor : public rclcpp::Node {
   public:
    ThrusterMonitor() : Node("thruster_monitor") {
        rcl_interfaces::msg::ParameterDescriptor read_only;
        read_only.read_only = true;
        this->declare_parameter("allocation.thruster_reversed", rclcpp::ParameterType::PARAMETER_BOOL_ARRAY, read_only);
        // Commanded thrust [N] below which a thruster's response is too small to judge.
        this->declare_parameter("min_thrust", 3.0);
        // Averaging window [s] of the effectiveness estimate.
        this->declare_parameter("window", 1.0);
        // Lag [s] of the thrust behind the command (ESC plus rotor).
        this->declare_parameter("response_time", 0.15);
        // Effectiveness below which a thruster counts as failed, and for how long [s].
        this->declare_parameter("fail_below", 0.3);
        this->declare_parameter("fail_after", 1.0);
        // Depth [m] the thrusters need to be under: near the surface they lose thrust anyway.
        this->declare_parameter("min_depth", 0.3);

        // Not get_parameter(name), which throws if the launch file did not set it.
        rclcpp::Parameter reversed;
        if (!this->get_parameter("allocation.thruster_reversed", reversed) ||
            reversed.as_bool_array().size() != NUM_THRUSTERS) {
            throw std::invalid_argument(std::format(
                "allocation.thruster_reversed must be {} booleans (from the vehicle description)", NUM_THRUSTERS));
        }
        for (int i = 0; i < NUM_THRUSTERS; ++i) {
            reversed_[i] = reversed.as_bool_array()[i];
        }

        // A NaN or inf command stops the thruster (sub_low, sim_thrusters), so expect nothing from it.
        for (int i = 0; i < NUM_THRUSTERS; ++i) {
            command_subs_[i] = this->create_subscription<std_msgs::msg::Float64>(
                std::format("control/thruster_{}", i), 10, [this, i](const std_msgs::msg::Float64::SharedPtr msg) {
                    command_[i] = std::isfinite(msg->data) ? msg->data : 0.0;
                });
        }
        thrust_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "thrusters/measured_thrust", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { measurement_callback(*msg); });
        depth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/depth", rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { depth_ = -msg->pose.pose.position.z; });

        health_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("thruster_health", 1);
        clear_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/clear", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                              std_srvs::srv::Trigger::Response::SharedPtr response) {
                failed_.fill(false);
                below_.fill(0.0);
                RCLCPP_INFO(this->get_logger(), "cleared all thruster failures");
                publish();
                response->success = true;
            });
        timer_ = this->create_timer(1s, [this]() { publish(); });
    }

   private:
    void measurement_callback(const std_msgs::msg::Float64MultiArray& msg) {
        if (msg.data.size() != NUM_THRUSTERS) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "thrusters/measured_thrust needs %d values, got %zu", NUM_THRUSTERS, msg.data.size());
            return;
        }
        // One NaN would stay in that thruster's averages for good and hide any failure.
        if (!std::ranges::all_of(msg.data, [](double f) { return std::isfinite(f); })) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "ignoring thrusters/measured_thrust with NaN/inf in it");
            return;
        }
        const rclcpp::Time now = this->now();
        const double dt = last_ ? std::clamp((now - *last_).seconds(), 0.0, 0.5) : 0.0;
        last_ = now;
        if (dt <= 0.0) {
            return;
        }
        const double min_thrust = this->get_parameter("min_thrust").as_double();
        const double window = std::max(this->get_parameter("window").as_double(), 1e-3);
        const double response_time = std::max(this->get_parameter("response_time").as_double(), 0.0);
        const double fail_below = this->get_parameter("fail_below").as_double();
        const double fail_after = this->get_parameter("fail_after").as_double();
        const bool submerged = depth_ && *depth_ >= this->get_parameter("min_depth").as_double();

        const double forget = std::exp(-dt / window);
        const double follow = response_time > 0.0 ? 1.0 - std::exp(-dt / response_time) : 1.0;
        bool changed = false;
        for (int i = 0; i < NUM_THRUSTERS; ++i) {
            // What the thruster should push now, delayed as the real one is.
            expected_[i] += follow * (thruster_force(command_[i], reversed_[i]) - expected_[i]);
            const double measured = msg.data[i];
            cross_[i] = forget * cross_[i] + (1.0 - forget) * expected_[i] * measured;
            power_[i] = forget * power_[i] + (1.0 - forget) * expected_[i] * expected_[i];

            if (failed_[i] || !submerged || power_[i] < min_thrust * min_thrust) {
                below_[i] = 0.0;
                continue;
            }
            const double effectiveness = cross_[i] / power_[i];
            below_[i] = effectiveness < fail_below ? below_[i] + dt : 0.0;
            if (below_[i] >= fail_after) {
                failed_[i] = true;
                changed = true;
                RCLCPP_ERROR(this->get_logger(), "thruster %d failed: pushing %.0f%% of what it is commanded to", i,
                             100.0 * effectiveness);
            }
        }
        if (changed) {
            publish();
        }
    }

    void publish() {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(NUM_THRUSTERS);
        for (int i = 0; i < NUM_THRUSTERS; ++i) {
            msg.data[i] = failed_[i] ? 0.0 : 1.0;
        }
        health_pub_->publish(msg);
    }

    std::array<bool, NUM_THRUSTERS> reversed_{};
    std::array<double, NUM_THRUSTERS> command_{};
    std::array<double, NUM_THRUSTERS> expected_{};
    std::array<double, NUM_THRUSTERS> cross_{};
    std::array<double, NUM_THRUSTERS> power_{};
    std::array<double, NUM_THRUSTERS> below_{};
    std::array<bool, NUM_THRUSTERS> failed_{};
    std::optional<double> depth_;
    std::optional<rclcpp::Time> last_;

    std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, NUM_THRUSTERS> command_subs_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr depth_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr health_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterMonitor>());
    rclcpp::shutdown();
    return 0;
}
