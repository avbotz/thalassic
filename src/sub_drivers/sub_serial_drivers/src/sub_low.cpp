#include "sub_serial_drivers/sub_low.hpp"

#include <cctype>
#include <cstdio>
#include <format>
#include <sstream>

using namespace std::chrono_literals;

static constexpr size_t MAX_RX_BUFFER = 4096;

namespace {

std::string escaped_payload(std::string_view payload) {
    std::string out;
    out.reserve(payload.size());

    for (const unsigned char ch : payload) {
        switch (ch) {
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                if (std::isprint(ch)) {
                    out.push_back(static_cast<char>(ch));
                } else {
                    out += std::format("\\x{:02x}", ch);
                }
                break;
        }
    }

    return out;
}

}

SubLow::SubLow(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("sub_low", options) {
    this->declare_parameter<std::string>("ecm_host", "192.168.7.2");
    this->declare_parameter<int>("ecm_port", 7777);
    this->declare_parameter<int>("ecm_connect_timeout_ms", 1000);
}

SubLow::~SubLow() { stop_thrusters(); }

SubLow::CallbackReturn SubLow::on_configure(const rclcpp_lifecycle::State&) {
    const std::string host = this->get_parameter("ecm_host").as_string();
    const int port = this->get_parameter("ecm_port").as_int();
    const int timeout_ms = this->get_parameter("ecm_connect_timeout_ms").as_int();

    try {
        board_ = std::make_unique<TcpClient>(host, port, timeout_ms);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "could not connect to USB CDC ECM board: %s", e.what());
        return CallbackReturn::FAILURE;
    }

    board_->set_trace_callback([this](std::string_view direction, std::string_view data) {
        const std::string payload = escaped_payload(data);
        RCLCPP_DEBUG(this->get_logger(), "tcp %.*s %zu bytes: %s", static_cast<int>(direction.size()),
                     direction.data(), data.size(), payload.c_str());
    });

    RCLCPP_INFO(this->get_logger(), "sub_low connected to USB CDC ECM board at %s", board_->description().c_str());

    kill_pub_ = this->create_publisher<std_msgs::msg::Bool>("kill_switch", rclcpp::QoS(1).transient_local());

    auto thruster_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        thruster_subs_[i] = this->create_subscription<std_msgs::msg::Float64>(
            std::format("control/thruster_{}", i), thruster_qos, [this, i](const std_msgs::msg::Float64& msg) {
                RCLCPP_DEBUG(this->get_logger(), "thruster request: index=%d power=%f active=%s", i, msg.data,
                             is_active_ ? "true" : "false");
                if (is_active_) {
                    set_thruster_power(i, msg.data);
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "thruster request ignored: index=%d inactive node", i);
                }
            });
    }

    launch_torpedo_srv_ = this->create_service<sub_driver_interfaces::srv::LaunchTorpedo>(
        "launch_torpedo", [this](const std::shared_ptr<sub_driver_interfaces::srv::LaunchTorpedo::Request> request,
                                 std::shared_ptr<sub_driver_interfaces::srv::LaunchTorpedo::Response> response) {
            this->launch_torpedo_callback(request, response);
        });

    set_dropper_srv_ = this->create_service<sub_driver_interfaces::srv::SetDropper>(
        "set_dropper", [this](const std::shared_ptr<sub_driver_interfaces::srv::SetDropper::Request> request,
                              std::shared_ptr<sub_driver_interfaces::srv::SetDropper::Response> response) {
            this->set_dropper_callback(request, response);
        });

    poll_timer_ = this->create_wall_timer(5ms, [this]() {
        if (is_active_) {
            poll_board();
        }
    });

    return CallbackReturn::SUCCESS;
}

SubLow::CallbackReturn SubLow::on_activate(const rclcpp_lifecycle::State& state) {
    LifecycleNode::on_activate(state);

    is_active_ = true;

    RCLCPP_INFO(this->get_logger(), "sub_low active");
    return CallbackReturn::SUCCESS;
}

SubLow::CallbackReturn SubLow::on_deactivate(const rclcpp_lifecycle::State& state) {
    LifecycleNode::on_deactivate(state);

    is_active_ = false;
    stop_thrusters();

    RCLCPP_INFO(this->get_logger(), "sub_low deactivated");
    return CallbackReturn::SUCCESS;
}

SubLow::CallbackReturn SubLow::on_cleanup(const rclcpp_lifecycle::State&) {
    poll_timer_.reset();
    kill_pub_.reset();
    launch_torpedo_srv_.reset();
    set_dropper_srv_.reset();

    for (auto& sub : thruster_subs_) {
        sub.reset();
    }

    board_.reset();
    rx_buffer_.clear();

    return CallbackReturn::SUCCESS;
}

SubLow::CallbackReturn SubLow::on_shutdown(const rclcpp_lifecycle::State&) {
    poll_timer_.reset();
    kill_pub_.reset();
    launch_torpedo_srv_.reset();
    set_dropper_srv_.reset();

    for (auto& sub : thruster_subs_) {
        sub.reset();
    }

    board_.reset();
    rx_buffer_.clear();

    return CallbackReturn::SUCCESS;
}

void SubLow::stop_thrusters() {
    if (!board_) {
        return;
    }
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        RCLCPP_DEBUG(this->get_logger(), "stop thruster request: index=%d power=0.000000", i);
        board_->write(std::format("t {} 0.0\n", i));
    }
}

void SubLow::launch_torpedo_callback(const std::shared_ptr<sub_driver_interfaces::srv::LaunchTorpedo::Request> request,
                                     std::shared_ptr<sub_driver_interfaces::srv::LaunchTorpedo::Response> response) {
    RCLCPP_DEBUG(this->get_logger(), "launch_torpedo request: torpedo_id=%u open=%s active=%s", request->torpedo_id,
                 request->open ? "true" : "false", is_active_ ? "true" : "false");

    if (!is_active_) {
        response->success = false;
        response->message = "sub_low is not active.";
        RCLCPP_DEBUG(this->get_logger(), "launch_torpedo response: success=false message=\"%s\"",
                     response->message.c_str());
        return;
    }

    if (request->torpedo_id >= NUM_TORPEDO_THRUSTERS) {
        response->success = false;
        response->message = std::format("Invalid torpedo id {}.", request->torpedo_id);
        RCLCPP_DEBUG(this->get_logger(), "launch_torpedo response: success=false message=\"%s\"",
                     response->message.c_str());
        return;
    }

    if (!board_->write(std::format("t {} {}\n", request->torpedo_id, request->open ? 1 : 0))) {
        response->success = false;
        response->message = "USB CDC ECM board write failed.";
        RCLCPP_DEBUG(this->get_logger(), "launch_torpedo response: success=false message=\"%s\"",
                     response->message.c_str());
        return;
    }

    response->success = true;
    response->message =
        std::format("Torpedo thruster {} {}.", request->torpedo_id, request->open ? "opened" : "closed");
    RCLCPP_DEBUG(this->get_logger(), "launch_torpedo response: success=true message=\"%s\"",
                 response->message.c_str());
}

void SubLow::set_dropper_callback(const std::shared_ptr<sub_driver_interfaces::srv::SetDropper::Request> request,
                                  std::shared_ptr<sub_driver_interfaces::srv::SetDropper::Response> response) {
    RCLCPP_DEBUG(this->get_logger(), "set_dropper request: open=%s active=%s", request->open ? "true" : "false",
                 is_active_ ? "true" : "false");

    if (!is_active_) {
        response->success = false;
        response->message = "sub_low is not active.";
        RCLCPP_DEBUG(this->get_logger(), "set_dropper response: success=false message=\"%s\"",
                     response->message.c_str());
        return;
    }

    if (!board_->write(std::format("d {}\n", request->open ? 1 : 0))) {
        response->success = false;
        response->message = "USB CDC ECM board write failed.";
        RCLCPP_DEBUG(this->get_logger(), "set_dropper response: success=false message=\"%s\"",
                     response->message.c_str());
        return;
    }

    response->success = true;
    response->message = std::format("Dropper {}.", request->open ? "opened" : "closed");
    RCLCPP_DEBUG(this->get_logger(), "set_dropper response: success=true message=\"%s\"", response->message.c_str());
}

void SubLow::set_thruster_power(int index, double normalized) {
    RCLCPP_DEBUG(this->get_logger(), "thruster command: index=%d power=%f", index, normalized);
    if (!board_->write(std::format("t {} {}\n", index, normalized))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "USB CDC ECM board write failed");
    }
}

void SubLow::poll_board() {
    if (!board_->read_available(rx_buffer_)) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "USB CDC ECM board read error / device disconnected");
        return;
    }

    size_t start_pos = 0;
    for (size_t i = 0; i < rx_buffer_.size(); ++i) {
        if (rx_buffer_[i] == '\n') {
            size_t len = i - start_pos;
            handle_line(std::string_view(rx_buffer_.data() + start_pos, len));
            start_pos = i + 1;
        }
    }

    if (rx_buffer_.size() > MAX_RX_BUFFER) {
        rx_buffer_.clear();
    }
}

void SubLow::handle_line(const std::string_view line) {
    // not optimal, can pass string_view directly to istringstream in C++23
    // TODO: find better way to avoid string copy
    std::istringstream ss{std::string{line}};

    char tag;
    if (!(ss >> tag)) {
        return;
    }

    if (tag == 'x') {
        int value;
        if (ss >> value) {
            std_msgs::msg::Bool msg;
            msg.data = value;
            RCLCPP_DEBUG(this->get_logger(), "kill_switch publish: data=%s", msg.data ? "true" : "false");
            kill_pub_->publish(msg);
        }
    } else if (tag == 'd') {
        float value;
        if (ss >> value) {
            RCLCPP_DEBUG(this->get_logger(), "depth feedback: value=%f", value);
        }
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto node = std::make_shared<SubLow>();

    exec.add_node(node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
