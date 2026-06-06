#include "serial_drivers/sub_low.hpp"

#include <cstdio>
#include <format>

using namespace std::chrono_literals;

static constexpr size_t MAX_RX_BUFFER = 4096;

SubLow::SubLow(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("sub_low", options) {
    this->declare_parameter<std::string>("device", "/dev/ttyACM0");
    this->declare_parameter<int>("baud", 115200);
}

SubLow::~SubLow() { stop_thrusters(); }

SubLow::CallbackReturn SubLow::on_configure(const rclcpp_lifecycle::State&) {
    const std::string device = this->get_parameter("device").as_string();
    const int baud = this->get_parameter("baud").as_int();

    try {
        serial_ = std::make_unique<SerialPort>(device, baud);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "could not open serial: %s", e.what());
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(this->get_logger(), "sub_low connected to %s @ %d baud", device.c_str(), baud);

    kill_pub_ = this->create_publisher<std_msgs::msg::Bool>("kill_switch", rclcpp::QoS(1).transient_local());

    auto thruster_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        thruster_subs_[i] = this->create_subscription<std_msgs::msg::Float64>(
            std::format("control/thruster_{}", i), thruster_qos, [this, i](const std_msgs::msg::Float64& msg) {
                if (is_active_) {
                    set_thruster_power(i, msg.data);
                }
            });
    }

    poll_timer_ = this->create_wall_timer(5ms, [this]() {
        if (is_active_) {
            poll_serial();
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

    for (auto& sub : thruster_subs_) {
        sub.reset();
    }

    serial_.reset();
    rx_buffer_.clear();

    return CallbackReturn::SUCCESS;
}

SubLow::CallbackReturn SubLow::on_shutdown(const rclcpp_lifecycle::State&) {
    poll_timer_.reset();
    kill_pub_.reset();

    for (auto& sub : thruster_subs_) {
        sub.reset();
    }

    serial_.reset();
    rx_buffer_.clear();

    return CallbackReturn::SUCCESS;
}

void SubLow::stop_thrusters() {
    if (!serial_) {
        return;
    }
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        serial_->write(std::format("t {} 0.0\n", i));
    }
}

void SubLow::set_thruster_power(int index, double normalized) {
    if (!serial_->write(std::format("t {} {}\n", index, normalized))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "serial write failed");
    }
}

void SubLow::poll_serial() {
    if (!serial_->read_available(rx_buffer_)) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "serial read error / device disconnected");
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
            kill_pub_->publish(msg);
        }
    } else if (tag == 'd') {
        float value;
        if (ss >> value) {
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
