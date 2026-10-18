#include "sub_serial_drivers/serial_node.hpp"

#include <format>
#include <stdexcept>

using namespace std::chrono_literals;

namespace sub_serial_drivers {
namespace {

// How often the port is read: the longest a line waits to be handled.
constexpr auto POLL_PERIOD = 5ms;
// The longest a write waits for a device that has stopped taking input.
constexpr auto WRITE_TIMEOUT = 50ms;
// Neither device sends lines anywhere near this long.
constexpr size_t MAX_LINE_LENGTH = 256;

}  // namespace

SerialNode::SerialNode(const std::string& name, const rclcpp::NodeOptions& options, const std::string& default_device)
    : Node(name, options),
      device_(declare_parameter("device", default_device)),
      baud_(declare_parameter("baud", 115200)),
      // How often to try the port again while it is gone [s].
      reconnect_period_(declare_seconds("reconnect_period", 1.0)),
      // How long the device may send nothing useful before the port is reopened [s].
      data_timeout_(declare_seconds("data_timeout", 1.0)) {
    diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
    diagnostics_->add("serial link", this, &SerialNode::diagnose);
    poll_timer_ = create_wall_timer(POLL_PERIOD, [this]() { poll(); });
}

std::chrono::duration<double> SerialNode::declare_seconds(const std::string& name, double default_value) {
    const double value = declare_parameter(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::format("{} must be a finite, positive number of seconds", name));
    }
    return std::chrono::duration<double>(value);
}

diagnostic_updater::FrequencyStatusParam SerialNode::min_rate(double rate) {
    return {&min_rates_.emplace_back(rate), &no_max_rate_};
}

bool SerialNode::write(std::string_view text) {
    if (!port_) {
        return false;
    }
    if (port_->write(text, WRITE_TIMEOUT)) {
        return true;
    }
    disconnect("the device is not taking writes");
    return false;
}

void SerialNode::poll() {
    const SteadyTime now = std::chrono::steady_clock::now();
    if (!port_) {
        if (!last_open_attempt_ || now - *last_open_attempt_ >= reconnect_period_) {
            connect();
        }
        return;
    }
    if (!port_->read(received_)) {
        disconnect("the device is gone");
        return;
    }
    // handle_line() may write, and a failed write disconnects: stop there.
    size_t start = 0;
    for (size_t end; port_ && (end = received_.find('\n', start)) != std::string::npos; start = end + 1) {
        if (skip_line_) {
            skip_line_ = false;
        } else if (handle_line(trim(std::string_view(received_).substr(start, end - start)))) {
            last_data_ = now;
        }
    }
    if (!port_) {
        return;
    }
    received_.erase(0, start);
    if (received_.size() > MAX_LINE_LENGTH) {
        received_.clear();
        skip_line_ = true;
    }
    if (now - last_data_.value_or(connected_at_) > data_timeout_) {
        disconnect(std::format("nothing from the device for {:g} s", data_timeout_.count()));
    }
}

void SerialNode::connect() {
    const SteadyTime now = std::chrono::steady_clock::now();
    last_open_attempt_ = now;
    try {
        port_ = std::make_unique<SerialPort>(device_, baud_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "%s; retrying", e.what());
        return;
    }
    received_.clear();
    skip_line_ = true;
    connected_at_ = now;
    last_data_.reset();
    RCLCPP_INFO(get_logger(), "connected to %s at %d baud%s", device_.c_str(), baud_,
                port_->low_latency() ? "" : " (no low-latency mode)");
    on_connect();
}

void SerialNode::disconnect(const std::string& reason) {
    if (!port_) {
        return;
    }
    port_.reset();
    ++disconnects_;
    RCLCPP_ERROR(get_logger(), "%s: %s lost, reconnecting", reason.c_str(), device_.c_str());
}

void SerialNode::diagnose(diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.add("device", device_);
    status.add("connected", connected() ? "true" : "false");
    status.add("low-latency mode", connected() && port_->low_latency() ? "true" : "false");
    status.add("disconnects", disconnects_);
    status.add("last data", since(last_data_));
    if (connected()) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "connected");
    } else {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "not connected");
    }
}

std::string since(std::optional<SteadyTime> time) {
    if (!time) {
        return "never";
    }
    return std::format("{:.2f} s ago", std::chrono::duration<double>(std::chrono::steady_clock::now() - *time).count());
}

std::string_view trim(std::string_view text) {
    const size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) {
        return {};
    }
    return text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
}

std::vector<std::string_view> split(std::string_view text, char delimiter) {
    std::vector<std::string_view> fields;
    size_t start = 0;
    for (size_t end; (end = text.find(delimiter, start)) != std::string_view::npos; start = end + 1) {
        fields.push_back(text.substr(start, end - start));
    }
    fields.push_back(text.substr(start));
    return fields;
}

}  // namespace sub_serial_drivers
