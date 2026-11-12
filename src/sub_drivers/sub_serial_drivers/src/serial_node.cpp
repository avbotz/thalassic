#include "sub_serial_drivers/serial_node.hpp"

#include <format>

using namespace std::chrono_literals;

namespace sub_serial_drivers {
namespace {

// How often the port is read: the longest a record waits to be handled.
constexpr auto POLL_PERIOD = 5ms;
// The longest a write waits for a device that has stopped taking input.
constexpr auto WRITE_TIMEOUT = 50ms;
// Neither device sends records anywhere near this long.
constexpr size_t MAX_RECORD_LENGTH = 256;

}  // namespace

SerialNode::SerialNode(const std::string& name, const rclcpp::NodeOptions& options, char delimiter)
    : Node(name, options),
      // Read-only, so the listener is not needed after this.
      serial_params_(serial_node::ParamListener(get_node_parameters_interface(), get_logger()).get_params()),
      reconnect_period_(serial_params_.reconnect_period),
      data_timeout_(serial_params_.data_timeout),
      delimiter_(delimiter) {
    diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
    diagnostics_->add("serial link", this, &SerialNode::diagnose);
    poll_timer_ = create_wall_timer(POLL_PERIOD, [this]() { poll(); });
}

diagnostic_updater::FrequencyStatusParam SerialNode::min_rate(double rate) {
    return {&min_rates_.emplace_back(rate), &no_max_rate_};
}

bool SerialNode::write(std::string_view data) {
    if (!port_) {
        return false;
    }
    if (port_->write(data, WRITE_TIMEOUT)) {
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
    if (now - connected_at_ < SETTLE_TIME) {
        // What the device kept from before the open: old, but it would look current.
        received_.clear();
        return;
    }
    // handle_record() may write, and a failed write disconnects: stop there.
    size_t start = 0;
    for (size_t end; port_ && (end = received_.find(delimiter_, start)) != std::string::npos; start = end + 1) {
        if (skip_record_) {
            skip_record_ = false;
        } else if (handle_record(std::string_view(received_).substr(start, end - start))) {
            last_data_ = now;
        }
    }
    if (!port_) {
        return;
    }
    received_.erase(0, start);
    if (received_.size() > MAX_RECORD_LENGTH) {
        received_.clear();
        skip_record_ = true;
    }
    if (now - last_data_.value_or(connected_at_) > data_timeout_) {
        disconnect(std::format("nothing from the device for {:g} s", data_timeout_.count()));
    }
}

void SerialNode::connect() {
    const SteadyTime now = std::chrono::steady_clock::now();
    last_open_attempt_ = now;
    try {
        port_ = std::make_unique<SerialPort>(serial_params_.device, static_cast<int>(serial_params_.baud));
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "%s; retrying", e.what());
        return;
    }
    received_.clear();
    skip_record_ = true;
    connected_at_ = now;
    last_data_.reset();
    const std::optional<int> latency_timer = port_->latency_timer();
    const std::string latency = latency_timer ? std::format(", latency timer {} ms", *latency_timer) : "";
    RCLCPP_INFO(get_logger(), "connected to %s at %ld baud%s", serial_params_.device.c_str(), serial_params_.baud,
                latency.c_str());
    on_connect();
}

void SerialNode::disconnect(const std::string& reason) {
    if (!port_) {
        return;
    }
    port_.reset();
    ++disconnects_;
    RCLCPP_ERROR(get_logger(), "%s: %s lost, reconnecting", reason.c_str(), serial_params_.device.c_str());
}

void SerialNode::diagnose(diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.add("device", serial_params_.device);
    status.add("connected", connected() ? "true" : "false");
    // 1 ms is low-latency mode; "none" for a board on CDC-ACM, which has no such timer.
    const std::optional<int> latency_timer = connected() ? port_->latency_timer() : std::nullopt;
    status.add("latency timer [ms]", latency_timer ? std::to_string(*latency_timer) : "none");
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
