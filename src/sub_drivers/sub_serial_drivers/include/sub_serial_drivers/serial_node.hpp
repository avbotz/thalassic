#pragma once

#include <charconv>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sub_serial_drivers/serial_port.hpp"

namespace sub_serial_drivers {

using SteadyTime = std::chrono::steady_clock::time_point;

// A node that talks to one serial device sending newline-terminated text. It
// keeps the link up by itself: whenever the device goes away (unplugged,
// reset), stops taking writes, or sends nothing handle_line() accepts for
// `data_timeout`, the port is closed and tried again every `reconnect_period`,
// and on_connect() runs once it is back. Everything runs on the executor's
// thread and nothing blocks for long, so there are no threads or locks.
class SerialNode : public rclcpp::Node {
   protected:
    SerialNode(const std::string& name, const rclcpp::NodeOptions& options, const std::string& default_device);

    // One line from the device, trimmed. True if it shows the device working.
    virtual bool handle_line(std::string_view line) = 0;
    // The port has just been opened: tell the device whatever it needs to know.
    virtual void on_connect() {}

    // Sends `text` to the device. False if not connected, or if the device would
    // not take it, in which case the port is reopened.
    bool write(std::string_view text);
    bool connected() const { return port_ != nullptr; }

    // Declares parameter `name`, which must be a finite, positive number of seconds.
    std::chrono::duration<double> declare_seconds(const std::string& name, double default_value);

    // Its "serial link" status is already added.
    std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
    // Limits for a rate check in /diagnostics: at least `rate` [Hz], averaged over
    // the last few updates, and no upper limit.
    diagnostic_updater::FrequencyStatusParam min_rate(double rate);

   private:
    void poll();
    void connect();
    void disconnect(const std::string& reason);
    void diagnose(diagnostic_updater::DiagnosticStatusWrapper& status);

    std::string device_;
    int baud_;
    std::chrono::duration<double> reconnect_period_;
    std::chrono::duration<double> data_timeout_;

    std::unique_ptr<SerialPort> port_;
    std::string received_;
    // Dropping what arrives up to the next newline: the rest of a line cut off
    // when the port opened, or of one too long to be real.
    bool skip_line_{true};
    std::optional<SteadyTime> last_open_attempt_;
    SteadyTime connected_at_;
    std::optional<SteadyTime> last_data_;
    int disconnects_{0};
    rclcpp::TimerBase::SharedPtr poll_timer_;
    // What min_rate() hands out: the rate checks keep pointers to their limits.
    std::deque<double> min_rates_;
    double no_max_rate_{std::numeric_limits<double>::infinity()};
};

// "never", or how long ago `time` was, for /diagnostics.
std::string since(std::optional<SteadyTime> time);

// `text` without the spaces and line endings around it.
std::string_view trim(std::string_view text);

// `text` cut at each `delimiter`, empty fields included.
std::vector<std::string_view> split(std::string_view text, char delimiter);

// The number `text` (spaces around it allowed) is exactly, if it is one and finite.
template <class T>
std::optional<T> parse_number(std::string_view text) {
    text = trim(text);
    T value{};
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (text.empty() || error != std::errc() || end != text.data() + text.size()) {
        return std::nullopt;
    }
    if constexpr (std::is_floating_point_v<T>) {
        // from_chars reads "nan" and "inf", neither of which is a measurement.
        if (!std::isfinite(value)) {
            return std::nullopt;
        }
    }
    return value;
}

}  // namespace sub_serial_drivers
