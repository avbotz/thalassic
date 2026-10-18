#pragma once

#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

// Waits up to `timeout` for `condition`; whether it came true.
inline bool eventually(const std::function<bool()>& condition, std::chrono::milliseconds timeout = 3s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!condition()) {
        if (std::chrono::steady_clock::now() > deadline) {
            return false;
        }
        std::this_thread::sleep_for(10ms);
    }
    return true;
}

inline std::function<bool(const std::string&)> starts_with(const std::string& prefix) {
    return [prefix](const std::string& text) { return text.starts_with(prefix); };
}

// The device end of a serial link: a pseudo-terminal behind a symlink, as
// /dev/maritime is. Unplugging closes it, so the driver sees the device go away;
// plugging it back in puts a new one behind the same path.
class FakeDevice {
   public:
    // `terminator` ends each command the driver sends.
    explicit FakeDevice(char terminator = '\n') : terminator_(terminator) {
        std::string pattern = (std::filesystem::temp_directory_path() / "fake_device_XXXXXX").string();
        if (::mkdtemp(pattern.data()) == nullptr) {
            throw std::runtime_error("mkdtemp failed");
        }
        directory_ = pattern;
        plug();
    }
    ~FakeDevice() {
        unplug();
        std::filesystem::remove_all(directory_);
    }
    FakeDevice(const FakeDevice&) = delete;
    FakeDevice& operator=(const FakeDevice&) = delete;

    std::string path() const { return (directory_ / "device").string(); }

    void plug() {
        int slave = -1;
        if (::openpty(&master_, &slave, nullptr, nullptr, nullptr) != 0) {
            throw std::runtime_error("openpty failed");
        }
        const std::filesystem::path target = ::ttyname(slave);
        ::close(slave);
        const auto staged = directory_ / "device.new";
        std::filesystem::remove(staged);
        std::filesystem::create_symlink(target, staged);
        std::filesystem::rename(staged, directory_ / "device");
        pending_.clear();
        log_.clear();
    }

    void unplug() {
        if (master_ >= 0) {
            ::close(master_);
            master_ = -1;
        }
    }

    void send(std::string_view text) {
        if (::write(master_, text.data(), text.size()) != static_cast<ssize_t>(text.size())) {
            throw std::runtime_error("write to the pseudo-terminal failed");
        }
    }

    // The commands the driver has sent since the last clear().
    const std::vector<std::string>& log() const { return log_; }
    void clear() { log_.clear(); }

    // Collects what the driver sends for `duration`.
    void read_for(std::chrono::milliseconds duration) {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            read_some(20ms);
        }
    }

    // Collects what the driver sends until a command since the last clear()
    // matches; whether one did within `timeout`.
    bool wait_for(const std::function<bool(const std::string&)>& match, std::chrono::milliseconds timeout = 3s) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!std::ranges::any_of(log_, match)) {
            if (std::chrono::steady_clock::now() > deadline) {
                return false;
            }
            read_some(20ms);
        }
        return true;
    }
    bool wait_for(const std::string& command, std::chrono::milliseconds timeout = 3s) {
        return wait_for([command](const std::string& c) { return c == command; }, timeout);
    }

   private:
    void read_some(std::chrono::milliseconds timeout) {
        pollfd ready{master_, POLLIN, 0};
        if (::poll(&ready, 1, static_cast<int>(timeout.count())) <= 0 || !(ready.revents & POLLIN)) {
            // Nothing, or the driver does not have the device open.
            std::this_thread::sleep_for(timeout);
            return;
        }
        char chunk[4096];
        const ssize_t n = ::read(master_, chunk, sizeof(chunk));
        if (n <= 0) {
            return;
        }
        pending_.append(chunk, static_cast<size_t>(n));
        for (size_t end; (end = pending_.find(terminator_)) != std::string::npos; pending_.erase(0, end + 1)) {
            log_.push_back(pending_.substr(0, end));
        }
    }

    std::filesystem::path directory_;
    char terminator_;
    int master_{-1};
    std::string pending_;
    std::vector<std::string> log_;
};

// What a subscription has received, safe to read while the executor runs.
template <class T>
class Inbox {
   public:
    void add(const T& item) {
        std::scoped_lock lock(mutex_);
        items_.push_back(item);
    }
    std::vector<T> items() const {
        std::scoped_lock lock(mutex_);
        return items_;
    }
    size_t size() const {
        std::scoped_lock lock(mutex_);
        return items_.size();
    }

   private:
    mutable std::mutex mutex_;
    std::vector<T> items_;
};

// `defaults`, with those `overrides` names replaced.
inline std::vector<rclcpp::Parameter> merged(std::vector<rclcpp::Parameter> defaults,
                                             const std::vector<rclcpp::Parameter>& overrides) {
    for (const rclcpp::Parameter& parameter : overrides) {
        const auto same = std::ranges::find(defaults, parameter.get_name(), &rclcpp::Parameter::get_name);
        if (same != defaults.end()) {
            *same = parameter;
        } else {
            defaults.push_back(parameter);
        }
    }
    return defaults;
}

// rclcpp initialized for as long as it lives.
struct RosContext {
    RosContext() { rclcpp::init(0, nullptr); }
    ~RosContext() { rclcpp::shutdown(); }
};

// The node under test spinning beside a peer node that talks to it, in a ROS
// context and namespace of their own.
template <class NodeT>
struct Rig {
    explicit Rig(const std::vector<rclcpp::Parameter>& parameters) {
        static int count = 0;
        const std::string ns = std::format("/test_{}_{}", ::getpid(), count++);
        node = std::make_shared<NodeT>(
            rclcpp::NodeOptions().parameter_overrides(parameters).arguments({"--ros-args", "-r", "__ns:=" + ns}));
        peer = std::make_shared<rclcpp::Node>("peer", ns);
        executor.add_node(node);
        executor.add_node(peer);
        spinner = std::thread([this]() { executor.spin(); });
    }
    ~Rig() {
        executor.cancel();
        spinner.join();
        executor.remove_node(peer);
        executor.remove_node(node);
    }
    Rig(const Rig&) = delete;
    Rig& operator=(const Rig&) = delete;

    // In this order: the context is up first and down last.
    RosContext context;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::shared_ptr<NodeT> node;
    rclcpp::Node::SharedPtr peer;
    std::thread spinner;
};
