#include "sub_serial_drivers/sub_low.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <format>
#include <tuple>

#include <rclcpp_components/register_node_macro.hpp>

namespace sub_serial_drivers {

namespace maritime {

namespace {

constexpr size_t STATE_SIZE = 18;
constexpr std::uint8_t STATE_KILLED = 1 << 0;
constexpr std::uint8_t STATE_DEPTH_VALID = 1 << 1;

void put_u32(std::string& out, std::uint32_t value) {
    for (int i = 0; i < 4; ++i) {
        out += static_cast<char>(value >> (8 * i));
    }
}

std::uint32_t get_u32(std::string_view in) {
    std::uint32_t value = 0;
    for (int i = 0; i < 4; ++i) {
        value |= static_cast<std::uint32_t>(static_cast<std::uint8_t>(in[i])) << (8 * i);
    }
    return value;
}

}  // namespace

std::string cobs_encode(std::string_view data) {
    // Blocks of up to 254 bytes that are not zero, each after a code byte one more
    // than its length. A zero follows every block whose code is not 0xFF.
    std::string out(1, '\0');
    size_t code_at = 0;
    const auto finish = [&]() {
        out[code_at] = static_cast<char>(out.size() - code_at);
        code_at = out.size();
        out += '\0';
    };
    for (const char byte : data) {
        if (out.size() - code_at == 0xFF) {
            finish();
        }
        if (byte == '\0') {
            finish();
        } else {
            out += byte;
        }
    }
    out[code_at] = static_cast<char>(out.size() - code_at);
    return out;
}

std::optional<std::string> cobs_decode(std::string_view encoded) {
    std::string out;
    for (size_t i = 0; i < encoded.size();) {
        const size_t code = static_cast<std::uint8_t>(encoded[i]);
        if (code == 0 || i + code > encoded.size()) {
            return std::nullopt;
        }
        const std::string_view block = encoded.substr(i + 1, code - 1);
        if (block.find('\0') != std::string_view::npos) {
            return std::nullopt;
        }
        out += block;
        i += code;
        if (code != 0xFF && i < encoded.size()) {
            out += '\0';
        }
    }
    return out;
}

std::string frame(std::uint8_t type, std::string_view payload) {
    std::string body(1, static_cast<char>(type));
    body += payload;
    // The leading delimiter ends whatever came before: a frame cut off when the
    // port closed.
    return '\0' + cobs_encode(body) + '\0';
}

std::optional<State> parse_state(std::string_view record) {
    const auto body = cobs_decode(record);
    // The type, then at least the protocol.
    if (!body || body->size() < 2 || static_cast<std::uint8_t>((*body)[0]) != STATE) {
        return std::nullopt;
    }
    const std::string_view payload = std::string_view(*body).substr(1);
    State state{};
    state.protocol = static_cast<std::uint8_t>(payload[0]);
    if (state.protocol != PROTOCOL) {
        return state;
    }
    if (payload.size() != STATE_SIZE) {
        return std::nullopt;
    }
    const auto flags = static_cast<std::uint8_t>(payload[1]);
    const auto depth = std::bit_cast<float>(get_u32(payload.substr(2)));
    state.killed = (flags & STATE_KILLED) != 0;
    // A reading, and a number.
    if ((flags & STATE_DEPTH_VALID) != 0 && std::isfinite(depth)) {
        state.depth = depth;
    }
    state.uptime_ms = get_u32(payload.substr(6));
    state.stops = get_u32(payload.substr(10));
    state.rx_errors = get_u32(payload.substr(14));
    return state;
}

double sanitize(double command) {
    return std::isfinite(command) ? std::clamp(command, -MAX_COMMAND, MAX_COMMAND) : 0.0;
}

std::string thruster_frame(const std::array<double, NUM_THRUSTERS>& commands) {
    std::string payload;
    for (const double command : commands) {
        put_u32(payload, std::bit_cast<std::uint32_t>(static_cast<float>(command)));
    }
    return frame(THRUSTERS, payload);
}

std::string grabber_frame(bool open) {
    std::string payload;
    // How closed it is.
    put_u32(payload, std::bit_cast<std::uint32_t>(open ? 0.0f : 1.0f));
    return frame(GRABBER, payload);
}

}  // namespace maritime

namespace {

constexpr int NUM_TORPEDOES = 2;
constexpr int NUM_DROPPERS = 2;
// Stopping every thruster.
const std::string STOP = maritime::thruster_frame({});

}  // namespace

SubLow::SubLow(const rclcpp::NodeOptions& options)
    : SerialNode("sub_low", options, '\0'),
      // Read-only, so the listener is not needed after this.
      params_(sub_low::ParamListener(get_node_parameters_interface(), get_logger()).get_params()) {
    kill_pub_ = create_publisher<std_msgs::msg::Bool>("kill_switch", rclcpp::QoS(1).transient_local());
    depth_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry/depth", rclcpp::SensorDataQoS());
    thruster_sub_ = create_subscription<sub_control_interfaces::msg::ThrusterCommand>(
        "control/thrusters", rclcpp::QoS(1).best_effort(),
        [this](const sub_control_interfaces::msg::ThrusterCommand& msg) { thrusters_callback(msg); });
    launch_torpedo_srv_ = create_service<sub_driver_interfaces::srv::LaunchTorpedo>(
        "launch_torpedo", [this](const sub_driver_interfaces::srv::LaunchTorpedo::Request::SharedPtr request,
                                 sub_driver_interfaces::srv::LaunchTorpedo::Response::SharedPtr response) {
            std::tie(response->success, response->message) =
                actuate(maritime::TORPEDO, "torpedo", request->torpedo_id, request->open);
        });
    set_dropper_srv_ = create_service<sub_driver_interfaces::srv::SetDropper>(
        "set_dropper", [this](const sub_driver_interfaces::srv::SetDropper::Request::SharedPtr request,
                              sub_driver_interfaces::srv::SetDropper::Response::SharedPtr response) {
            std::tie(response->success, response->message) =
                actuate(maritime::DROPPER, "dropper", request->dropper_id, request->open);
        });
    set_grabber_srv_ = create_service<sub_driver_interfaces::srv::SetGrabber>(
        "set_grabber", [this](const sub_driver_interfaces::srv::SetGrabber::Request::SharedPtr request,
                              sub_driver_interfaces::srv::SetGrabber::Response::SharedPtr response) {
            std::tie(response->success, response->message) = grab(request->open);
        });

    diagnostics_->setHardwareID("maritime");
    diagnostics_->add("board", this, &SubLow::diagnose_board);
    kill_diagnostic_ = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>("kill_switch", *diagnostics_,
                                                                                       min_rate(maritime::STATE_RATE));
    depth_diagnostic_ = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>("odometry/depth", *diagnostics_,
                                                                                        min_rate(maritime::STATE_RATE));
    // Fewer than one per watchdog period, and the board has been stopping the
    // thrusters between them. Stamps from up to 1 s ahead (the library's default)
    // to command_timeout behind.
    command_diagnostic_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
        "control/thrusters", *diagnostics_, min_rate(1.0 / maritime::COMMAND_WATCHDOG),
        diagnostic_updater::TimeStampStatusParam(-1.0, params_.command_timeout), get_clock());
}

SubLow::~SubLow() {
    // Rather than leave the thrusters on their last command until the board's
    // watchdog notices.
    write(STOP);
}

void SubLow::on_connect() {
    // Start from stopped thrusters, whatever the board was last told, without
    // waiting for its watchdog. The frame's leading delimiter ends any frame the
    // board was cut off in.
    write(STOP);
}

bool SubLow::handle_record(std::string_view record) {
    const auto state = maritime::parse_state(record);
    if (!state) {
        return false;
    }
    last_state_ = std::chrono::steady_clock::now();
    handle_state(*state);
    return true;
}

void SubLow::handle_state(const maritime::State& state) {
    if (state.protocol != maritime::PROTOCOL) {
        if (state.protocol != board_protocol_) {
            RCLCPP_ERROR(get_logger(),
                         "the board speaks protocol %d, not %d: flash maritime, or the thrusters will not move",
                         state.protocol, maritime::PROTOCOL);
        }
        board_protocol_ = state.protocol;
        return;
    }
    board_protocol_ = state.protocol;
    if (board_uptime_ms_ && state.uptime_ms < *board_uptime_ms_) {
        ++board_restarts_;
        RCLCPP_ERROR(get_logger(), "the board restarted (uptime %u ms, was %u ms)", state.uptime_ms, *board_uptime_ms_);
    }
    if (board_stops_ && state.stops > *board_stops_) {
        RCLCPP_ERROR(get_logger(), "no thruster commands reached the board for %g s: it stopped the thrusters",
                     maritime::COMMAND_WATCHDOG);
    }
    if (board_rx_errors_ && state.rx_errors > *board_rx_errors_) {
        RCLCPP_ERROR(get_logger(), "the board dropped %u frames from sub_low", state.rx_errors - *board_rx_errors_);
    }
    board_uptime_ms_ = state.uptime_ms;
    board_stops_ = state.stops;
    board_rx_errors_ = state.rx_errors;

    killed_ = state.killed;
    std_msgs::msg::Bool msg;
    msg.data = state.killed;
    kill_pub_->publish(msg);
    kill_diagnostic_->tick();
    depth_ = state.depth;
    if (state.depth) {
        publish_depth(*state.depth);
        depth_diagnostic_->tick();
    }
}

void SubLow::publish_depth(double depth) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = params_.depth_frame_id;
    odom.child_frame_id = params_.depth_child_frame_id;
    // The board reports depth (positive down); ENU z is positive up.
    odom.pose.pose.position.z = -depth;
    odom.pose.pose.orientation.w = 1.0;
    odom.pose.covariance[14] = params_.depth_z_variance;
    depth_pub_->publish(odom);
}

void SubLow::thrusters_callback(const sub_control_interfaces::msg::ThrusterCommand& msg) {
    const rclcpp::Time now = this->now();
    const rclcpp::Time stamp(msg.header.stamp, now.get_clock_type());
    command_diagnostic_->tick(stamp);
    const double age = (now - stamp).seconds();
    if (age > params_.command_timeout) {
        // Delayed in the graph: acting on it would replay the past. The board's
        // watchdog stops the thrusters if nothing fresher comes.
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "ignoring a thruster command %.2f s old", age);
        return;
    }
    std::array<double, maritime::NUM_THRUSTERS> commands{};
    std::ranges::transform(msg.command, commands.begin(), maritime::sanitize);
    if (!std::ranges::all_of(msg.command, [](double c) { return std::isfinite(c); })) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                              "thruster commands are not all finite: stopping those thrusters");
    }
    write(maritime::thruster_frame(commands));
}

std::pair<bool, std::string> SubLow::actuate(std::uint8_t type, const char* name, int id, bool open) {
    if (!connected()) {
        return {false, "the board is not connected"};
    }
    if (id < 0 || id >= (type == maritime::TORPEDO ? NUM_TORPEDOES : NUM_DROPPERS)) {
        return {false, std::format("no {} {}", name, id)};
    }
    const std::string payload = {static_cast<char>(id), static_cast<char>(open)};
    if (!write(maritime::frame(type, payload))) {
        return {false, "serial write failed"};
    }
    return {true, std::format("{} {} {}", name, id, open ? "opened" : "closed")};
}

std::pair<bool, std::string> SubLow::grab(bool open) {
    if (!connected()) {
        return {false, "the board is not connected"};
    }
    if (!write(maritime::grabber_frame(open))) {
        return {false, "serial write failed"};
    }
    return {true, open ? "grabber opened" : "grabber closed"};
}

void SubLow::diagnose_board(diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.add("protocol", board_protocol_ ? std::to_string(*board_protocol_) : "unknown");
    status.add("last state", since(last_state_));
    status.add("uptime [ms]", board_uptime_ms_ ? std::to_string(*board_uptime_ms_) : "unknown");
    status.add("restarts seen", board_restarts_);
    status.add("watchdog stops", board_stops_ ? std::to_string(*board_stops_) : "unknown");
    status.add("frames dropped by the board", board_rx_errors_ ? std::to_string(*board_rx_errors_) : "unknown");
    status.add("depth [m]", depth_ ? std::format("{:.3f}", *depth_) : "unknown");
    // Whether the kill switch and depth keep coming is the topic statuses' to report.
    using diagnostic_msgs::msg::DiagnosticStatus;
    const bool state_recent = last_state_ && std::chrono::steady_clock::now() - *last_state_ <
                                                 std::chrono::duration<double>(maritime::STATE_TIMEOUT);
    if (!connected()) {
        status.summary(DiagnosticStatus::ERROR, "not connected");
    } else if (!state_recent) {
        status.summary(DiagnosticStatus::ERROR,
                       std::format("no state for {:g} s (firmware older than protocol {} sends text, not frames)",
                                   maritime::STATE_TIMEOUT, maritime::PROTOCOL));
    } else if (board_protocol_ != maritime::PROTOCOL) {
        status.summary(DiagnosticStatus::ERROR,
                       std::format("the board speaks protocol {}, not {}: the thrusters will not move",
                                   *board_protocol_, maritime::PROTOCOL));
    } else {
        status.summary(DiagnosticStatus::OK, *killed_ ? "killed" : "released");
    }
}

}  // namespace sub_serial_drivers

RCLCPP_COMPONENTS_REGISTER_NODE(sub_serial_drivers::SubLow)
