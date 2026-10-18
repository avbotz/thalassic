#include "sub_serial_drivers/sub_low.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <tuple>

#include <rclcpp_components/register_node_macro.hpp>

namespace sub_serial_drivers {

namespace maritime {

std::optional<Message> parse_line(std::string_view line) {
    const std::vector<std::string_view> fields = split(trim(line), ' ');
    if (fields.size() == 2 && fields[0] == "x") {
        const auto killed = parse_number<int>(fields[1]);
        if (killed == 0 || killed == 1) {
            return Kill{*killed == 1};
        }
    } else if (fields.size() == 2 && fields[0] == "d") {
        if (const auto depth = parse_number<double>(fields[1])) {
            return Depth{*depth};
        }
    } else if (fields.size() == 4 && fields[0] == "h") {
        const auto protocol = parse_number<int>(fields[1]);
        const auto uptime_ms = parse_number<std::uint32_t>(fields[2]);
        const auto stops = parse_number<std::uint32_t>(fields[3]);
        if (protocol && uptime_ms && stops) {
            return Heartbeat{*protocol, *uptime_ms, *stops};
        }
    }
    return std::nullopt;
}

double sanitize(double command) {
    return std::isfinite(command) ? std::clamp(command, -MAX_COMMAND, MAX_COMMAND) : 0.0;
}

std::string thruster_line(const std::array<double, NUM_THRUSTERS>& commands) {
    std::string text = "m";
    for (const double command : commands) {
        text += std::format(" {:.3f}", command);
    }
    return text + "\n";
}

}  // namespace maritime

namespace {

constexpr int NUM_TORPEDOES = 2;
constexpr int NUM_DROPPERS = 2;
// Stopping every thruster.
constexpr std::string_view STOP = "a 0\n";

}  // namespace

SubLow::SubLow(const rclcpp::NodeOptions& options) : SerialNode("sub_low", options, "/dev/ttyACM0") {
    // A thruster command older than this when it arrives is not acted on [s].
    command_timeout_ = declare_seconds("command_timeout", 0.5);
    depth_frame_id_ = declare_parameter<std::string>("depth_frame_id", "odom");
    depth_child_frame_id_ = declare_parameter<std::string>("depth_child_frame_id", "base_link");
    depth_z_variance_ = declare_parameter("depth_z_variance", 0.01);

    kill_pub_ = create_publisher<std_msgs::msg::Bool>("kill_switch", rclcpp::QoS(1).transient_local());
    depth_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry/depth", rclcpp::SensorDataQoS());
    thruster_sub_ = create_subscription<sub_control_interfaces::msg::ThrusterCommand>(
        "control/thrusters", rclcpp::QoS(1).best_effort(),
        [this](const sub_control_interfaces::msg::ThrusterCommand& msg) { thrusters_callback(msg); });
    launch_torpedo_srv_ = create_service<sub_driver_interfaces::srv::LaunchTorpedo>(
        "launch_torpedo", [this](const sub_driver_interfaces::srv::LaunchTorpedo::Request::SharedPtr request,
                                 sub_driver_interfaces::srv::LaunchTorpedo::Response::SharedPtr response) {
            std::tie(response->success, response->message) =
                actuate('t', "torpedo", request->torpedo_id, request->open);
        });
    set_dropper_srv_ = create_service<sub_driver_interfaces::srv::SetDropper>(
        "set_dropper", [this](const sub_driver_interfaces::srv::SetDropper::Request::SharedPtr request,
                              sub_driver_interfaces::srv::SetDropper::Response::SharedPtr response) {
            std::tie(response->success, response->message) =
                actuate('d', "dropper", request->dropper_id, request->open);
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
        diagnostic_updater::TimeStampStatusParam(-1.0, command_timeout_.count()), get_clock());
}

SubLow::~SubLow() {
    // Rather than leave the thrusters on their last command until the board's
    // watchdog notices.
    write(STOP);
}

void SubLow::on_connect() {
    // Start from stopped thrusters, whatever the board was last told, without
    // waiting for its watchdog. The newline first ends any line the board was cut
    // off in.
    write(std::format("\n{}", STOP));
}

bool SubLow::handle_line(std::string_view line) {
    const auto message = maritime::parse_line(line);
    if (!message) {
        return false;
    }
    if (const auto* kill = std::get_if<maritime::Kill>(&*message)) {
        killed_ = kill->killed;
        std_msgs::msg::Bool msg;
        msg.data = kill->killed;
        kill_pub_->publish(msg);
        kill_diagnostic_->tick();
    } else if (const auto* depth = std::get_if<maritime::Depth>(&*message)) {
        depth_ = depth->metres;
        publish_depth(depth->metres);
        depth_diagnostic_->tick();
    } else {
        handle_heartbeat(std::get<maritime::Heartbeat>(*message));
        last_heartbeat_ = std::chrono::steady_clock::now();
    }
    return true;
}

void SubLow::handle_heartbeat(const maritime::Heartbeat& heartbeat) {
    if (board_uptime_ms_ && heartbeat.uptime_ms < *board_uptime_ms_) {
        ++board_restarts_;
        RCLCPP_ERROR(get_logger(), "the board restarted (uptime %u ms, was %u ms)", heartbeat.uptime_ms,
                     *board_uptime_ms_);
    }
    if (heartbeat.stops > board_stops_) {
        RCLCPP_ERROR(get_logger(), "no thruster commands reached the board for %g s: it stopped the thrusters",
                     maritime::COMMAND_WATCHDOG);
    }
    board_protocol_ = heartbeat.protocol;
    board_uptime_ms_ = heartbeat.uptime_ms;
    board_stops_ = heartbeat.stops;
}

void SubLow::publish_depth(double depth) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = depth_frame_id_;
    odom.child_frame_id = depth_child_frame_id_;
    // The board reports depth (positive down); ENU z is positive up.
    odom.pose.pose.position.z = -depth;
    odom.pose.pose.orientation.w = 1.0;
    odom.pose.covariance[14] = depth_z_variance_;
    depth_pub_->publish(odom);
}

void SubLow::thrusters_callback(const sub_control_interfaces::msg::ThrusterCommand& msg) {
    const rclcpp::Time now = this->now();
    const rclcpp::Time stamp(msg.header.stamp, now.get_clock_type());
    command_diagnostic_->tick(stamp);
    const double age = (now - stamp).seconds();
    if (age > command_timeout_.count()) {
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
    write(maritime::thruster_line(commands));
}

std::pair<bool, std::string> SubLow::actuate(char command, const char* name, int id, bool open) {
    if (!connected()) {
        return {false, "the board is not connected"};
    }
    if (id < 0 || id >= (command == 't' ? NUM_TORPEDOES : NUM_DROPPERS)) {
        return {false, std::format("no {} {}", name, id)};
    }
    if (!write(std::format("{} {} {}\n", command, id, open ? 1 : 0))) {
        return {false, "serial write failed"};
    }
    return {true, std::format("{} {} {}", name, id, open ? "opened" : "closed")};
}

void SubLow::diagnose_board(diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.add("protocol", board_protocol_ ? std::to_string(*board_protocol_) : "unknown");
    status.add("last heartbeat", since(last_heartbeat_));
    status.add("uptime [ms]", board_uptime_ms_ ? std::to_string(*board_uptime_ms_) : "unknown");
    status.add("restarts seen", board_restarts_);
    status.add("watchdog stops", board_stops_);
    status.add("depth [m]", depth_ ? std::format("{:.3f}", *depth_) : "unknown");
    // Whether the kill switch and depth keep coming is the topic statuses' to report.
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                   killed_ ? (*killed_ ? "killed" : "released") : "kill switch not reported yet");
}

}  // namespace sub_serial_drivers

RCLCPP_COMPONENTS_REGISTER_NODE(sub_serial_drivers::SubLow)
