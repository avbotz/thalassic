#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sub_control_interfaces/msg/thruster_command.hpp>
#include <sub_driver_interfaces/srv/launch_torpedo.hpp>
#include <sub_driver_interfaces/srv/set_dropper.hpp>

#include "sub_serial_drivers/serial_node.hpp"

namespace sub_serial_drivers {

// The maritime board's protocol 2, newline-terminated ASCII (maritime's README has
// the board's side). The parts sub_low uses:
//
//   host -> board
//     m <c0> ... <c7>   all eight thruster commands
//     a <c>             every thruster
//     t <i> <0|1>       close/open torpedo i
//     d <i> <0|1>       close/open dropper i
//   board -> host, each at 5 Hz
//     x <0|1>           kill switch, 1 = killed
//     d <m>             depth [m, positive down]; none while the sensor cannot be read
//     h <protocol> <uptime_ms> <stops>
//                       heartbeat; stops counts the times the board's watchdog
//                       stopped the thrusters because commands stopped
namespace maritime {

constexpr int NUM_THRUSTERS = 8;
// Hard ceiling on |normalized thruster command|: the board sends 1500 +/- 400 *
// command us to the Basic ESCs, so this keeps them within 1260-1740 us whatever is
// published. Matches MAX_POWER_LIMIT in sub_control and MAX_POWER in the firmware.
constexpr double MAX_COMMAND = 0.6;
// The board stops every thruster after this long without a thruster command [s]
// (COMMAND_TIMEOUT_MS in the firmware).
constexpr double COMMAND_WATCHDOG = 0.5;
// How often the board sends its kill switch, depth and heartbeat lines [Hz]
// (STATE_PERIOD_MS in the firmware).
constexpr double STATE_RATE = 5.0;

struct Kill {
    bool killed;
    bool operator==(const Kill&) const = default;
};
struct Depth {
    double metres;
    bool operator==(const Depth&) const = default;
};
struct Heartbeat {
    int protocol;
    std::uint32_t uptime_ms;
    std::uint32_t stops;
    bool operator==(const Heartbeat&) const = default;
};
using Message = std::variant<Kill, Depth, Heartbeat>;

// One line from the board, or nothing if it is not one of those, or garbled.
std::optional<Message> parse_line(std::string_view line);
// A thruster command the ESCs can take: NaN/inf stop, the rest within +/-MAX_COMMAND.
double sanitize(double command);
// The `m` line that sets every thruster.
std::string thruster_line(const std::array<double, NUM_THRUSTERS>& commands);

}  // namespace maritime

// Bridge to the maritime board over USB serial: control/thrusters out, one `m`
// line each; kill_switch and odometry/depth in; the launch_torpedo and
// set_dropper services.
//
// The thrusters are stopped whenever the port (re)opens and when this node
// exits, and a command older than `command_timeout` is not acted on. When the
// commands stop (sub_control, this node or the link died), the board's own
// watchdog stops the thrusters; nothing here fakes a kill when the link is lost.
class SubLow : public SerialNode {
   public:
    explicit SubLow(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SubLow() override;

   private:
    bool handle_line(std::string_view line) override;
    void on_connect() override;
    void handle_heartbeat(const maritime::Heartbeat& heartbeat);
    void publish_depth(double depth);
    void thrusters_callback(const sub_control_interfaces::msg::ThrusterCommand& msg);
    // Opens or closes torpedo or dropper `id` with `command` ('t' or 'd'): (success, message).
    std::pair<bool, std::string> actuate(char command, const char* name, int id, bool open);
    void diagnose_board(diagnostic_updater::DiagnosticStatusWrapper& status);

    std::chrono::duration<double> command_timeout_;
    std::string depth_frame_id_;
    std::string depth_child_frame_id_;
    double depth_z_variance_;

    // What the board last said.
    std::optional<int> board_protocol_;
    std::optional<SteadyTime> last_heartbeat_;
    std::optional<std::uint32_t> board_uptime_ms_;
    std::uint32_t board_stops_{0};
    int board_restarts_{0};
    std::optional<bool> killed_;
    std::optional<double> depth_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kill_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr depth_pub_;
    rclcpp::Subscription<sub_control_interfaces::msg::ThrusterCommand>::SharedPtr thruster_sub_;
    rclcpp::Service<sub_driver_interfaces::srv::LaunchTorpedo>::SharedPtr launch_torpedo_srv_;
    rclcpp::Service<sub_driver_interfaces::srv::SetDropper>::SharedPtr set_dropper_srv_;
    std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> kill_diagnostic_;
    std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> depth_diagnostic_;
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> command_diagnostic_;
};

}  // namespace sub_serial_drivers
