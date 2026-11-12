#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sub_control_interfaces/msg/thruster_command.hpp>
#include <sub_driver_interfaces/srv/launch_torpedo.hpp>
#include <sub_driver_interfaces/srv/set_dropper.hpp>
#include <sub_driver_interfaces/srv/set_grabber.hpp>

#include "sub_serial_drivers/serial_node.hpp"
#include "sub_serial_drivers/sub_low_parameters.hpp"

namespace sub_serial_drivers {

// The maritime board's protocol 3 (https://sw.avbotz.com/maritime/protocol/ has the
// board's side). Every frame on the wire is
//
//   0x00, COBS(type, payload), 0x00
//
// with multi-byte fields little-endian and floats IEEE 754 single precision. The
// parts sub_low uses:
//
//   host -> board
//     THRUSTERS   f32 command[8]
//     TORPEDO     u8 id, u8 open (0, 1)
//     DROPPER     u8 id, u8 open (0, 1)
//     GRABBER     f32 closed (0 open, 1 closed)
//   board -> host, at 5 Hz
//     STATE       u8 protocol, u8 flags, f32 depth_m, u32 uptime_ms,
//                 u32 command_timeouts, u32 rx_errors
namespace maritime {

// The protocol the board must speak (PROTOCOL_VERSION in the firmware). Older
// firmware speaks text, so sends sub_low no frames at all.
constexpr int PROTOCOL = 3;
constexpr int NUM_THRUSTERS = 8;
// Hard ceiling on |normalized thruster command|: the board sends 1500 +/- 400 *
// command us to the Basic ESCs, so this keeps them within 1260-1740 us whatever is
// published. Matches MAX_POWER_LIMIT in sub_control and MAX_POWER in the firmware.
constexpr double MAX_COMMAND = 0.6;
// The board stops every thruster after this long without a thruster command [s]
// (COMMAND_TIMEOUT_MS in the firmware).
constexpr double COMMAND_WATCHDOG = 0.5;
// How often the board sends its state [Hz] (STATE_PERIOD_MS in the firmware).
constexpr double STATE_RATE = 5.0;
// Five missed states [s].
constexpr double STATE_TIMEOUT = 5.0 / STATE_RATE;

// Frame types (include/protocol.h in the firmware).
constexpr std::uint8_t THRUSTERS = 0x01;
constexpr std::uint8_t TORPEDO = 0x02;
constexpr std::uint8_t DROPPER = 0x03;
constexpr std::uint8_t GRABBER = 0x04;
constexpr std::uint8_t STATE = 0x81;

struct State {
    int protocol;
    bool killed;
    // [m, positive down]; nothing while the board cannot read its pressure sensor.
    std::optional<double> depth;
    std::uint32_t uptime_ms;
    // The times the board's watchdog stopped the thrusters because commands stopped.
    std::uint32_t stops;
    // The frames from the host the board dropped.
    std::uint32_t rx_errors;
    bool operator==(const State&) const = default;
};

// `data` with its zero bytes encoded away, so that a zero can end it.
std::string cobs_encode(std::string_view data);
// What cobs_encode() made `encoded` from, or nothing if it cannot have.
std::optional<std::string> cobs_decode(std::string_view encoded);
// The frame of `type`, delimiters and all.
std::string frame(std::uint8_t type, std::string_view payload);
// The STATE in `record` (what came between two delimiters), or nothing if it is
// another frame or garbled. Only `protocol` is set when it is not PROTOCOL: the
// rest of another protocol's layout is unknown.
std::optional<State> parse_state(std::string_view record);
// A thruster command the ESCs can take: NaN/inf stop, the rest within +/-MAX_COMMAND.
double sanitize(double command);
// The THRUSTERS frame that sets every thruster.
std::string thruster_frame(const std::array<double, NUM_THRUSTERS>& commands);
// The GRABBER frame that opens or closes the grabber all the way.
std::string grabber_frame(bool open);

}  // namespace maritime

// Bridge to the maritime board over USB serial: control/thrusters out, one
// THRUSTERS frame each; kill_switch and odometry/depth in; the launch_torpedo,
// set_dropper and set_grabber services.
//
// The thrusters are stopped whenever the port (re)opens and when this node
// exits, and a command older than `command_timeout` is not acted on. When the
// commands stop (sub_control, this node or the link died), the board's own
// watchdog stops the thrusters; nothing here fakes a kill when the link is lost.
// Its parameters are in sub_low_parameters.yaml.
class SubLow : public SerialNode {
   public:
    explicit SubLow(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SubLow() override;

   private:
    bool handle_record(std::string_view record) override;
    void on_connect() override;
    void handle_state(const maritime::State& state);
    void publish_depth(double depth);
    void thrusters_callback(const sub_control_interfaces::msg::ThrusterCommand& msg);
    // Opens or closes torpedo or dropper `id` (`type` TORPEDO or DROPPER): (success, message).
    std::pair<bool, std::string> actuate(std::uint8_t type, const char* name, int id, bool open);
    // Opens or closes the grabber: (success, message).
    std::pair<bool, std::string> grab(bool open);
    void diagnose_board(diagnostic_updater::DiagnosticStatusWrapper& status);

    const sub_low::Params params_;

    // What the board last said.
    std::optional<int> board_protocol_;
    std::optional<SteadyTime> last_state_;
    std::optional<std::uint32_t> board_uptime_ms_;
    // Counted since the board started, so the first state only sets them.
    std::optional<std::uint32_t> board_stops_;
    std::optional<std::uint32_t> board_rx_errors_;
    int board_restarts_{0};
    std::optional<bool> killed_;
    std::optional<double> depth_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kill_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr depth_pub_;
    rclcpp::Subscription<sub_control_interfaces::msg::ThrusterCommand>::SharedPtr thruster_sub_;
    rclcpp::Service<sub_driver_interfaces::srv::LaunchTorpedo>::SharedPtr launch_torpedo_srv_;
    rclcpp::Service<sub_driver_interfaces::srv::SetDropper>::SharedPtr set_dropper_srv_;
    rclcpp::Service<sub_driver_interfaces::srv::SetGrabber>::SharedPtr set_grabber_srv_;
    std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> kill_diagnostic_;
    std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> depth_diagnostic_;
    std::unique_ptr<diagnostic_updater::TopicDiagnostic> command_diagnostic_;
};

}  // namespace sub_serial_drivers
