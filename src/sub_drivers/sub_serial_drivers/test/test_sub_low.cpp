// sub_low against a pretend board on a pseudo-terminal: the protocol, stale
// commands and reconnecting after the link drops.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <future>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sub_control_interfaces/msg/thruster_command.hpp>
#include <sub_driver_interfaces/srv/launch_torpedo.hpp>
#include <sub_driver_interfaces/srv/set_grabber.hpp>

#include "fake_device.hpp"
#include "sub_serial_drivers/sub_low.hpp"

namespace maritime = sub_serial_drivers::maritime;
using namespace std::string_literals;
using sub_control_interfaces::msg::ThrusterCommand;
using sub_serial_drivers::SubLow;

using Commands = std::array<double, maritime::NUM_THRUSTERS>;
using Thrusts = std::array<float, maritime::NUM_THRUSTERS>;

constexpr std::uint8_t KILLED = 1 << 0;
constexpr std::uint8_t DEPTH_VALID = 1 << 1;

void put_u32(std::string& out, std::uint32_t value) {
    for (int i = 0; i < 4; ++i) {
        out += static_cast<char>(value >> (8 * i));
    }
}

// A STATE frame's record: what comes between its delimiters.
std::string state_record(std::uint8_t flags, float depth, std::uint32_t uptime_ms, std::uint32_t stops = 0,
                         std::uint32_t rx_errors = 0, std::uint8_t protocol = maritime::PROTOCOL) {
    std::string body = {static_cast<char>(maritime::STATE), static_cast<char>(protocol), static_cast<char>(flags)};
    for (const std::uint32_t field : {std::bit_cast<std::uint32_t>(depth), uptime_ms, stops, rx_errors}) {
        put_u32(body, field);
    }
    return maritime::cobs_encode(body);
}

// The same, as the board sends it.
std::string state_frame(std::uint8_t flags, float depth, std::uint32_t uptime_ms,
                        std::uint8_t protocol = maritime::PROTOCOL) {
    return '\0' + state_record(flags, depth, uptime_ms, 0, 0, protocol) + '\0';
}

// The record of a frame sub_low sends.
std::string record_of(const std::string& frame) { return frame.substr(1, frame.size() - 2); }

// The commands in a THRUSTERS frame's record, if it is one.
std::optional<Thrusts> thrusts(const std::string& record) {
    const auto body = maritime::cobs_decode(record);
    if (!body || body->size() != 1 + 4 * maritime::NUM_THRUSTERS ||
        static_cast<std::uint8_t>((*body)[0]) != maritime::THRUSTERS) {
        return std::nullopt;
    }
    Thrusts commands{};
    for (size_t i = 0; i < commands.size(); ++i) {
        std::uint32_t bits = 0;
        for (size_t b = 0; b < 4; ++b) {
            bits |= static_cast<std::uint32_t>(static_cast<std::uint8_t>((*body)[1 + 4 * i + b])) << (8 * b);
        }
        commands[i] = std::bit_cast<float>(bits);
    }
    return commands;
}

std::function<bool(const std::string&)> thrusting(double value) {
    return [value](const std::string& record) {
        const auto commands = thrusts(record);
        return commands && std::ranges::all_of(*commands, [value](float c) { return c == static_cast<float>(value); });
    };
}

TEST(Maritime, Cobs) {
    // Wikipedia's examples.
    std::string ascending;
    for (int byte = 1; byte <= 0xFE; ++byte) {
        ascending += static_cast<char>(byte);
    }
    const std::vector<std::pair<std::string, std::string>> examples = {
        {"\x00"s, "\x01\x01"s},
        {"\x00\x00"s, "\x01\x01\x01"s},
        {"\x00\x11\x00"s, "\x01\x02\x11\x01"s},
        {"\x11\x22\x00\x33"s, "\x03\x11\x22\x02\x33"s},
        {"\x11\x22\x33\x44"s, "\x05\x11\x22\x33\x44"s},
        {"\x11\x00\x00\x00"s, "\x02\x11\x01\x01\x01"s},
        {ascending, "\xFF"s + ascending},
        {"\x00"s + ascending, "\x01\xFF"s + ascending},
        {ascending + "\xFF"s, "\xFF"s + ascending + "\x02\xFF"s},
    };
    for (const auto& [data, encoded] : examples) {
        EXPECT_EQ(maritime::cobs_encode(data), encoded);
        EXPECT_EQ(maritime::cobs_decode(encoded), data);
    }
    // Ends inside a block, or has a zero.
    for (const std::string& garbage : {"\x03\x11"s, "\x00"s, "\x03\x11\x00"s}) {
        EXPECT_FALSE(maritime::cobs_decode(garbage).has_value());
    }
}

TEST(Maritime, Frames) {
    // TORPEDO, torpedo 1, open.
    EXPECT_EQ(maritime::frame(maritime::TORPEDO, "\x01\x01"s), "\x00\x04\x02\x01\x01\x00"s);
    // GRABBER, 1.0f (closed), then 0.0f (open).
    EXPECT_EQ(maritime::grabber_frame(false), "\x00\x02\x04\x01\x03\x80\x3f\x00"s);
    EXPECT_EQ(maritime::grabber_frame(true), "\x00\x02\x04\x01\x01\x01\x01\x00"s);

    const std::string frame = maritime::thruster_frame({0.5, -0.5, 0.0, 0.25, 0.0, 0.0, 0.0, -0.125});
    ASSERT_TRUE(frame.starts_with('\0') && frame.ends_with('\0'));
    std::string body = "\x01"s;
    for (const std::uint32_t bits : {0x3F000000u, 0xBF000000u, 0u, 0x3E800000u, 0u, 0u, 0u, 0xBE000000u}) {
        put_u32(body, bits);
    }
    EXPECT_EQ(maritime::cobs_decode(record_of(frame)), body);
}

TEST(Maritime, SanitizeStopsNonFiniteAndClamps) {
    EXPECT_EQ(maritime::sanitize(std::nan("")), 0.0);
    EXPECT_EQ(maritime::sanitize(INFINITY), 0.0);
    EXPECT_EQ(maritime::sanitize(2.0), maritime::MAX_COMMAND);
    EXPECT_EQ(maritime::sanitize(-2.0), -maritime::MAX_COMMAND);
    EXPECT_EQ(maritime::sanitize(0.25), 0.25);
}

TEST(Maritime, ParseState) {
    EXPECT_EQ(maritime::parse_state(state_record(KILLED | DEPTH_VALID, 0.5f, 123456, 3, 1)),
              (maritime::State{3, true, 0.5, 123456, 3, 1}));
    EXPECT_EQ(maritime::parse_state(state_record(0, 0.0f, 1000)),
              (maritime::State{3, false, std::nullopt, 1000, 0, 0}));
    // A depth that is not a number is no reading, but the rest stands.
    EXPECT_EQ(maritime::parse_state(state_record(DEPTH_VALID, NAN, 1000)),
              (maritime::State{3, false, std::nullopt, 1000, 0, 0}));
    // Another protocol's layout is unknown.
    EXPECT_EQ(maritime::parse_state(maritime::cobs_encode("\x81\x02\x05"s)),
              (maritime::State{2, false, std::nullopt, 0, 0, 0}));

    const std::string state = *maritime::cobs_decode(state_record(0, 0.0f, 1000));
    const std::string log = "[00:00:01.000,000] <err> main: " + std::string(100, 'x');
    for (const std::string& garbage :
         {maritime::cobs_encode("\x01"s + state.substr(1)), maritime::cobs_encode(state.substr(0, state.size() - 1)),
          maritime::cobs_encode(state + "\x00"s), maritime::cobs_encode("\x81"s), "\x05\x81"s, ""s, log}) {
        EXPECT_FALSE(maritime::parse_state(garbage).has_value());
    }
}

class SubLowTest : public ::testing::Test {
   protected:
    void start(const std::vector<rclcpp::Parameter>& overrides = {}) {
        rig = std::make_unique<Rig<SubLow>>(merged(
            {{"device", board.path()}, {"reconnect_period", 0.1}, {"command_timeout", 0.5}, {"data_timeout", 30.0}},
            overrides));
        publisher = rig->peer->create_publisher<ThrusterCommand>("control/thrusters", 10);
        kill_sub = rig->peer->create_subscription<std_msgs::msg::Bool>(
            "kill_switch", rclcpp::QoS(10).transient_local(),
            [this](const std_msgs::msg::Bool& msg) { kills.add(msg.data); });
        // Connecting stops the thrusters first.
        ASSERT_TRUE(board.wait_for(STOP));
        connected();
        ASSERT_TRUE(eventually([this]() { return publisher->get_subscription_count() > 0; }));
    }

    // The driver drops everything for SETTLE_TIME after connecting, then the first
    // record, as the end of one cut off.
    void connected() {
        std::this_thread::sleep_for(sub_serial_drivers::SETTLE_TIME + 50ms);
        board.send("\0"s);
        board.clear();
    }

    void command(const Commands& values, double age = 0.0) {
        ThrusterCommand msg;
        msg.header.stamp = rig->peer->now() - rclcpp::Duration::from_seconds(age);
        msg.command = values;
        publisher->publish(msg);
    }

    // The records the driver has sent since the last clear(), less the empty ones
    // between one frame's delimiter and the next's.
    std::vector<std::string> sent() const {
        std::vector<std::string> records;
        std::ranges::copy_if(board.log(), std::back_inserter(records), [](const std::string& r) { return !r.empty(); });
        return records;
    }

    void TearDown() override {
        kill_sub.reset();
        publisher.reset();
        rig.reset();
    }

    const std::string STOP = record_of(maritime::thruster_frame({}));
    FakeDevice board{'\0'};
    std::unique_ptr<Rig<SubLow>> rig;
    rclcpp::Publisher<ThrusterCommand>::SharedPtr publisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub;
    Inbox<bool> kills;
};

Commands all(double value) {
    Commands commands;
    commands.fill(value);
    return commands;
}

TEST_F(SubLowTest, CommandsAreOneSanitizedFrame) {
    ASSERT_NO_FATAL_FAILURE(start());
    command({0.2, -0.2, 0, 0, 0, 0, NAN, 9.0});
    ASSERT_TRUE(board.wait_for([](const std::string& record) { return thrusts(record).has_value(); }));
    const std::vector<std::string> records = sent();
    ASSERT_EQ(records.size(), 1u);
    EXPECT_EQ(thrusts(records[0]), (Thrusts{0.2f, -0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.6f}));
}

TEST_F(SubLowTest, StaleCommandsAreIgnored) {
    ASSERT_NO_FATAL_FAILURE(start());
    command(all(0.3), 2.0);
    board.read_for(300ms);
    EXPECT_TRUE(sent().empty());

    command(all(0.3));
    EXPECT_TRUE(board.wait_for(thrusting(0.3)));
}

TEST_F(SubLowTest, KillSwitchStatesArePublished) {
    ASSERT_NO_FATAL_FAILURE(start());
    board.send(state_frame(KILLED, 0.0f, 1000) + state_frame(0, 0.0f, 1200));
    EXPECT_TRUE(eventually([this]() { return kills.items() == std::vector<bool>{true, false}; }));
}

TEST_F(SubLowTest, ActuatorServices) {
    ASSERT_NO_FATAL_FAILURE(start());
    using sub_driver_interfaces::srv::LaunchTorpedo;
    const auto client = rig->peer->create_client<LaunchTorpedo>("launch_torpedo");
    ASSERT_TRUE(client->wait_for_service(3s));
    auto request = std::make_shared<LaunchTorpedo::Request>();
    request->torpedo_id = 1;
    request->open = true;
    auto response = client->async_send_request(request);
    ASSERT_EQ(response.wait_for(3s), std::future_status::ready);
    EXPECT_TRUE(response.get()->success);
    // TORPEDO, torpedo 1, open.
    EXPECT_TRUE(board.wait_for("\x04\x02\x01\x01"s));

    request->torpedo_id = 2;
    response = client->async_send_request(request);
    ASSERT_EQ(response.wait_for(3s), std::future_status::ready);
    EXPECT_FALSE(response.get()->success);

    using sub_driver_interfaces::srv::SetGrabber;
    const auto grabber = rig->peer->create_client<SetGrabber>("set_grabber");
    ASSERT_TRUE(grabber->wait_for_service(3s));
    auto grab = std::make_shared<SetGrabber::Request>();
    grab->open = false;
    auto grabbed = grabber->async_send_request(grab);
    ASSERT_EQ(grabbed.wait_for(3s), std::future_status::ready);
    EXPECT_TRUE(grabbed.get()->success);
    EXPECT_TRUE(board.wait_for(record_of(maritime::grabber_frame(false))));
}

TEST_F(SubLowTest, ReconnectsAfterTheLinkDrops) {
    ASSERT_NO_FATAL_FAILURE(start());
    board.unplug();
    board.plug();
    // Reopened, and the thrusters stopped first.
    ASSERT_TRUE(board.wait_for(STOP));
    connected();
    command(all(0.1));
    EXPECT_TRUE(board.wait_for(thrusting(0.1)));
}

TEST_F(SubLowTest, WhatTheBoardKeptWhileClosedIsDropped) {
    ASSERT_NO_FATAL_FAILURE(start());
    board.unplug();
    board.plug();
    ASSERT_TRUE(board.wait_for(STOP));
    // States from before the open, delivered as soon as it happens.
    board.send(state_frame(0, 0.1f, 1000) + state_frame(0, 0.1f, 1200));
    connected();
    board.send(state_frame(KILLED, 0.1f, 1400));
    ASSERT_TRUE(eventually([this]() { return kills.size() == 1; }));
    std::this_thread::sleep_for(100ms);
    EXPECT_EQ(kills.items(), std::vector<bool>{true});
}

TEST_F(SubLowTest, DiagnosticsCatchABoardOnAnotherProtocol) {
    using diagnostic_msgs::msg::DiagnosticArray;
    using diagnostic_msgs::msg::DiagnosticStatus;
    ASSERT_NO_FATAL_FAILURE(start());
    Inbox<DiagnosticStatus> statuses;
    const auto subscription =
        rig->peer->create_subscription<DiagnosticArray>("/diagnostics", 10, [&statuses](const DiagnosticArray& msg) {
            for (const DiagnosticStatus& status : msg.status) {
                if (status.name.ends_with(": board")) {
                    statuses.add(status);
                }
            }
        });
    // The board's status, once states like `state` keep coming.
    const auto board_status = [&](const std::string& state, std::uint8_t level, const std::string& message) {
        return eventually(
            [&]() {
                board.send(state);
                const auto items = statuses.items();
                return !items.empty() && items.back().level == level && items.back().message.starts_with(message);
            },
            5s);
    };
    EXPECT_TRUE(board_status(state_frame(0, 0.0f, 1000, 2), DiagnosticStatus::ERROR, "the board speaks protocol 2"));
    EXPECT_TRUE(board_status(state_frame(0, 0.0f, 2000), DiagnosticStatus::OK, "released"));
}

TEST_F(SubLowTest, ASilentBoardIsReopened) {
    ASSERT_NO_FATAL_FAILURE(start({{"data_timeout", 0.3}}));
    // The board says nothing: the port is reopened, stopping the thrusters again.
    EXPECT_TRUE(board.wait_for(STOP));
}
