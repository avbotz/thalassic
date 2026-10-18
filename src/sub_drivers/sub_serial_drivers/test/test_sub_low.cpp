// sub_low against a pretend board on a pseudo-terminal: the protocol, stale
// commands and reconnecting after the link drops.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <std_msgs/msg/bool.hpp>
#include <sub_control_interfaces/msg/thruster_command.hpp>
#include <sub_driver_interfaces/srv/launch_torpedo.hpp>

#include "fake_device.hpp"
#include "sub_serial_drivers/sub_low.hpp"

namespace maritime = sub_serial_drivers::maritime;
using sub_control_interfaces::msg::ThrusterCommand;
using sub_serial_drivers::SubLow;

using Commands = std::array<double, maritime::NUM_THRUSTERS>;

TEST(Maritime, ThrusterLine) {
    const Commands commands = {0.1, -0.2, 0.0, 0.6, -0.6, 0.0125, 0.0, 0.333};
    EXPECT_EQ(maritime::thruster_line(commands), "m 0.100 -0.200 0.000 0.600 -0.600 0.013 0.000 0.333\n");
}

TEST(Maritime, SanitizeStopsNonFiniteAndClamps) {
    EXPECT_EQ(maritime::sanitize(std::nan("")), 0.0);
    EXPECT_EQ(maritime::sanitize(INFINITY), 0.0);
    EXPECT_EQ(maritime::sanitize(2.0), maritime::MAX_COMMAND);
    EXPECT_EQ(maritime::sanitize(-2.0), -maritime::MAX_COMMAND);
    EXPECT_EQ(maritime::sanitize(0.25), 0.25);
}

TEST(Maritime, ParseLine) {
    EXPECT_EQ(maritime::parse_line("x 1\n"), maritime::Message(maritime::Kill{true}));
    EXPECT_EQ(maritime::parse_line("x 0\r\n"), maritime::Message(maritime::Kill{false}));
    EXPECT_EQ(maritime::parse_line("d 0.532"), maritime::Message(maritime::Depth{0.532}));
    EXPECT_EQ(maritime::parse_line("d -0.010"), maritime::Message(maritime::Depth{-0.01}));
    EXPECT_EQ(maritime::parse_line("h 2 123456 3"), maritime::Message(maritime::Heartbeat{2, 123456, 3}));
    for (const char* garbage : {"d nan", "d 0.5[00:00:01.234,000] <err> pressure", "d", "x", "x 2", "h 2 12",
                                "i 0.1 0.2 0.3", "", "[00:00:01.000,000] <err> main: something"}) {
        EXPECT_FALSE(maritime::parse_line(garbage).has_value()) << garbage;
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
        ASSERT_TRUE(board.wait_for("a 0"));
        connected();
        ASSERT_TRUE(eventually([this]() { return publisher->get_subscription_count() > 0; }));
    }

    // The driver drops the first line after connecting, as the end of one cut off.
    void connected() {
        board.send("\n");
        board.clear();
    }

    void command(const Commands& values, double age = 0.0) {
        ThrusterCommand msg;
        msg.header.stamp = rig->peer->now() - rclcpp::Duration::from_seconds(age);
        msg.command = values;
        publisher->publish(msg);
    }

    void TearDown() override {
        kill_sub.reset();
        publisher.reset();
        rig.reset();
    }

    FakeDevice board;
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

TEST_F(SubLowTest, CommandsAreOneSanitizedLine) {
    ASSERT_NO_FATAL_FAILURE(start());
    command({0.2, -0.2, 0, 0, 0, 0, NAN, 9.0});
    ASSERT_TRUE(board.wait_for(starts_with("m ")));
    EXPECT_EQ(board.log(), std::vector<std::string>{"m 0.200 -0.200 0.000 0.000 0.000 0.000 0.000 0.600"});
}

TEST_F(SubLowTest, StaleCommandsAreIgnored) {
    ASSERT_NO_FATAL_FAILURE(start());
    command(all(0.3), 2.0);
    board.read_for(300ms);
    EXPECT_TRUE(board.log().empty());

    command(all(0.3));
    EXPECT_TRUE(board.wait_for(starts_with("m 0.300 ")));
}

TEST_F(SubLowTest, KillSwitchLinesArePublished) {
    ASSERT_NO_FATAL_FAILURE(start());
    board.send("x 1\nx 0\n");
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
    EXPECT_TRUE(board.wait_for("t 1 1"));

    request->torpedo_id = 2;
    response = client->async_send_request(request);
    ASSERT_EQ(response.wait_for(3s), std::future_status::ready);
    EXPECT_FALSE(response.get()->success);
}

TEST_F(SubLowTest, ReconnectsAfterTheLinkDrops) {
    ASSERT_NO_FATAL_FAILURE(start());
    board.unplug();
    board.plug();
    // Reopened, and the thrusters stopped first.
    ASSERT_TRUE(board.wait_for("a 0"));
    connected();
    command(all(0.1));
    EXPECT_TRUE(board.wait_for(starts_with("m 0.100 ")));
}

TEST_F(SubLowTest, ASilentBoardIsReopened) {
    ASSERT_NO_FATAL_FAILURE(start({{"data_timeout", 0.3}}));
    // The board says nothing: the port is reopened, stopping the thrusters again.
    EXPECT_TRUE(board.wait_for("a 0"));
}
