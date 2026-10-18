// The NaviGuider driver against a pretend module on a pseudo-terminal.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>

#include "fake_device.hpp"
#include "sub_serial_drivers/naviguider_imu.hpp"

namespace naviguider = sub_serial_drivers::naviguider;
using sensor_msgs::msg::Imu;
using sub_serial_drivers::NaviGuiderImu;

TEST(NaviGuider, ParseLine) {
    // As the module sends it: ", " between fields, LF CR between lines.
    const auto gyro = naviguider::parse_line("\r246511934, 4, 0.0, 0.002131, 0.0, 3.000000");
    ASSERT_TRUE(gyro.has_value());
    EXPECT_EQ(gyro->sensor, naviguider::GYROSCOPE);
    EXPECT_EQ(gyro->values[1], 0.002131);
    EXPECT_EQ(gyro->accuracy, 3.0);
    const auto orientation = naviguider::parse_line("246511934, 15, 0.06, -0.1, -0.84, 0.52, 0.0");
    ASSERT_TRUE(orientation.has_value());
    EXPECT_EQ(orientation->values[3], 0.52);
    for (const char* garbage : {
             "246511934, gyroscope, 0.0, 0.0, 0.0, 3",  // verbose
             "246511934, 254, 7, 1, 0",                 // a meta event
             "246511934, 4, 0.0, 0.0, 3",               // a value lost on the link
             "246511934, 4, nan, 0.0, 0.0, 3",
             "junk",
         }) {
        EXPECT_FALSE(naviguider::parse_line(garbage).has_value()) << garbage;
    }
}

constexpr std::string_view GYRO = "1.0,4,0.1,0.2,0.3,3\n";
constexpr std::string_view ORIENTATION = "1.0,15,0.0,0.0,0.0,1.0,3\n";
constexpr std::string_view ACCEL = "1.0,1,0.0,0.0,9.8,3\n";

class NaviGuiderImuTest : public ::testing::Test {
   protected:
    void start(const std::vector<rclcpp::Parameter>& overrides = {}) {
        rig = std::make_unique<Rig<NaviGuiderImu>>(merged(
            {{"device", module.path()}, {"reconnect_period", 0.1}, {"max_sample_age", 0.2}, {"data_timeout", 30.0}},
            overrides));
        subscription =
            rig->peer->create_subscription<Imu>("imu/data", 10, [this](const Imu& msg) { received.add(msg); });
        // Connecting starts the sensors.
        ASSERT_TRUE(module.wait_for("s 4,50"));
        connected();
        ASSERT_TRUE(eventually([this]() { return subscription->get_publisher_count() > 0; }));
    }

    // The driver drops the first line after connecting, as the end of one cut off.
    void connected() {
        module.send("\n");
        module.clear();
    }

    void TearDown() override {
        subscription.reset();
        rig.reset();
    }

    FakeDevice module{'\r'};
    std::unique_ptr<Rig<NaviGuiderImu>> rig;
    rclcpp::Subscription<Imu>::SharedPtr subscription;
    Inbox<Imu> received;
};

TEST_F(NaviGuiderImuTest, PublishesEachGyroSampleOnceAccelAndOrientationHaveReported) {
    ASSERT_NO_FATAL_FAILURE(start());
    module.send(GYRO);
    std::this_thread::sleep_for(200ms);
    EXPECT_EQ(received.size(), 0u);
    // Two gyro samples, two messages.
    module.send(std::string(ACCEL) + std::string(ORIENTATION) + std::string(GYRO) + std::string(GYRO));
    ASSERT_TRUE(eventually([this]() { return received.size() == 2; }));
    const Imu msg = received.items()[0];
    EXPECT_DOUBLE_EQ(msg.angular_velocity.z, 0.3);
    EXPECT_DOUBLE_EQ(msg.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(msg.linear_acceleration.z, 9.8);
}

TEST_F(NaviGuiderImuTest, ASensorThatStopsStopsTheMessages) {
    ASSERT_NO_FATAL_FAILURE(start());
    module.send(std::string(ACCEL) + std::string(ORIENTATION) + std::string(GYRO));
    ASSERT_TRUE(eventually([this]() { return received.size() == 1; }));
    // The orientation goes quiet; the gyro and the accelerometer keep coming.
    std::this_thread::sleep_for(300ms);
    module.send(std::string(ACCEL) + std::string(GYRO));
    std::this_thread::sleep_for(200ms);
    EXPECT_EQ(received.size(), 1u);
    // The orientation is back.
    module.send(std::string(ORIENTATION) + std::string(ACCEL) + std::string(GYRO));
    EXPECT_TRUE(eventually([this]() { return received.size() == 2; }));
}

TEST_F(NaviGuiderImuTest, GarbageAndNonFiniteValuesAreIgnored) {
    ASSERT_NO_FATAL_FAILURE(start());
    module.send("junk\n1.0,15,nan,0.0,0.0,1.0,3\n" + std::string(ACCEL) + std::string(GYRO));
    std::this_thread::sleep_for(150ms);
    EXPECT_EQ(received.size(), 0u);
    // A gyro line short of its accuracy is not a gyro sample.
    module.send(std::string(ACCEL) + std::string(ORIENTATION) + "1.0,4,0.1,0.2,0.3\n");
    std::this_thread::sleep_for(100ms);
    EXPECT_EQ(received.size(), 0u);
    module.send(std::string(ACCEL) + std::string(ORIENTATION) + std::string(GYRO));
    EXPECT_TRUE(eventually([this]() { return received.size() == 1; }));
}

TEST_F(NaviGuiderImuTest, GyroAccuracySetsTheRateCovariance) {
    ASSERT_NO_FATAL_FAILURE(start());
    module.send(std::string(ACCEL) + std::string(ORIENTATION) + "1.0,4,0.1,0.2,0.3,3\n1.0,4,0.1,0.2,0.3,1\n");
    ASSERT_TRUE(eventually([this]() { return received.size() == 2; }));
    EXPECT_EQ(received.items()[0].angular_velocity_covariance, naviguider::GYRO_COVARIANCE);
    EXPECT_EQ(received.items()[1].angular_velocity_covariance, naviguider::GYRO_COVARIANCE_UNCALIBRATED);
}

TEST_F(NaviGuiderImuTest, ADisabledAccelerometerIsNotWaitedFor) {
    ASSERT_NO_FATAL_FAILURE(start({{"accel_rate", 0}}));
    module.send(std::string(ORIENTATION) + std::string(GYRO));
    ASSERT_TRUE(eventually([this]() { return received.size() == 1; }));
    EXPECT_EQ(received.items()[0].linear_acceleration_covariance[0], -1.0);
    EXPECT_GT(received.items()[0].orientation_covariance[0], 0.0);
}

TEST_F(NaviGuiderImuTest, ASilentModuleIsRestarted) {
    ASSERT_NO_FATAL_FAILURE(start({{"data_timeout", 0.3}}));
    // Nothing comes back: the port is reopened and the sensors started again.
    ASSERT_TRUE(module.wait_for("s 4,50"));
    connected();
    module.send(std::string(ACCEL) + std::string(ORIENTATION) + std::string(GYRO));
    EXPECT_TRUE(eventually([this]() { return received.size() == 1; }));
}
