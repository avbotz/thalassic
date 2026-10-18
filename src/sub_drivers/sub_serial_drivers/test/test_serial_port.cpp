#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <string>
#include <system_error>

#include "fake_device.hpp"
#include "sub_serial_drivers/serial_port.hpp"

using sub_serial_drivers::SerialPort;

TEST(SerialPort, ReadsWhatArrivesAndNoticesTheDeviceGoing) {
    FakeDevice device;
    SerialPort port(device.path(), 115200);
    std::string received;
    EXPECT_TRUE(port.read(received));
    EXPECT_EQ(received, "");
    device.send("x 1\n");
    EXPECT_TRUE(eventually([&]() { return port.read(received) && received == "x 1\n"; }));
    device.unplug();
    EXPECT_FALSE(port.read(received));
}

TEST(SerialPort, WriteGivesUpOnADeviceThatStopsReading) {
    FakeDevice device;
    SerialPort port(device.path(), 115200);
    const auto begin = std::chrono::steady_clock::now();
    EXPECT_FALSE(port.write(std::string(1 << 20, 'x'), 50ms));
    EXPECT_LT(std::chrono::steady_clock::now() - begin, 1s);
}

TEST(SerialPort, OneOpenerAtATime) {
    FakeDevice device;
    SerialPort port(device.path(), 115200);
    EXPECT_THROW(SerialPort(device.path(), 115200), std::system_error);
}

TEST(SerialPort, RejectsWhatItCannotOpen) {
    FakeDevice device;
    EXPECT_THROW(SerialPort(device.path(), 1234), std::invalid_argument);
    EXPECT_THROW(SerialPort("/dev/null", 115200), std::system_error);  // not a serial port
    EXPECT_THROW(SerialPort("/nonexistent", 115200), std::system_error);
}
