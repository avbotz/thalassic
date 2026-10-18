#pragma once

#include <chrono>
#include <string>
#include <string_view>

namespace sub_serial_drivers {

// A serial device in raw 8N1 mode.
class SerialPort {
   public:
    // Opens `device` and locks it against other processes.
    // Throws std::system_error, or std::invalid_argument if unable to configure device properly.
    SerialPort(const std::string& device, int baud);
    ~SerialPort();
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Appends whatever has arrived to `buffer`. False once the device is gone
    // (unplugged, reset).
    bool read(std::string& buffer);
    // Writes all of `data`, waiting up to `timeout` for the device to take it.
    // False if it could not.
    bool write(std::string_view data, std::chrono::milliseconds timeout);
    // Whether the USB adapter passes on what it receives at once (an FTDI within
    // 1 ms instead of 16).
    bool low_latency() const { return low_latency_; }

   private:
    int fd_{-1};
    bool low_latency_{false};
};

}  // namespace sub_serial_drivers
