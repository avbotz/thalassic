#pragma once

#include <chrono>
#include <optional>
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

    // Appends whatever has arrived to `buffer`. False once the device is gone (unplugged, reset).
    bool read(std::string& buffer);
    // Writes all of `data`, waiting up to `timeout` for the device to take it. False if it could not.
    bool write(std::string_view data, std::chrono::milliseconds timeout);
    // How long the USB adapter holds what it receives before passing it on [ms], as its driver reports
    // it: 1 for an FTDI in low-latency mode, 16 otherwise. Nothing if it has no such timer (a CDC-ACM board).
    std::optional<int> latency_timer() const { return latency_timer_; }

   private:
    int fd_{-1};
    std::optional<int> latency_timer_;
};

}  // namespace sub_serial_drivers
