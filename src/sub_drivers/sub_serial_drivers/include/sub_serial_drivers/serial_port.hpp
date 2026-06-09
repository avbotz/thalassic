#ifndef SUB_SERIAL_DRIVERS_SERIAL_PORT_HPP_
#define SUB_SERIAL_DRIVERS_SERIAL_PORT_HPP_

#include <string>

// Minimal RAII wrapper around a POSIX serial (tty) device in raw, non-blocking
// mode. Reads never block (they return whatever is currently buffered); writes
// are best-effort.
class SerialPort {
   public:
    // Opens `device` at the specified `baud` rate. Throws std::runtime_error if the device cannot be opened or configured.
    SerialPort(const std::string& device, int baud); 
    ~SerialPort();

    // Destructor defined so rule of 5 applies:
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    SerialPort(SerialPort&& o) noexcept;
    SerialPort& operator=(SerialPort&& o) noexcept;

    // Writes the bytes verbatim. Returns false on a write error.
    bool write(const std::string& data);

    // Appends any bytes currently available to `out`. Returns true when idle or
    // after reading (nothing more to read); false on a read error / disconnect.
    bool read_available(std::string& out);

   private:
    int fd_{-1};
};

#endif  // SUB_SERIAL_DRIVERS_SERIAL_PORT_HPP_
