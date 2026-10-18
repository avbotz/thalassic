#include "sub_serial_drivers/serial_port.hpp"

#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <format>
#include <stdexcept>
#include <system_error>

namespace sub_serial_drivers {
namespace {

speed_t to_speed(int baud) {
    switch (baud) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 921600:
            return B921600;
        default:
            throw std::invalid_argument(std::format("unsupported baud rate {}", baud));
    }
}

[[noreturn]] void fail(const char* what, const std::string& device) {
    throw std::system_error(errno, std::generic_category(), std::format("{} {}", what, device));
}

}  // namespace

SerialPort::SerialPort(const std::string& device, int baud) {
    const speed_t speed = to_speed(baud);

    // O_RDWR - opens file for both reading and writing
    // O_NOCTTY - don't treat device like a terminal
    // O_NONBLOCK - open in non-blocking mode
    // O_CLOEXEC - close device before exec, not too important here but good practice
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
        fail("cannot open", device);
    }

    try {
        // LOCK_EX - acquire an exclusive lock to the device
        // LOCK_NB - fail immediately if held by another process
        if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
            fail("another process has open", device);
        }

        termios tio{};
        if (::tcgetattr(fd_, &tio) != 0) {
            fail("not a serial port:", device);
        }

        // Disables:
        // Canonical mode (line by line input that requires enter to send)
        // Local echoing (character echoed back locally)
        // Special signal generation (e.g. Ctrl+C sending SIGINT)
        // IO character translations (e.g. \r to \n)
        ::cfmakeraw(&tio);

        // CLOCAL - local connection and ignores modem control lines
        // CREAD - enables the receiver
        tio.c_cflag |= CLOCAL | CREAD;

        // CRTSCTS - clear hardware flow control (RTS/CTS)
        // CSTOPB - Clears 2 stop bit flag and uses 1 stop bit
        tio.c_cflag &= ~(CRTSCTS | CSTOPB);

        // read will block until 1 byte is available in the buffer
        tio.c_cc[VMIN] = 1;
        // No timing component (either intercharacter or overall timeout)
        tio.c_cc[VTIME] = 0;
        // Useful resource on VMIN and VTIME - http://www.unixwiz.net/techtips/termios-vmin-vtime.html

        // Set input/output baud rate and set overall set config
        if (::cfsetispeed(&tio, speed) != 0 || ::cfsetospeed(&tio, speed) != 0 ||
            ::tcsetattr(fd_, TCSANOW, &tio) != 0) {
            fail("cannot configure", device);
        }
    } catch (...) {
        ::close(fd_);
        throw;
    }

    // Modem control lines not used.
    int lines = TIOCM_DTR | TIOCM_RTS;
    ::ioctl(fd_, TIOCMBIC, &lines);

    // An FTDI adapter holds what it receives for up to 16 ms (its latency timer)
    // before passing it on; in low-latency mode, 1 ms.
    serial_struct serial{};
    if (::ioctl(fd_, TIOCGSERIAL, &serial) == 0) {
        serial.flags |= ASYNC_LOW_LATENCY;
        low_latency_ = ::ioctl(fd_, TIOCSSERIAL, &serial) == 0;
    }
    // Nothing from before this open counts.
    ::tcflush(fd_, TCIOFLUSH);
}

SerialPort::~SerialPort() {
    // Unsent output first: otherwise closing waits up to 30 s for a device that
    // has stopped reading to take it.
    ::tcflush(fd_, TCOFLUSH);
    ::close(fd_);
}

bool SerialPort::read(std::string& buffer) {
    char chunk[512];
    while (true) {
        const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
        if (n > 0) {
            buffer.append(chunk, static_cast<size_t>(n));
            if (static_cast<size_t>(n) < sizeof(chunk)) {
                return true;
            }
        } else if (n < 0 && errno == EINTR) {
            continue;
        } else {
            // Nothing more for now, or 0 (hung up) or an error such as EIO (gone).
            return n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK);
        }
    }
}

bool SerialPort::write(std::string_view data, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!data.empty()) {
        const ssize_t n = ::write(fd_, data.data(), data.size());
        if (n > 0) {
            data.remove_prefix(static_cast<size_t>(n));
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            return false;
        }
        // The device is not taking it as fast: wait for room, up to the deadline.
        const auto left =
            std::chrono::ceil<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
        if (left <= 0) {
            return false;
        }
        pollfd waiting{fd_, POLLOUT, 0};
        if (::poll(&waiting, 1, static_cast<int>(left)) < 0 && errno != EINTR) {
            return false;
        }
    }
    return true;
}

}  // namespace sub_serial_drivers
