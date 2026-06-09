#include "sub_serial_drivers/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <format>

static speed_t to_speed(int baud) {
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
            return B0;
    }
}

SerialPort::SerialPort(const std::string& device, int baud) {
    const speed_t speed = to_speed(baud);
    if (speed == B0) {
        throw std::runtime_error(std::format("unsupported baud rate: {}", baud));
    }

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        throw std::runtime_error(std::format("open({}): {}", device, std::strerror(errno)));
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        close(fd_);
        throw std::runtime_error(std::format("tcgetattr: {}", std::strerror(errno)));
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= (CLOCAL | CREAD);  // ignore modem control lines, enable receiver
    tty.c_cflag &= ~CRTSCTS;          // no hardware flow control
    tty.c_cc[VMIN] = 0;               // read() returns immediately with whatever is there
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_);
        throw std::runtime_error(std::format("tcsetattr: {}", std::strerror(errno)));
    }
    tcflush(fd_, TCIOFLUSH);
}

SerialPort::~SerialPort() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

SerialPort::SerialPort(SerialPort&& o) noexcept : fd_(o.fd_) { o.fd_ = -1; }

SerialPort& SerialPort::operator=(SerialPort&& o) noexcept {
    if (this != &o) {
        if (fd_ >= 0) ::close(fd_);
        fd_ = o.fd_;
        o.fd_ = -1;
    }
    return *this;
}

bool SerialPort::write(const std::string& data) {
    const char* p = data.data();
    size_t left = data.size();
    while (left > 0) {
        const ssize_t n = ::write(fd_, p, left);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                continue;
            }
            return false;
        }
        p += n;
        left -= static_cast<size_t>(n);
    }
    return true;
}

bool SerialPort::read_available(std::string& out) {
    char buf[256];
    while (true) {
        const ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n > 0) {
            out.append(buf, static_cast<size_t>(n));
        } else if (n == 0) {
            return false;  // EOF: the device went away
        } else {
            // nothing buffered
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return true;
            }

            // syscall interrupted, try again
            if (errno == EINTR) {
                continue;
            }

            return false;
        }
    }
}
