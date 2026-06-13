#include "sub_serial_drivers/tcp_client.hpp"

#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace {

class AddrInfo {
   private:
    addrinfo* result_{nullptr};

   public:
    AddrInfo(const std::string& host, const std::string& service) {
        addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;

        const int rc = getaddrinfo(host.c_str(), service.c_str(), &hints, &result_);
        if (rc != 0) {
            throw std::runtime_error("getaddrinfo(" + host + ":" + service + "): " + gai_strerror(rc));
        }
    }

    ~AddrInfo() {
        if (result_ != nullptr) {
            freeaddrinfo(result_);
        }
    }

    addrinfo* get() const { return result_; }
};

void set_nonblocking(int fd) {
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        throw std::runtime_error("fcntl(F_GETFL): " + std::string(std::strerror(errno)));
    }
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
        throw std::runtime_error("fcntl(F_SETFL): " + std::string(std::strerror(errno)));
    }
}

bool wait_connected(int fd, int timeout_ms) {
    pollfd pfd{};
    pfd.fd = fd;
    pfd.events = POLLOUT;

    while (true) {
        const int rc = poll(&pfd, 1, timeout_ms);
        if (rc > 0) {
            int error = 0;
            socklen_t len = sizeof(error);
            if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &len) != 0) {
                return false;
            }
            if (error != 0) {
                errno = error;
                return false;
            }
            return true;
        }
        if (rc == 0) {
            errno = ETIMEDOUT;
            return false;
        }
        if (errno != EINTR) {
            return false;
        }
    }
}

}

TcpClient::TcpClient(const std::string& host, int port, int connect_timeout_ms) : host_(host), port_(port) {
    if (port <= 0 || port > 65535) {
        throw std::runtime_error("invalid TCP port: " + std::to_string(port));
    }
    if (connect_timeout_ms <= 0) {
        throw std::runtime_error("connect timeout must be positive");
    }

    AddrInfo addresses(host, std::to_string(port));
    std::string last_error = "no addresses returned";

    for (addrinfo* ai = addresses.get(); ai != nullptr; ai = ai->ai_next) {
        const int fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) {
            last_error = std::strerror(errno);
            continue;
        }

        try {
            set_nonblocking(fd);
        } catch (const std::exception& e) {
            last_error = e.what();
            ::close(fd);
            continue;
        }

        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        if (connect(fd, ai->ai_addr, ai->ai_addrlen) == 0 ||
            (errno == EINPROGRESS && wait_connected(fd, connect_timeout_ms))) {
            fd_ = fd;
            return;
        }

        last_error = std::strerror(errno);
        ::close(fd);
    }

    throw std::runtime_error("connect(" + host + ":" + std::to_string(port) + "): " + last_error);
}

TcpClient::~TcpClient() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

bool TcpClient::write(const std::string& data) {
    const char* p = data.data();
    size_t left = data.size();
    while (left > 0) {
        const ssize_t n = send(fd_, p, left, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        p += n;
        left -= static_cast<size_t>(n);
    }
    return true;
}

bool TcpClient::read_available(std::string& out) {
    char buf[512];
    while (true) {
        const ssize_t n = recv(fd_, buf, sizeof(buf), 0);
        if (n > 0) {
            out.append(buf, static_cast<size_t>(n));
        } else if (n == 0) {
            return false;
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return true;
            if (errno == EINTR) continue;
            return false;
        }
    }
}

std::string TcpClient::description() const { return host_ + ":" + std::to_string(port_) + " over TCP"; }
