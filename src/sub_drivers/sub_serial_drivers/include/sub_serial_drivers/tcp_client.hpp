#ifndef SUB_LOW_TCP_CLIENT_HPP_
#define SUB_LOW_TCP_CLIENT_HPP_

#include <functional>
#include <string>
#include <string_view>

// TCP client for boards exposed through USB CDC ECM. CDC ECM appears as a
// network interface on the host; this class connects to the board's IP/port and
// carries the newline-delimited command stream used by the low-level firmware.
class TcpClient {
   public:
    TcpClient(const std::string& host, int port, int connect_timeout_ms);
    ~TcpClient();

    TcpClient(const TcpClient&) = delete;
    TcpClient& operator=(const TcpClient&) = delete;

    bool write(const std::string& data);
    bool read_available(std::string& out);
    std::string description() const;
    void set_trace_callback(std::function<void(std::string_view direction, std::string_view data)> callback);

   private:
    std::string host_;
    int port_;
    int fd_{-1};
    std::function<void(std::string_view direction, std::string_view data)> trace_callback_;
};

#endif  // SUB_LOW_TCP_CLIENT_HPP_
