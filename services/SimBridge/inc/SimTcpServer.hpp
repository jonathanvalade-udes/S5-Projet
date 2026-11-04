#pragma once

#include <string>

namespace services {

class SimTcpServer {
public:
    explicit SimTcpServer(int port = 9000);
    ~SimTcpServer();

    bool start();
    void stop();

    bool poll_client();
    bool read_line(std::string& line);
    bool send_response(const std::string& response);
    void close_client();

    bool is_client_connected() const { return m_client_fd >= 0; }
    int get_client_fd() const { return m_client_fd; }

private:
    int m_port{9000};
    int m_server_fd{-1};
    int m_client_fd{-1};
    std::string m_rx_buffer;
};

} // namespace services
