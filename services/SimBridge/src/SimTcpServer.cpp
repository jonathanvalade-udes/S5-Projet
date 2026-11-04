#include "SimTcpServer.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace services {

SimTcpServer::SimTcpServer(int port) : m_port(port) {}

SimTcpServer::~SimTcpServer() {
    stop();
}

bool SimTcpServer::start() {
    m_server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (m_server_fd < 0) {
        std::cerr << "[SimTcpServer] Socket creation failed: " << strerror(errno) << std::endl;
        return false;
    }

    int opt = 1;
    setsockopt(m_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#ifdef SO_REUSEPORT
    setsockopt(m_server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(m_port);

    if (bind(m_server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[SimTcpServer] Bind failed on port " << m_port << ": " << strerror(errno) << std::endl;
        close(m_server_fd);
        m_server_fd = -1;
        return false;
    }

    if (listen(m_server_fd, 1) < 0) {
        std::cerr << "[SimTcpServer] Listen failed: " << strerror(errno) << std::endl;
        close(m_server_fd);
        m_server_fd = -1;
        return false;
    }

    int flags = fcntl(m_server_fd, F_GETFL, 0);
    fcntl(m_server_fd, F_SETFL, flags | O_NONBLOCK);

    std::cout << "[SimTcpServer] Server listening on port " << m_port << " (non-blocking RT mode)." << std::endl;
    return true;
}

void SimTcpServer::stop() {
    close_client();
    if (m_server_fd >= 0) {
        close(m_server_fd);
        m_server_fd = -1;
    }
}

void SimTcpServer::close_client() {
    if (m_client_fd >= 0) {
        close(m_client_fd);
        m_client_fd = -1;
        m_rx_buffer.clear();
    }
}

bool SimTcpServer::poll_client() {
    if (m_client_fd >= 0) return true;

    sockaddr_in client_addr{};
    socklen_t addr_len = sizeof(client_addr);
    m_client_fd = accept(m_server_fd, (struct sockaddr*)&client_addr, &addr_len);

    if (m_client_fd < 0) {
        return false;
    }

    fcntl(m_client_fd, F_SETFL, 0);
    struct timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 200000;
    setsockopt(m_client_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

    int nodelay = 1;
    setsockopt(m_client_fd, IPPROTO_TCP, TCP_NODELAY, (char*)&nodelay, sizeof(nodelay));

#ifdef SO_NOSIGPIPE
    int nosigpipe = 1;
    setsockopt(m_client_fd, SOL_SOCKET, SO_NOSIGPIPE, &nosigpipe, sizeof(nosigpipe));
#endif

    std::cout << "[SimTcpServer] Client connected from " 
              << inet_ntoa(client_addr.sin_addr) << ":" << ntohs(client_addr.sin_port) << std::endl;
    m_rx_buffer.clear();
    return true;
}

bool SimTcpServer::read_line(std::string& line) {
    if (m_client_fd < 0) return false;

    size_t newline_pos = m_rx_buffer.find('\n');
    if (newline_pos != std::string::npos) {
        line = m_rx_buffer.substr(0, newline_pos);
        m_rx_buffer.erase(0, newline_pos + 1);
        return true;
    }

    char recv_buf[1024];
    ssize_t bytes_read = recv(m_client_fd, recv_buf, sizeof(recv_buf) - 1, 0);

    if (bytes_read <= 0) {
        if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            return false;
        }
        std::cout << "[SimTcpServer] Client disconnected." << std::endl;
        close_client();
        return false;
    }

    recv_buf[bytes_read] = '\0';
    m_rx_buffer.append(recv_buf, bytes_read);

    newline_pos = m_rx_buffer.find('\n');
    if (newline_pos != std::string::npos) {
        line = m_rx_buffer.substr(0, newline_pos);
        m_rx_buffer.erase(0, newline_pos + 1);
        return true;
    }

    return false;
}

bool SimTcpServer::send_response(const std::string& response) {
    if (m_client_fd < 0) return false;
    ssize_t sent = send(m_client_fd, response.c_str(), response.size(), 0);
    if (sent < 0) {
        std::cerr << "[SimTcpServer] Send failed: " << strerror(errno) << std::endl;
        close_client();
        return false;
    }
    return true;
}

} // namespace services
