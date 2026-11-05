#include "PosixClient.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cstring>
#include <cerrno>

#if !defined(MSG_NOSIGNAL)
#define MSG_NOSIGNAL 0
#endif

namespace services {

PosixClient::PosixClient(std::string default_socket_path)
    : m_default_socket_path(std::move(default_socket_path)) {}

PosixClient::~PosixClient() {
    disconnect();
}

int PosixClient::get_or_connect(const std::string& path) {
    std::string target = path.empty() ? m_default_socket_path : path;

    auto it = m_connections.find(target);
    if (it != m_connections.end() && it->second >= 0) {
        return it->second;
    }

    // 1. Attempt connection via UNIX domain stream socket
    int sock = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock >= 0) {
        int flags = fcntl(sock, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(sock, F_SETFL, flags | O_NONBLOCK);
        }

        struct sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, target.c_str(), sizeof(addr.sun_path) - 1);

        if (::connect(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == 0) {
            m_connections[target] = sock;
            return sock;
        }
        ::close(sock);
    }

    // 2. Fallback to POSIX FIFO / pipe
    int fifo = ::open(target.c_str(), O_WRONLY | O_NONBLOCK);
    if (fifo >= 0) {
        m_connections[target] = fifo;
        return fifo;
    }

    // 3. Fallback to default socket if target was service-specific
    if (target != m_default_socket_path && !m_default_socket_path.empty()) {
        auto def_it = m_connections.find(m_default_socket_path);
        if (def_it != m_connections.end() && def_it->second >= 0) {
            return def_it->second;
        }
        return get_or_connect(m_default_socket_path);
    }

    return -1;
}

bool PosixClient::connect(const std::string& path) {
    int fd = get_or_connect(path);
    return fd >= 0;
}

void PosixClient::disconnect(const std::string& path) {
    if (path.empty()) {
        for (auto& [p, fd] : m_connections) {
            if (fd >= 0) {
                ::close(fd);
            }
        }
        m_connections.clear();
    } else {
        auto it = m_connections.find(path);
        if (it != m_connections.end()) {
            if (it->second >= 0) {
                ::close(it->second);
            }
            m_connections.erase(it);
        }
    }
}

bool PosixClient::send_packet(const void* data, size_t len, const std::string& target_path) {
    if (!data || len == 0) return false;

    int fd = get_or_connect(target_path);
    if (fd < 0) return false;

    ssize_t w = ::send(fd, data, len, MSG_NOSIGNAL);
    if (w < 0 && (errno == ENOTSOCK || errno == EOPNOTSUPP)) {
        w = ::write(fd, data, len);
    }

    if (w > 0) {
        return true;
    }

    std::string key = target_path.empty() ? m_default_socket_path : target_path;
    disconnect(key);
    return false;
}

bool PosixClient::send_command(const std::string& cmd, const std::string& target_path) {
    if (cmd.empty()) return false;
    std::string payload = cmd;
    if (payload.back() != '\n') {
        payload.push_back('\n');
    }
    return send_packet(payload.data(), payload.size(), target_path);
}

void PosixClient::poll_and_drain() {
    for (auto it = m_connections.begin(); it != m_connections.end();) {
        int fd = it->second;
        if (fd < 0) {
            it = m_connections.erase(it);
            continue;
        }

        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;

        int ret = poll(&pfd, 1, 0);
        if (ret > 0) {
            if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
                ::close(fd);
                it = m_connections.erase(it);
                continue;
            } else if (pfd.revents & POLLIN) {
                char drain_buf[512];
                ssize_t n = ::recv(fd, drain_buf, sizeof(drain_buf), 0);
                if (n < 0 && (errno == ENOTSOCK || errno == EOPNOTSUPP)) {
                    n = ::read(fd, drain_buf, sizeof(drain_buf));
                }
                if (n <= 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    ::close(fd);
                    it = m_connections.erase(it);
                    continue;
                }
            }
        }
        ++it;
    }
}

} // namespace services
