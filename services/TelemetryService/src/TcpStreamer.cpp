#include "TcpStreamer.hpp"
#include "commands/global_commands.hpp"
#include "responses/global_responses.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <vector>

#if !defined(MSG_NOSIGNAL)
#define MSG_NOSIGNAL 0
#endif

namespace services {

TcpStreamer::TcpStreamer(int port) : m_port(port) {}

TcpStreamer::~TcpStreamer() {
    stop();
}

bool TcpStreamer::init(int port) {
    if (port > 0) {
        m_port = port;
    }
    if (m_port <= 0) {
        return false;
    }

    m_server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_server_fd < 0) {
        return false;
    }

    int opt = 1;
    setsockopt(m_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(SO_REUSEPORT)
    setsockopt(m_server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

    struct sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(m_port);

    if (bind(m_server_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        close(m_server_fd);
        m_server_fd = -1;
        return false;
    }

    if (listen(m_server_fd, 5) < 0) {
        close(m_server_fd);
        m_server_fd = -1;
        return false;
    }

    int flags = fcntl(m_server_fd, F_GETFL, 0);
    fcntl(m_server_fd, F_SETFL, flags | O_NONBLOCK);
    return true;
}

void TcpStreamer::stop() {
    close_client();
    if (m_server_fd >= 0) {
        close(m_server_fd);
        m_server_fd = -1;
    }
}

void TcpStreamer::close_client() {
    if (m_client_fd >= 0) {
        close(m_client_fd);
        m_client_fd = -1;
    }
    m_client_connected.store(false);
    m_client_handshaked.store(false);
}

void TcpStreamer::poll_events(const CommandCallback& on_command) {
    if (m_server_fd < 0) return;

    if (m_client_fd < 0) {
        accept_client();
        return;
    }

    struct pollfd pfd{};
    pfd.fd = m_client_fd;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, 0);
    if (ret > 0) {
        if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
            close_client();
        } else if (pfd.revents & POLLIN) {
            handle_client_data(on_command);
        }
    }
}

void TcpStreamer::accept_client() {
    struct sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);
    int cfd = accept(m_server_fd, (struct sockaddr*)&client_addr, &client_len);
    if (cfd >= 0) {
        int flags = fcntl(cfd, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(cfd, F_SETFL, flags | O_NONBLOCK);
        }

        int nodelay = 1;
        setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

#if defined(SO_NOSIGPIPE)
        int nosig = 1;
        setsockopt(cfd, SOL_SOCKET, SO_NOSIGPIPE, &nosig, sizeof(nosig));
#endif
        int sndbuf = 65536;
        setsockopt(cfd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

        m_client_fd = cfd;
        m_client_connected.store(true);
        m_client_handshaked.store(false);
    }
}

void TcpStreamer::handle_client_data(const CommandCallback& on_command) {
    if (m_client_fd < 0) return;

    shared::headers::PacketHeader rx_hdr{};
    ssize_t n = recv(m_client_fd, &rx_hdr, sizeof(rx_hdr), MSG_PEEK);
    if (n <= 0) {
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
        close_client();
        return;
    }
    if (static_cast<size_t>(n) < sizeof(rx_hdr)) return;

    bool has_crc = (rx_hdr.headerFlag & shared::headers::flags::CRC_ENABLED) != 0;
    size_t expected_packet_len = sizeof(shared::headers::PacketHeader) +
                                 rx_hdr.payloadLength +
                                 (has_crc ? sizeof(shared::crc::PacketCRC) : 0);

    if (expected_packet_len > 4096) {
        close_client();
        return;
    }

    std::vector<uint8_t> rx_buffer(expected_packet_len);
    size_t received = 0;
    while (received < expected_packet_len) {
        ssize_t r = recv(m_client_fd, rx_buffer.data() + received, expected_packet_len - received, 0);
        if (r <= 0) {
            if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                struct pollfd pfd{};
                pfd.fd = m_client_fd;
                pfd.events = POLLIN;
                if (poll(&pfd, 1, 50) > 0) {
                    continue;
                }
            }
            close_client();
            return;
        }
        received += r;
    }

    shared::headers::PacketHeader hdr{};
    std::vector<uint8_t> payload;
    uint32_t crc_val = 0;
    if (!shared::headers::deserializePacket(rx_buffer.data(), rx_buffer.size(), hdr, payload, crc_val)) {
        return;
    }

    if (hdr.type == shared::headers::HeadersType::Command) {
        bool is_ping = false;
        if (payload.size() == sizeof(uint32_t)) {
            uint32_t cmd_val = 0;
            std::memcpy(&cmd_val, payload.data(), sizeof(cmd_val));
            if (cmd_val == static_cast<uint32_t>(shared::commands::CommandID::Ping)) {
                is_ping = true;
            }
        }

        if (!is_ping && on_command) {
            on_command(hdr, payload);
        }

        if (is_ping || (hdr.headerFlag & shared::headers::flags::REPLY_REQUESTED)) {
            shared::headers::PacketHeader pong_hdr{};
            pong_hdr.type = shared::headers::HeadersType::Response;
            pong_hdr.headerFlag = shared::headers::flags::CRC_ENABLED;
            pong_hdr.seqNumber = hdr.seqNumber;
            pong_hdr.payloadLength = sizeof(uint32_t);

            uint32_t pong_code = static_cast<uint32_t>(shared::responses::ResponseID::Pong);
            uint8_t pong_buf[sizeof(shared::headers::PacketHeader) + sizeof(uint32_t) + sizeof(shared::crc::PacketCRC)];
            int sz = shared::headers::serializePacket(
                pong_buf,
                sizeof(pong_buf),
                pong_hdr,
                reinterpret_cast<const uint8_t*>(&pong_code),
                sizeof(pong_code)
            );

            if (sz > 0) {
                size_t sent = 0;
                while (sent < static_cast<size_t>(sz)) {
                    ssize_t w = send(m_client_fd, pong_buf + sent, sz - sent, MSG_NOSIGNAL);
                    if (w <= 0) {
                        close_client();
                        return;
                    }
                    sent += w;
                }
            }
            m_client_handshaked.store(true);
        }
    }
}

bool TcpStreamer::stream_packet(const shared::car::CarTelemetry& telemetry) {
    if (m_client_fd < 0 || !m_client_handshaked.load()) {
        return false;
    }

    shared::headers::PacketHeader hdr{};
    hdr.type = shared::headers::HeadersType::Telemetry;
    hdr.headerFlag = shared::headers::flags::CRC_ENABLED;
    hdr.seqNumber = m_seq_num++;
    hdr.payloadLength = sizeof(telemetry);

    int packet_size = shared::headers::serializePacket(
        m_packet_buf,
        sizeof(m_packet_buf),
        hdr,
        reinterpret_cast<const uint8_t*>(&telemetry),
        sizeof(telemetry)
    );

    if (packet_size <= 0) {
        return false;
    }

    size_t sent = 0;
    while (sent < static_cast<size_t>(packet_size)) {
        ssize_t w = send(m_client_fd, m_packet_buf + sent, packet_size - sent, MSG_NOSIGNAL);
        if (w <= 0) {
            if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                break;
            }
            close_client();
            return false;
        }
        sent += w;
    }

    return (sent == static_cast<size_t>(packet_size));
}

} // namespace services
