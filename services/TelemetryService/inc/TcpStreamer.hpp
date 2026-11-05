#pragma once

#include <atomic>
#include <functional>
#include <cstdint>
#include <vector>
#include "header/header.hpp"
#include "car/full_packet.hpp"
#include "crc/crc_global.hpp"

namespace services {

/**
 * @brief Non-blocking single-threaded TCP server and telemetry streaming engine.
 * Handles client connection, Ping-Pong handshake, command routing, and CarTelemetry streaming.
 */
class TcpStreamer {
public:
    using CommandCallback = std::function<void(const shared::headers::PacketHeader& hdr,
                                               const std::vector<uint8_t>& payload)>;

    explicit TcpStreamer(int port = 91);
    ~TcpStreamer();

    bool init(int port = -1);
    void stop();

    void poll_events(const CommandCallback& on_command);
    bool stream_packet(const shared::car::CarTelemetry& telemetry);

    bool is_client_connected() const { return m_client_connected.load(); }
    bool is_client_handshaked() const { return m_client_handshaked.load(); }
    int get_port() const { return m_port; }

private:
    void accept_client();
    void handle_client_data(const CommandCallback& on_command);
    void close_client();

    int m_port{91};
    int m_server_fd{-1};
    int m_client_fd{-1};
    std::atomic<bool> m_client_connected{false};
    std::atomic<bool> m_client_handshaked{false};
    uint16_t m_seq_num{0};

    // Pre-allocated packet buffer for zero-allocation real-time streaming
    uint8_t m_packet_buf[sizeof(shared::headers::PacketHeader) +
                         sizeof(shared::car::CarTelemetry) +
                         sizeof(shared::crc::PacketCRC)];
};

} // namespace services
