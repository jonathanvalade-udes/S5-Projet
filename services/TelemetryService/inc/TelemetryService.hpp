#pragma once

#include <atomic>
#include <string>
#include <vector>
#include "PosixClient.hpp"
#include "TcpStreamer.hpp"
#include "ShmCollector.hpp"
#include "UsbLogger.hpp"

namespace services {

/**
 * @brief Real-time Telemetry Service hub.
 * Coordinates reading shared memory from other services, forwarding commands over POSIX IPC,
 * and streaming CarTelemetry frames over TCP.
 */
class TelemetryService {
public:
    explicit TelemetryService(int hz = 100000, bool enable_ui = false, int tcp_port = 91, int udp_port = 9002,
                              std::string posix_socket_path = "/tmp/peripherals.sock");
    ~TelemetryService();

    bool init();
    void run();
    void stop();

    bool is_running() const { return m_running.load(); }
    const TelemetryMetrics& get_metrics() const { return m_collector.get_metrics(); }
    shared::car::CarTelemetry get_car_telemetry() const { return m_collector.get_telemetry(); }
    bool is_client_connected() const { return m_tcp_streamer.is_client_connected(); }
    bool is_client_handshaked() const { return m_tcp_streamer.is_client_handshaked(); }
    int get_tcp_port() const { return m_tcp_streamer.get_port(); }
    void set_tcp_port(int port) { m_tcp_streamer.set_port(port); }

    // POSIX connection to other services
    bool open_posix_connection(const std::string& path = "") { return m_posix_client.connect(path); }
    void close_posix_connection(const std::string& path = "") { m_posix_client.disconnect(path); }
    bool send_posix_message(const std::string& msg, const std::string& target = "") {
        return m_posix_client.send_command(msg, target);
    }
    bool send_posix_packet(const void* data, size_t len, const std::string& target = "") {
        return m_posix_client.send_packet(data, len, target);
    }
    int get_posix_fd(const std::string& path = "") const { return m_posix_client.get_fd(path); }
    const std::string& get_posix_socket_path() const { return m_posix_client.get_socket_path(); }
    void set_posix_socket_path(const std::string& path) { m_posix_client.set_socket_path(path); }

    std::string resolve_service_socket(const std::vector<uint8_t>& payload) const;

    // Backward-compatibility no-op
    void publish_ipc() {}

private:
    void handle_client_command(const shared::headers::PacketHeader& hdr, const std::vector<uint8_t>& payload);
    void broadcast_udp();

    std::atomic<bool> m_running{false};
    int m_hz{50};
    bool m_enable_ui{false};
    int m_udp_port{9002};
    int m_udp_fd{-1};

    PosixClient m_posix_client;
    TcpStreamer m_tcp_streamer;
    ShmCollector m_collector;
    UsbLogger m_usb_logger;
};

} // namespace services
