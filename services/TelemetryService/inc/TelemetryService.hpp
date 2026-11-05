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
    explicit TelemetryService(int hz = 100, bool enable_ui = false, int tcp_port = 91, int udp_port = 9002,
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

    // POSIX connection to other services
    bool send_posix_message(const std::string& msg, const std::string& target = "") {
        return m_posix_client.send_command(msg, target);
    }
    const std::string& get_posix_socket_path() const { return m_posix_client.get_socket_path(); }

    std::string resolve_service_socket(const std::vector<uint8_t>& payload) const;

private:
    void handle_client_command(const shared::headers::PacketHeader& hdr, const std::vector<uint8_t>& payload);
    void broadcast_udp();

    std::atomic<bool> m_running{false};
    int m_hz{100};
    int m_udp_port{9002};
    int m_udp_fd{-1};

    PosixClient m_posix_client;
    TcpStreamer m_tcp_streamer;
    ShmCollector m_collector;
    UsbLogger m_usb_logger;
};

} // namespace services
