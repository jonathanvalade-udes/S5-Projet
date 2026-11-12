#include "TelemetryService.hpp"
#include "commands/commands_header.hpp"
#include "control/picar_config.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <csignal>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cstring>
#include <sstream>

namespace services {

static const char* state_to_string(uint32_t state) {
    switch (static_cast<shared::fsm::MachineStates>(state)) {
        case shared::fsm::MachineStates::Init: return "Init";
        case shared::fsm::MachineStates::Unarmed: return "Unarmed";
        case shared::fsm::MachineStates::Armed: return "Armed";
        case shared::fsm::MachineStates::LineFollowingForward: return "LineFollowingForward";
        case shared::fsm::MachineStates::CollisionRisk: return "CollisionRisk";
        case shared::fsm::MachineStates::Stopped: return "Stopped";
        case shared::fsm::MachineStates::RightDeviation: return "RightDeviation";
        case shared::fsm::MachineStates::LeftDeviation: return "LeftDeviation";
        case shared::fsm::MachineStates::Abort: return "Abort";
        case shared::fsm::MachineStates::Failure: return "Failure";
        default: return "Undefined";
    }
}

TelemetryService::TelemetryService(int hz, bool /*enable_ui*/, int tcp_port, int udp_port,
                                   std::string posix_socket_path)
    : m_running(false),
      m_hz(hz > 0 ? hz : 100),
      m_udp_port(udp_port),
      m_posix_client(std::move(posix_socket_path)),
      m_tcp_streamer(tcp_port),
      m_collector(m_hz) {}

TelemetryService::~TelemetryService() {
    stop();
    if (m_udp_fd >= 0) {
        close(m_udp_fd);
        m_udp_fd = -1;
    }
}

bool TelemetryService::init() {
    std::signal(SIGPIPE, SIG_IGN);
    m_collector.init();
    m_posix_client.connect();

    if (m_tcp_streamer.get_port() > 0) {
        m_tcp_streamer.init();
    }

    if (m_udp_port > 0) {
        m_udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_udp_fd >= 0) {
            int broadcast = 1;
            setsockopt(m_udp_fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
            int flags = fcntl(m_udp_fd, F_GETFL, 0);
            fcntl(m_udp_fd, F_SETFL, flags | O_NONBLOCK);
        }
    }

    m_usb_logger.init();
    m_running.store(true);
    return true;
}

std::string TelemetryService::resolve_service_socket(const std::vector<uint8_t>& payload) const {
    if (payload.size() >= sizeof(shared::commands::CommandsHeader)) {
        const auto* cmd_hdr = reinterpret_cast<const shared::commands::CommandsHeader*>(payload.data());
        switch (cmd_hdr->serviceID) {
            case shared::commands::ServiceID::FSM:
                return "/tmp/fsm.sock";
            case shared::commands::ServiceID::MOTORS:
                return "/tmp/motors.sock";
            case shared::commands::ServiceID::SENSORS:
                return "/tmp/sensors.sock";
            case shared::commands::ServiceID::TELEMETRY:
                return m_posix_client.get_socket_path();
            default:
                break;
        }
    }

    if (!payload.empty()) {
        std::string str(payload.begin(), payload.end());
        if (str.find("trigger_start") != std::string::npos || str.find("start") != std::string::npos) {
            return "/tmp/fsm.sock";
        }
        if (str.find("emergency_stop") != std::string::npos || str.find("stop") != std::string::npos) {
            return "/tmp/motors.sock";
        }
    }

    return m_posix_client.get_socket_path();
}

void TelemetryService::handle_client_command(const shared::headers::PacketHeader& hdr,
                                             const std::vector<uint8_t>& payload) {
    // 1. Prepare the same packet command without the CRC
    shared::headers::PacketHeader fwd_hdr = hdr;
    fwd_hdr.headerFlag &= ~shared::headers::flags::CRC_ENABLED;

    std::vector<uint8_t> fwd_packet(sizeof(fwd_hdr) + payload.size());
    std::memcpy(fwd_packet.data(), &fwd_hdr, sizeof(fwd_hdr));
    if (!payload.empty()) {
        std::memcpy(fwd_packet.data() + sizeof(fwd_hdr), payload.data(), payload.size());
    }

    // 2. Resolve the target service
    std::string target_socket = resolve_service_socket(payload);

    // 3. Redirect the packet command without CRC to the target service
    m_posix_client.send_packet(fwd_packet.data(), fwd_packet.size(), target_socket);

    // 4. Update configuration if applicable
    if (!payload.empty()) {
        std::string cmd_str(payload.begin(), payload.end());
        size_t eq = cmd_str.find('=');
        if (eq != std::string::npos) {
            std::string k = cmd_str.substr(0, eq);
            std::string v = cmd_str.substr(eq + 1);
            k.erase(std::remove_if(k.begin(), k.end(), ::isspace), k.end());
            v.erase(std::remove_if(v.begin(), v.end(), ::isspace), v.end());
            try {
                float fval = std::stof(v);
                config::PicarConfig::write_config(k, fval);
            } catch (...) {}
        }
    }
}

void TelemetryService::broadcast_udp() {
    if (m_udp_fd < 0 || m_udp_port <= 0) return;

    const auto& metrics = m_collector.get_metrics();
    std::ostringstream json;
    json << "{"
         << "\"uptime\":" << metrics.uptime_seconds << ","
         << "\"speed_ms\":" << metrics.current_speed_ms << ","
         << "\"speed_pct\":" << metrics.target_speed_percent << ","
         << "\"accel_ms2\":" << metrics.current_accel_ms2 << ","
         << "\"steer_deg\":" << metrics.target_steering_deg << ","
         << "\"steer_offset_deg\":" << metrics.steering_offset_deg << ","
         << "\"dist_cm\":" << metrics.filtered_dist_cm << ","
         << "\"odometry_m\":" << metrics.total_distance_m << ","
         << "\"fsm_state\":\"" << state_to_string(metrics.fsm_state) << "\","
         << "\"line\":["
         << (int)metrics.line_sensors[0] << ","
         << (int)metrics.line_sensors[1] << ","
         << (int)metrics.line_sensors[2] << ","
         << (int)metrics.line_sensors[3] << ","
         << (int)metrics.line_sensors[4] << "],"
         << "\"heartbeat\":" << metrics.heartbeat << ","
         << "\"emergency\":" << (metrics.emergency_stop ? "true" : "false") << ","
         << "\"violations\":" << metrics.accel_violations
         << "}\n";

    std::string payload = json.str();

    struct sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(m_udp_port);
    dest_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    sendto(m_udp_fd, payload.data(), payload.size(), 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
}

void TelemetryService::run() {
    const auto period = std::chrono::nanoseconds(1000000000LL / m_hz);
    auto next_deadline = std::chrono::steady_clock::now();
    const float dt = 1.0f / static_cast<float>(m_hz);

    while (m_running.load()) {
        next_deadline += period;

        // 1. Non-blocking TCP events: redirect received commands without CRC to services
        m_tcp_streamer.poll_events([this](const shared::headers::PacketHeader& hdr,
                                          const std::vector<uint8_t>& payload) {
            handle_client_command(hdr, payload);
        });

        // 2. Non-blocking POSIX socket drain
        m_posix_client.poll_and_drain();

        // 3. Read shared memory regions and update metrics
        m_collector.collect(dt);

        // 4. Stream CarTelemetry packet over TCP
        if (m_tcp_streamer.is_client_handshaked()) {
            m_tcp_streamer.stream_packet(m_collector.get_telemetry());
        }

        // 5. Broadcast UDP (if enabled)
        broadcast_udp();

        // 6. Push to background USB logger (if active)
        if (m_usb_logger.is_active()) {
            m_usb_logger.push_sample(m_collector.get_sample());
        }

        // 7. Deterministic RT sleep without delay
        auto now = std::chrono::steady_clock::now();
        if (now < next_deadline) {
            std::this_thread::sleep_until(next_deadline);
        } else {
            next_deadline = now;
        }
    }
}

void TelemetryService::stop() {
    if (m_running.load()) {
        m_running.store(false);
    }
    m_tcp_streamer.stop();
    m_posix_client.disconnect();
    m_usb_logger.stop();
}

} // namespace services
