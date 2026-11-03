#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <chrono>

// Shared headers from libs
#include "header/header.hpp"
#include "services/services_telemetry.hpp"
#include "car/full_packet.hpp"
#include "ipc/shared_memory.hpp"
#include "shared-services/memory/server.hpp"
#include "shared-services/memory/subscriber.hpp"

namespace services {

struct TelemetryMetrics {
    float current_speed_ms{0.0f};
    float current_accel_ms2{0.0f};
    float total_distance_m{0.0f};
    float filtered_dist_cm{100.0f};
    float target_speed_percent{0.0f};
    float target_steering_deg{0.0f};
    uint32_t fsm_state{0};
    uint8_t line_sensors[5]{0, 0, 0, 0, 0};
    bool emergency_stop{false};
    uint64_t heartbeat{0};
    uint32_t accel_violations{0};
    bool watchdog_stalled{false};
    uint64_t uptime_seconds{0};
};

class TelemetryService {
public:
    explicit TelemetryService(int hz = 20, bool enable_ui = true, int udp_port = 9002);
    ~TelemetryService();

    bool init();
    void run();
    void stop();

    bool is_running() const { return m_running.load(); }
    const TelemetryMetrics& get_metrics() const { return m_metrics; }

private:
    void update_metrics(float dt);
    void publish_ipc();
    void broadcast_udp();
    void render_dashboard();

    std::atomic<bool> m_running{false};
    int m_hz{20};
    bool m_enable_ui{true};
    int m_udp_port{9002};

    std::unique_ptr<SharedMemoryManager> m_shm_mgr;
    RobotTelemetry* m_telemetry{nullptr};

    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_telemetry_pub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_sensor_sub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_fsm_sub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_motor_sub;

    TelemetryMetrics m_metrics;
    float m_last_speed_ms{0.0f};
    uint64_t m_last_heartbeat{0};
    int m_heartbeat_stalled_cycles{0};
    int m_ui_render_divider{4}; // Render UI at 5 Hz (20 Hz / 4)
    int m_udp_fd{-1};
    std::chrono::steady_clock::time_point m_start_time;
};

} // namespace services
