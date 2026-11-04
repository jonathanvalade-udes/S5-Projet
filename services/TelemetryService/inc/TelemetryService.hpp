#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <queue>
#include <fstream>

// Shared headers from libs
#include "header/header.hpp"
#include "services/services_telemetry.hpp"
#include "car/full_packet.hpp"
#include "ipc/shared_memory.hpp"
#include "commands/global_commands.hpp"
#include "responses/global_responses.hpp"
#include "crc/crc_global.hpp"
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
    float steering_offset_deg{0.0f};
    uint32_t fsm_state{0};
    uint8_t line_sensors[5]{0, 0, 0, 0, 0};
    bool emergency_stop{false};
    uint64_t heartbeat{0};
    uint32_t accel_violations{0};
    bool watchdog_stalled{false};
    uint64_t uptime_seconds{0};
};

// Échantillon POD binaire brut ultra-rapide (44 octets, 0 allocation dynamique pour 100 kHz)
struct TelemetrySample {
    uint64_t timestamp_us{0};
    uint32_t uptime_s{0};
    uint32_t fsm_state{0};
    float speed_ms{0.0f};
    float accel_ms2{0.0f};
    float steer_deg{0.0f};
    float steer_offset_deg{0.0f};
    float dist_us_cm{0.0f};
    uint8_t line_sensors[5]{0, 0, 0, 0, 0};
    uint8_t emergency_stop{0};
    uint16_t accel_violations{0};
    float odometry_m{0.0f};
};

class TelemetryService {
public:
    explicit TelemetryService(int hz = 50, bool enable_ui = true, int tcp_port = 91, int udp_port = 9002);
    ~TelemetryService();

    bool init();
    void run();
    void stop();

    bool is_running() const { return m_running.load(); }
    const TelemetryMetrics& get_metrics() const { return m_metrics; }
    shared::car::CarTelemetry get_car_telemetry() const;
    bool is_client_connected() const { return m_client_connected.load(); }
    bool is_client_handshaked() const { return m_client_handshaked.load(); }
    int get_tcp_port() const { return m_tcp_port; }
    void set_tcp_port(int port) { m_tcp_port = port; }

private:
    void update_metrics(float dt);
    void collect_shared_memories();
    void publish_ipc();
    void broadcast_udp();
    void render_dashboard();

    // Enregistrement direct continu sur Clé USB via file FIFO (Zero lag, zero écriture carte SD, calibré 100 kHz)
    void push_to_usb_fifo(const TelemetrySample& sample);
    void usb_worker();
    std::string detect_usb_mount_point();

    // TCP Server
    bool init_tcp_server();
    void tcp_worker();
    void close_tcp_client();
    void close_tcp_server();
    bool handle_ping_pong();
    bool stream_telemetry_packet();

    std::atomic<bool> m_running{false};
    int m_hz{50};
    bool m_enable_ui{true};
    int m_tcp_port{91};
    int m_udp_port{9002};

    // Shared Memory components
    std::unique_ptr<SharedMemoryManager> m_shm_mgr;
    RobotTelemetry* m_telemetry{nullptr};

    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_telemetry_pub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_sensor_sub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_fsm_sub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_motor_sub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_servo_sub;

    // Full system telemetry (core payload)
    mutable std::mutex m_telem_mutex;
    shared::car::CarTelemetry m_car_telemetry{};

    TelemetryMetrics m_metrics;
    float m_last_speed_ms{0.0f};
    uint64_t m_last_heartbeat{0};
    int m_heartbeat_stalled_cycles{0};
    int m_ui_render_divider{10}; // Render UI at 5 Hz (50 Hz / 10)
    int m_udp_fd{-1};
    std::chrono::steady_clock::time_point m_start_time;

    // TCP Server state
    int m_tcp_server_fd{-1};
    int m_tcp_client_fd{-1};
    std::atomic<bool> m_client_connected{false};
    std::atomic<bool> m_client_handshaked{false};
    uint16_t m_tcp_seq_num{0};
    std::thread m_tcp_thread;

    // Synchronisation temps N pour streaming instantané
    std::mutex m_stream_mutex;
    std::condition_variable m_stream_cv;
    std::atomic<bool> m_new_data_available{false};

    // File FIFO haute performance (calibrée 100 kHz) et Thread d'arrière-plan pour clé USB
    std::queue<TelemetrySample> m_sample_fifo;
    std::mutex m_fifo_mutex;
    std::condition_variable m_fifo_cv;
    std::thread m_usb_thread;
    std::string m_active_usb_file_path;
    std::ofstream m_usb_stream;
    static constexpr size_t MAX_FIFO_SIZE = 100000;    // 100 000 échantillons en RAM (~4.4 Mo, 1s d'amortissement à 100 kHz)
    static constexpr size_t BATCH_NOTIFY_SIZE = 100;   // Réduit les context switches de 99%
};

} // namespace services
