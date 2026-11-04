#pragma once

#include <memory>
#include <mutex>
#include <chrono>
#include "car/full_packet.hpp"
#include "ipc/shared_memory.hpp"
#include "shared-services/memory/subscriber.hpp"
#include "UsbLogger.hpp" // For TelemetrySample

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

/**
 * @brief Reads shared memory regions of other services (Sensors, Motors, Servo, FSM).
 * Strictly read-only: never publishes to or creates shared memory.
 * Computes speed, acceleration, odometry, and watchdog health metrics.
 */
class ShmCollector {
public:
    explicit ShmCollector(int hz = 50);
    ~ShmCollector();

    bool init();
    void collect(float dt);

    shared::car::CarTelemetry get_telemetry() const;
    const TelemetryMetrics& get_metrics() const { return m_metrics; }
    TelemetrySample get_sample() const;

private:
    int m_hz{50};

    // Shared Memory components (READ-ONLY from other services)
    std::unique_ptr<SharedMemoryManager> m_shm_mgr;
    RobotTelemetry* m_telemetry{nullptr};

    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_sensor_sub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_fsm_sub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_motor_sub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_servo_sub;

    mutable std::mutex m_mutex;
    shared::car::CarTelemetry m_car_telemetry{};
    TelemetryMetrics m_metrics;

    float m_last_speed_ms{0.0f};
    uint64_t m_last_heartbeat{0};
    int m_heartbeat_stalled_cycles{0};
    std::chrono::steady_clock::time_point m_start_time;
};

} // namespace services
