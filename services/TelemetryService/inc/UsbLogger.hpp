#pragma once

#include <cstdint>
#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <fstream>
#include <atomic>

namespace services {

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

/**
 * @brief Threaded USB data logger.
 * Uses a fixed-capacity FIFO to stream telemetry directly to a mounted USB drive.
 * Completely inactive (spawns 0 threads) when no USB drive is detected.
 */
class UsbLogger {
public:
    UsbLogger();
    ~UsbLogger();

    bool init();
    void stop();
    void push_sample(const TelemetrySample& sample);
    bool is_active() const { return m_active; }

    static std::string detect_mount_point();

private:
    void worker_loop();

    bool m_active{false};
    std::atomic<bool> m_running{false};
    std::queue<TelemetrySample> m_fifo;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::thread m_thread;

    std::ofstream m_stream;

    static constexpr size_t MAX_FIFO_SIZE = 100000;
    static constexpr size_t BATCH_NOTIFY_SIZE = 100;
};

} // namespace services
