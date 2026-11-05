#pragma once

#include <atomic>
#include <string>
#include "LineEstimator.hpp"
#include "SensorAcquisition.hpp"
#include "SensorIpcHandler.hpp"
#include "SensorShmBridge.hpp"

namespace services {

class Sensors {
public:
    static constexpr int TRIG_PIN = 12;
    static constexpr int ECHO_PIN = 16;

    explicit Sensors(int hz = 100, int trig = TRIG_PIN, int echo = ECHO_PIN);
    ~Sensors();

    bool init();
    void run();
    void stop();

    bool is_running() const { return m_running.load(); }

    static float compute_line_error(const uint8_t sensors[5], float fallback_error = 0.0f) {
        return LineEstimator::compute_line_error(sensors, fallback_error);
    }
    static float compute_line_error_analog(const float darkness[5], float fallback_error = 0.0f) {
        return LineEstimator::compute_line_error_analog(darkness, fallback_error);
    }

private:
    std::atomic<bool> m_running{false};
    int m_hz{100};
    float m_filter_alpha{0.25f};
    size_t m_median_window{5};
    float m_last_line_error{0.0f};

    SensorAcquisition m_acquisition;
    SensorIpcHandler m_ipc_handler;
    SensorShmBridge m_shm_bridge;

    bool apply_config_key(const std::string& key, float value);
};

} // namespace services
