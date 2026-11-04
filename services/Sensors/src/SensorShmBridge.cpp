#include "SensorShmBridge.hpp"
#include "control/picar_config.hpp"
#include <iostream>

namespace services {

bool SensorShmBridge::init() {
    try {
        m_shm_mgr = std::make_unique<SharedMemoryManager>();
        m_telemetry = m_shm_mgr->open_shm(true);
        if (m_telemetry) {
            auto initial_cfg = config::PicarConfig::read_config();
            config::PicarConfig::apply_to_shm(initial_cfg, m_telemetry);
        }
    } catch (const std::exception& e) {
        std::cerr << "[SensorShmBridge] Warning: Failed to open POSIX SHM: " << e.what() << std::endl;
        m_telemetry = nullptr;
    }

    try {
        m_mem_server = std::make_unique<shared::services::memory::MemoryShareComponentServer>("Sensors");
    } catch (const std::exception& e) {
        std::cerr << "[SensorShmBridge] Warning: Failed to init MemoryShareComponentServer: " << e.what() << std::endl;
    }

    return true;
}

float SensorShmBridge::read_current_speed() {
    float speed = 0.0f;
    if (m_telemetry) {
        pthread_mutex_lock(&(m_telemetry->mtx));
        speed = m_telemetry->speed;
        pthread_mutex_unlock(&(m_telemetry->mtx));
    }
    return speed;
}

void SensorShmBridge::update_legacy_telemetry(float filtered_dist, float line_error,
                                             const uint8_t line_sensors[5], const float line_darkness[5]) {
    if (!m_telemetry) return;

    pthread_mutex_lock(&(m_telemetry->mtx));
    m_telemetry->filtered_dist = filtered_dist;
    m_telemetry->line_error = line_error;
    for (int i = 0; i < 5; ++i) {
        m_telemetry->line_sensors[i] = line_sensors[i];
        m_telemetry->line_darkness[i] = line_darkness[i];
    }
    m_telemetry->heartbeat++;
    pthread_mutex_unlock(&(m_telemetry->mtx));
}

void SensorShmBridge::publish_telemetry(float filtered_dist, const float line_darkness[5],
                                       bool has_line, int consecutive_us_errors) {
    if (!m_mem_server || !m_mem_server->isValid()) return;

    shared::sensors::SensorsTelemetry telem{};
    telem.ultrasonicSensor.calculatedDistance = filtered_dist;
    telem.ultrasonicSensor.rawDistanceCm = filtered_dist;
    telem.ultrasonicSensor.flags = shared::sensors::ultrasonic::flags::SENSOR_ACTIVATED;

    for (int i = 0; i < 5; ++i) {
        telem.lineFollower.sensorsData[i].normalizedValue = line_darkness[i];
    }
    telem.lineFollower.flags = has_line ? shared::sensors::lineFollower::flags::LINE_FOLLOWED
                                        : shared::sensors::lineFollower::flags::LINE_LOST;
    telem.flags = shared::sensors::flags::DISCOVERABLE;
    if (consecutive_us_errors >= 5) {
        telem.flags |= shared::sensors::flags::DRIVER_ERROR;
        telem.ultrasonicSensor.flags = shared::sensors::ultrasonic::flags::ANOMALIES_DETECTED;
    }
    if (!has_line) {
        telem.flags |= shared::sensors::flags::WARNING_DETECTED;
    }
    m_mem_server->publish(telem);
}

} // namespace services
