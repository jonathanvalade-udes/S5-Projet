#include "SensorShmBridge.hpp"
#include <iostream>

namespace services {

bool SensorShmBridge::init() {
    try {
        m_mem_server = std::make_unique<shared::services::memory::MemoryShareComponentServer>("Sensors");
    } catch (const std::exception& e) {
        std::cerr << "[SensorShmBridge] Warning: Failed to init MemoryShareComponentServer: " << e.what() << std::endl;
    }

    try {
        m_motors_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("Motors");
    } catch (const std::exception& e) {
        std::cerr << "[SensorShmBridge] Warning: Failed to init Motors subscriber: " << e.what() << std::endl;
    }

    return true;
}

float SensorShmBridge::read_current_speed() {
    float speed = 0.0f;
    if (m_motors_sub) {
        shared::motors::MotorsComponentTelemetry motors_telem{};
        if (m_motors_sub->read(motors_telem)) {
            speed = motors_telem.leftMotor.normalizedMotorThrottleApplied * 100.0f;
        }
    }
    return speed;
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
