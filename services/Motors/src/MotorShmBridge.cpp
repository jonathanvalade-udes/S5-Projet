#include "MotorShmBridge.hpp"
#include <iostream>
#include <cmath>

namespace services {

bool MotorShmBridge::init() {
    m_shm_mgr = std::make_unique<SharedMemoryManager>();
    try {
        m_telemetry = m_shm_mgr->open_shm(true);
    } catch (const std::exception& e) {
        std::cerr << "[MotorShmBridge] Failed to open shared memory: " << e.what() << std::endl;
        return false;
    }
    if (!m_telemetry) {
        std::cerr << "[MotorShmBridge] Invalid shared memory pointer!" << std::endl;
        return false;
    }

    try {
        m_mem_server = std::make_unique<shared::services::memory::MemoryShareComponentServer>("Motors");
        m_servo_mem_server = std::make_unique<shared::services::memory::MemoryShareComponentServer>("Servo");
        m_fsm_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("FSM");
    } catch (const std::exception& e) {
        std::cerr << "[MotorShmBridge] Warning: Shared memory setup issue: " << e.what() << std::endl;
    }

    return true;
}

bool MotorShmBridge::read_fsm_command(float& speed, float& steer, float& shm_offset, bool& emergency, uint64_t& heartbeat) {
    if (!m_telemetry) return false;

    pthread_mutex_lock(&(m_telemetry->mtx));
    speed = m_telemetry->target_speed;
    steer = m_telemetry->target_steering;
    shm_offset = m_telemetry->steering_offset;
    emergency = m_telemetry->emergency_stop;
    heartbeat = m_telemetry->heartbeat;
    pthread_mutex_unlock(&(m_telemetry->mtx));

    return true;
}

bool MotorShmBridge::check_watchdog(uint64_t current_heartbeat, int max_misses) {
    if (current_heartbeat == m_last_heartbeat) {
        m_watchdog_starvation_cycles++;
    } else {
        m_watchdog_starvation_cycles = 0;
        m_last_heartbeat = current_heartbeat;
    }
    return (m_watchdog_starvation_cycles > max_misses);
}

void MotorShmBridge::update_telemetry_speed(float speed) {
    if (m_telemetry) {
        pthread_mutex_lock(&(m_telemetry->mtx));
        m_telemetry->speed = speed;
        pthread_mutex_unlock(&(m_telemetry->mtx));
    }
}

void MotorShmBridge::update_telemetry_failsafe() {
    if (m_telemetry) {
        pthread_mutex_lock(&(m_telemetry->mtx));
        m_telemetry->speed = 0.0f;
        m_telemetry->state = 99; // FAILSAFE
        pthread_mutex_unlock(&(m_telemetry->mtx));
    }
}

void MotorShmBridge::publish_telemetry(float target_speed, float target_steering) {
    if (m_mem_server && m_mem_server->isValid()) {
        shared::motors::MotorsComponentTelemetry telem{};
        telem.leftMotor.normalizedMotorThrottleApplied = target_speed / 100.0f;
        telem.rightMotor.normalizedMotorThrottleApplied = target_speed / 100.0f;

        telem.steeringServo.normalizedCurrentAngle = target_steering / shared::servos::MAX_ANGLE;
        telem.steeringServo.normalizedTargettedAngle = target_steering / shared::servos::MAX_ANGLE;
        telem.steeringServo.flags = shared::servos::flags::RDY;

        telem.componentFlags = shared::motors::flags::component::DISCOVERABLE;
        m_mem_server->publish(telem);
    }

    if (m_servo_mem_server && m_servo_mem_server->isValid()) {
        shared::servos::ServoTelemetry servo_telem{};
        servo_telem.normalizedCurrentAngle = target_steering / shared::servos::MAX_ANGLE;
        servo_telem.normalizedTargettedAngle = target_steering / shared::servos::MAX_ANGLE;
        servo_telem.flags = shared::servos::flags::RDY;
        m_servo_mem_server->publish(servo_telem);
    }
}

} // namespace services
