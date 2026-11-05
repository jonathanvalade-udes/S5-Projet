#include "MotorShmBridge.hpp"
#include <iostream>
#include <cmath>

namespace services {

bool MotorShmBridge::init() {
    try {
        m_mem_server = std::make_unique<shared::services::memory::MemoryShareComponentServer>("Motors");
        m_servo_mem_server = std::make_unique<shared::services::memory::MemoryShareComponentServer>("Servo");
        m_fsm_cmd_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("FsmCommand");
    } catch (const std::exception& e) {
        std::cerr << "[MotorShmBridge] Warning: Shared memory setup issue: " << e.what() << std::endl;
    }

    return true;
}

bool MotorShmBridge::read_fsm_command(float& speed, float& steer, float& shm_offset, bool& emergency, uint64_t& heartbeat) {
    if (!m_fsm_cmd_sub) return false;

    shared::fsm::FsmCommandPayload cmd{};
    if (!m_fsm_cmd_sub->read(cmd)) {
        return false;
    }

    speed = cmd.target_speed;
    steer = cmd.target_steering;
    shm_offset = cmd.steering_offset;
    emergency = (cmd.emergency_stop != 0);
    heartbeat = cmd.heartbeat;

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
