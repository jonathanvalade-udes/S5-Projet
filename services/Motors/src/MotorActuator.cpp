#include "MotorActuator.hpp"
#include <iostream>

namespace services {

MotorActuator::MotorActuator(bool invert, float steering_offset)
    : m_invert(invert), m_steering_offset(steering_offset) {}

bool MotorActuator::init() {
    try {
        m_driver = std::make_unique<PiCarHatsMotorDriver>(picar::i2c::ADDR_PCA9685, m_invert);
        m_driver->set_steering_offset(m_steering_offset);
        std::cout << "[MotorActuator] Hardware driver initialized successfully." << std::endl;
    } catch (const std::exception& e) {
        std::cout << "[MotorActuator] Note: Hardware PCA9685/TB6612 fallback (" << e.what() << ")." << std::endl;
        m_driver.reset();
    }
    return true;
}

void MotorActuator::update_actuation(float target_speed, float target_steering, float dt, bool emergency_stop_requested) {
    if (emergency_stop_requested) {
        emergency_stop();
        return;
    }

    if (m_driver) {
        // Continuous time-scaled slew-rate limiter (~175 deg/s straight, ~300 deg/s curve)
        float max_steer_rate = (std::abs(target_steering) < 15.0f) ? 175.0f : 300.0f;
        float max_steer_step = max_steer_rate * dt;
        float delta = std::clamp(target_steering - m_applied_steering, -max_steer_step, max_steer_step);
        m_applied_steering += delta;

        m_driver->set_speed(target_speed);
        m_driver->set_steering(m_applied_steering);
    } else {
        m_applied_steering = target_steering;
    }
}

void MotorActuator::emergency_stop() {
    if (m_driver) {
        m_driver->emergency_stop();
    }
    m_applied_steering = 0.0f;
}

void MotorActuator::set_steering_offset(float offset_deg) {
    m_steering_offset = std::clamp(offset_deg, -15.0f, 15.0f);
    if (m_driver) {
        m_driver->set_steering_offset(m_steering_offset);
    }
}

} // namespace services
