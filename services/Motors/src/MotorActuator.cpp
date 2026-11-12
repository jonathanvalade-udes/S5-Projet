#include "MotorActuator.hpp"
#include <iostream>
#include <cstdlib>
#include <string>

namespace services {

MotorActuator::MotorActuator(bool invert, float steering_offset)
    : m_invert(invert), m_steering_offset(steering_offset) {}

bool MotorActuator::init(bool require_hardware) {
    const char* env_req = std::getenv("PICAR_REQUIRE_HARDWARE");
    if (env_req && (std::string(env_req) == "1" || std::string(env_req) == "true")) {
        require_hardware = true;
    }

    try {
        m_driver = std::make_unique<PiCarHatsMotorDriver>(picar::i2c::ADDR_PCA9685, m_invert);
        m_driver->set_steering_offset(m_steering_offset);
        std::cout << "[MotorActuator] Hardware driver initialized successfully." << std::endl;
    } catch (const std::exception& e) {
        std::cout << "[MotorActuator] Note: Hardware PCA9685/TB6612 fallback (" << e.what() << ")." << std::endl;
        m_driver.reset();
        if (require_hardware) {
            std::cerr << "[MotorActuator] ERROR: Hardware motor driver required but failed to initialize!" << std::endl;
            return false;
        }
    }
    return true;
}

void MotorActuator::update_actuation(float target_speed, float target_steering, float dt, bool emergency_stop_requested) {
    if (emergency_stop_requested) {
        emergency_stop();
        return;
    }

    // Protection temporelle
    if (dt <= 0.0f) {
        dt = 0.01f;
    } else if (dt > 0.2f) {
        dt = 0.2f;
    }

    // Protection contre les valeurs non finies (NaN / Inf)
    if (!std::isfinite(target_speed)) {
        target_speed = 0.0f;
    }
    if (!std::isfinite(target_steering)) {
        target_steering = 0.0f;
    }

    // Bornage physique strict des consignes
    target_speed = std::clamp(target_speed, -100.0f, 100.0f);
    target_steering = std::clamp(target_steering, -45.0f, 45.0f);

    if (m_driver) {
        // Continuous time-scaled slew-rate limiter (~175 deg/s straight, ~300 deg/s curve)
        float max_steer_rate = (std::abs(target_steering) < 15.0f) ? 175.0f : 300.0f;
        float max_steer_step = max_steer_rate * dt;
        float delta = std::clamp(target_steering - m_applied_steering, -max_steer_step, max_steer_step);
        m_applied_steering += delta;
        m_applied_speed = target_speed;

        m_driver->set_speed(m_applied_speed);
        m_driver->set_steering(m_applied_steering);
    } else {
        m_applied_steering = target_steering;
        m_applied_speed = target_speed;
    }
}

void MotorActuator::emergency_stop() {
    m_applied_speed = 0.0f;
    m_applied_steering = 0.0f;
    if (m_driver) {
        m_driver->emergency_stop();
    }
}

void MotorActuator::set_steering_offset(float offset_deg) {
    m_steering_offset = std::clamp(offset_deg, -15.0f, 15.0f);
    if (m_driver) {
        m_driver->set_steering_offset(m_steering_offset);
    }
}

} // namespace services
