#pragma once

#include <memory>
#include <algorithm>
#include <cmath>
#include "hal/motor_driver.hpp"

namespace services {

class MotorActuator {
public:
    explicit MotorActuator(bool invert = false, 
                           float steering_offset = picar::i2c::calibration::DEFAULT_STEERING_OFFSET_DEG);
    ~MotorActuator() = default;

    bool init();
    void update_actuation(float target_speed, float target_steering, float dt, bool emergency_stop);
    void emergency_stop();

    void set_steering_offset(float offset_deg);
    float get_steering_offset() const { return m_steering_offset; }
    float get_applied_steering() const { return m_applied_steering; }
    bool is_hardware_present() const { return m_driver != nullptr; }

private:
    bool m_invert{false};
    float m_steering_offset{0.0f};
    float m_applied_steering{0.0f};
    std::unique_ptr<IMotorDriver> m_driver;
};

} // namespace services
