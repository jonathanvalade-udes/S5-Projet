#pragma once

#include <atomic>
#include <string>
#include "MotorActuator.hpp"
#include "MotorIpcHandler.hpp"
#include "MotorShmBridge.hpp"

namespace services {

class Motors {
public:
    explicit Motors(int hz = 100, bool invert = false, 
                    float steering_offset = picar::i2c::calibration::DEFAULT_STEERING_OFFSET_DEG, 
                    bool offset_specified = false);
    ~Motors();

    bool init();
    void run();
    void stop();

    bool is_running() const { return m_running.load(); }
    void set_steering_offset(float offset_deg);
    float get_steering_offset() const;

private:
    std::atomic<bool> m_running{false};
    int m_hz{100};
    bool m_invert{false};
    float m_steering_offset{0.0f};
    bool m_offset_specified{false};

    MotorActuator m_actuator;
    MotorIpcHandler m_ipc_handler;
    MotorShmBridge m_shm_bridge;

    bool apply_config_key(const std::string& key, float value);
};

} // namespace services
