#include "Motors.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

namespace services {

Motors::Motors(int hz, bool invert, float steering_offset, bool offset_specified)
    : m_running(false),
      m_hz(hz > 0 ? hz : 100),
      m_invert(invert),
      m_steering_offset(steering_offset),
      m_offset_specified(offset_specified),
      m_actuator(invert, steering_offset),
      m_ipc_handler("motors.conf", "/tmp/motors.sock") {}

Motors::~Motors() {
    stop();
}

bool Motors::init() {
    std::cout << "[Motors] Initializing Actuator driver (Propulsion + Direction) at " << m_hz << " Hz..." << std::endl;

    if (!m_ipc_handler.init(
        [this](const std::string& key, float val) { return apply_config_key(key, val); },
        [this](uint32_t reason) {
            std::cerr << "[Motors] Emergency Stop received via IPC socket! Reason: " << reason << std::endl;
            m_actuator.emergency_stop();
            return true;
        }
    )) {
        std::cerr << "[Motors] Failed to initialize IPC handler!" << std::endl;
        return false;
    }

    if (!m_offset_specified) {
        m_steering_offset = m_ipc_handler.get_float("STEERING_OFFSET", picar::i2c::calibration::DEFAULT_STEERING_OFFSET_DEG);
        m_actuator.set_steering_offset(m_steering_offset);
    }
    m_hz = std::clamp(static_cast<int>(m_ipc_handler.get_float("HZ", static_cast<float>(m_hz))), 10, 10000);
    m_invert = (m_ipc_handler.get_float("INVERT", m_invert ? 1.0f : 0.0f) > 0.5f);

    if (!m_shm_bridge.init()) {
        std::cerr << "[Motors] Failed to initialize SHM bridge!" << std::endl;
        return false;
    }

    if (!m_actuator.init()) {
        std::cerr << "[Motors] Failed to initialize Actuator!" << std::endl;
        return false;
    }

    m_running.store(true);
    return true;
}

void Motors::run() {
    const auto period = std::chrono::nanoseconds(1000000000LL / m_hz);
    auto next_deadline = std::chrono::steady_clock::now();
    const float dt = 1.0f / static_cast<float>(m_hz);
    uint64_t loop_count = 0;
    const int max_watchdog_misses = (m_hz * 250) / 1000; // 250 ms timeout

    std::cout << "[Motors] Control loop started at " << m_hz << " Hz (Watchdog timeout: 250ms)." << std::endl;

    while (m_running.load(std::memory_order_relaxed)) {
        next_deadline += period;

        // 1. Non-blocking IPC commands (/tmp/motors.sock)
        m_ipc_handler.process_pending_commands();

        // 2. Read FSM commands from shared memory
        float target_speed = 0.0f;
        float target_steering = 0.0f;
        bool emergency_stop = false;
        uint64_t current_heartbeat = 0;

        m_shm_bridge.read_fsm_command(target_speed, target_steering, emergency_stop, current_heartbeat);

        // 3. Watchdog check
        if (m_shm_bridge.check_watchdog(current_heartbeat, max_watchdog_misses)) {
            target_speed = 0.0f; // Automatic safety cutout on lost FSM heartbeat
        }

        // 4. Actuate motor & servo with continuous time-scaled slew-rate limiter
        if (emergency_stop) {
            m_actuator.emergency_stop();
        } else {
            m_actuator.update_actuation(target_speed, target_steering, dt, false);
        }

        // 5. Zero-copy publish to pub/sub SHM
        shared::motors::MotorsConfig cfg{
            m_steering_offset,
            static_cast<float>(m_hz),
            m_invert ? 1.0f : 0.0f
        };
        m_shm_bridge.publish_telemetry(target_speed, target_steering, cfg);

        // 6. 1 Hz status output
        loop_count++;
        if (loop_count % m_hz == 0) {
            std::cout << "[Motors " << m_hz << "Hz] Throttle: " << target_speed 
                      << "% | Steering: " << target_steering << " deg"
                      << " (offset: " << m_steering_offset << " deg)"
                      << " | Watchdog misses: " << m_shm_bridge.get_watchdog_misses() << std::endl;
        }

        // 7. Deterministic anti-drift deadline pacing
        auto now = std::chrono::steady_clock::now();
        if (now < next_deadline) {
            std::this_thread::sleep_until(next_deadline);
        } else {
            next_deadline = now;
        }
    }

    m_actuator.emergency_stop();
    std::cout << "[Motors] Service loop terminated." << std::endl;
}

void Motors::stop() {
    if (m_running.load()) {
        std::cout << "[Motors] Stopping service..." << std::endl;
        m_running.store(false);
        m_actuator.emergency_stop();
    }
}

void Motors::set_steering_offset(float offset_deg) {
    m_steering_offset = std::clamp(offset_deg, -15.0f, 15.0f);
    m_actuator.set_steering_offset(m_steering_offset);
    m_ipc_handler.set_float("STEERING_OFFSET", m_steering_offset);
}

float Motors::get_steering_offset() const {
    return m_actuator.get_steering_offset();
}

bool Motors::apply_config_key(const std::string& key, float value) {
    if (key == "STEERING_OFFSET" || key == "OFFSET") {
        set_steering_offset(value);
        return true;
    }
    if (key == "HZ") {
        m_hz = std::clamp(static_cast<int>(value), 10, 10000);
        m_ipc_handler.set_float("HZ", static_cast<float>(m_hz));
        return true;
    }
    if (key == "INVERT") {
        m_invert = (value > 0.5f);
        m_ipc_handler.set_float("INVERT", m_invert ? 1.0f : 0.0f);
        return true;
    }
    return false;
}

} // namespace services
