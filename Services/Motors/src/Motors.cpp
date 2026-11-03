#include "Motors.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

namespace services {

Motors::Motors(int hz) : m_running(false), m_hz(hz) {}

Motors::~Motors() {
    stop();
}

bool Motors::init() {
    std::cout << "[Motors] Initializing Actuator driver (Propulsion + Direction) at " << m_hz << " Hz..." << std::endl;

#ifdef __linux__
    m_driver = std::make_unique<PiCarHatsMotorDriver>();
#else
    m_driver = std::make_unique<MockMotorDriver>(false);
#endif

    m_driver->emergency_stop();

    try {
        m_shm_mgr = std::make_unique<SharedMemoryManager>();
        m_telemetry = m_shm_mgr->open_shm(true);
    } catch (const std::exception& e) {
        std::cerr << "[Motors] Warning: Failed to open POSIX SHM: " << e.what() << std::endl;
        m_telemetry = nullptr;
    }

    try {
        m_mem_server = std::make_unique<shared::services::memory::MemoryShareComponentServer>("Motors");
    } catch (const std::exception& e) {
        std::cerr << "[Motors] Warning: MemoryShareComponentServer (Motors): " << e.what() << std::endl;
    }

    try {
        m_servo_mem_server = std::make_unique<shared::services::memory::MemoryShareComponentServer>("Servo");
    } catch (const std::exception& e) {
        std::cerr << "[Motors] Warning: MemoryShareComponentServer (Servo): " << e.what() << std::endl;
    }

    try {
        m_fsm_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("FSM");
    } catch (const std::exception& e) {
        std::cerr << "[Motors] Warning: MemoryShareComponentSubscriber (FSM): " << e.what() << std::endl;
    }

    m_running.store(true);
    return true;
}

void Motors::run() {
    const auto period_ms = std::chrono::milliseconds(1000 / m_hz);
    uint64_t loop_count = 0;
    uint64_t last_heartbeat = 0;
    int watchdog_starvation_cycles = 0;
    const int max_watchdog_misses = (m_hz * 250) / 1000; // 250 ms timeout

    std::cout << "[Motors] Motor & Steering control loop started (Watchdog timeout: 250ms)." << std::endl;

    while (m_running.load()) {
        auto start_time = std::chrono::steady_clock::now();

        float target_speed = 0.0f;
        float target_steering = 0.0f;
        bool emergency_stop_requested = false;
        uint64_t current_heartbeat = 0;

        if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            target_speed = m_telemetry->target_speed;
            target_steering = m_telemetry->target_steering;
            emergency_stop_requested = m_telemetry->emergency_stop;
            current_heartbeat = m_telemetry->heartbeat;
            pthread_mutex_unlock(&(m_telemetry->mtx));
        }

        // Watchdog check
        if (current_heartbeat == last_heartbeat) {
            watchdog_starvation_cycles++;
        } else {
            watchdog_starvation_cycles = 0;
            last_heartbeat = current_heartbeat;
        }

        if (watchdog_starvation_cycles > max_watchdog_misses) {
            target_speed = 0.0f; // Coupure de sécurité automatique
        }

        if (emergency_stop_requested) {
            if (m_driver) m_driver->emergency_stop();
            if (m_telemetry) {
                pthread_mutex_lock(&(m_telemetry->mtx));
                m_telemetry->speed = 0.0f;
                m_telemetry->state = 99; // FAILSAFE
                pthread_mutex_unlock(&(m_telemetry->mtx));
            }
        } else {
            if (m_driver) {
                m_driver->set_speed(target_speed);
                m_driver->set_steering(target_steering);
            }
            if (m_telemetry) {
                pthread_mutex_lock(&(m_telemetry->mtx));
                m_telemetry->speed = target_speed;
                pthread_mutex_unlock(&(m_telemetry->mtx));
            }
        }

        // 1. Publier la télémétrie Motors (avec direction intégrée)
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

        // 2. Publier également sur le canal Servo pour assurer la rétrocompatibilité complète
        if (m_servo_mem_server && m_servo_mem_server->isValid()) {
            shared::servos::ServoTelemetry servo_telem{};
            servo_telem.normalizedCurrentAngle = target_steering / shared::servos::MAX_ANGLE;
            servo_telem.normalizedTargettedAngle = target_steering / shared::servos::MAX_ANGLE;
            servo_telem.flags = shared::servos::flags::RDY;
            m_servo_mem_server->publish(servo_telem);
        }

        loop_count++;
        if (loop_count % m_hz == 0) {
            std::cout << "[Motors] Throttle: " << target_speed 
                      << "% | Steering: " << target_steering << " deg"
                      << " | Watchdog misses: " << watchdog_starvation_cycles << std::endl;
        }

        // Réception et synchronisation temps N sans délai artificiel
        bool woke_by_shm = false;
        if (m_fsm_sub && m_fsm_sub->isConnected()) {
            woke_by_shm = m_fsm_sub->waitForUpdate(period_ms.count());
        }

        if (!woke_by_shm) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed < period_ms) {
                std::this_thread::sleep_for(period_ms - elapsed);
            }
        }
    }

    // Hardware safety on shutdown
    std::cout << "[Motors] Failsafe: Stopping all actuators..." << std::endl;
    if (m_driver) m_driver->emergency_stop();

    if (m_telemetry) {
        pthread_mutex_lock(&(m_telemetry->mtx));
        m_telemetry->speed = 0.0f;
        m_telemetry->target_speed = 0.0f;
        m_telemetry->state = 0; // STOP
        pthread_mutex_unlock(&(m_telemetry->mtx));
    }

    std::cout << "[Motors] Service loop terminated." << std::endl;
}

void Motors::stop() {
    if (m_running.load()) {
        std::cout << "[Motors] Stopping service..." << std::endl;
        m_running.store(false);
    }
}

} // namespace services
