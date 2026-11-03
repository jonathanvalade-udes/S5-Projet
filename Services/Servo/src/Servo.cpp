#include "Servo.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

namespace services {

Servo::Servo(int hz) : m_running(false), m_hz(hz) {}

Servo::~Servo() {
    stop();
}

bool Servo::init() {
    std::cout << "[Servo] Initializing Steering Servo service at " << m_hz << " Hz..." << std::endl;

    try {
        m_shm_mgr = std::make_unique<SharedMemoryManager>();
        m_telemetry = m_shm_mgr->open_shm(true);
    } catch (const std::exception& e) {
        std::cerr << "[Servo] Warning: Failed to open POSIX SHM: " << e.what() << std::endl;
        m_telemetry = nullptr;
    }

    try {
        m_mem_server = std::make_unique<shared::services::memory::MemoryShareComponentServer>("Servo");
    } catch (const std::exception& e) {
        std::cerr << "[Servo] Warning: MemoryShareComponentServer: " << e.what() << std::endl;
    }

    m_running.store(true);
    return true;
}

void Servo::run() {
    const auto period_ms = std::chrono::milliseconds(1000 / m_hz);
    uint64_t loop_count = 0;

    std::cout << "[Servo] Steering Servo monitor running." << std::endl;

    while (m_running.load()) {
        auto start_time = std::chrono::steady_clock::now();

        float target_steering = 0.0f;
        if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            target_steering = m_telemetry->target_steering;
            pthread_mutex_unlock(&(m_telemetry->mtx));
        }

        // Publish to Pub/Sub SHM
        if (m_mem_server && m_mem_server->isValid()) {
            shared::servos::ServoTelemetry telem{};
            telem.normalizedCurrentAngle = target_steering / shared::servos::MAX_ANGLE;
            telem.normalizedTargettedAngle = target_steering / shared::servos::MAX_ANGLE;
            telem.flags = shared::servos::flags::RDY;
            m_mem_server->publish(telem);
        }

        loop_count++;
        if (loop_count % m_hz == 0) {
            std::cout << "[Servo] Steering Angle: " << target_steering << " deg" << std::endl;
        }

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < period_ms) {
            std::this_thread::sleep_for(period_ms - elapsed);
        }
    }

    std::cout << "[Servo] Service loop terminated." << std::endl;
}

void Servo::stop() {
    if (m_running.load()) {
        std::cout << "[Servo] Stopping service..." << std::endl;
        m_running.store(false);
    }
}

} // namespace services
