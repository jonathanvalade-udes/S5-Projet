#include "Sensors.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

namespace services {

float Sensors::compute_line_error(const uint8_t sensors[5]) {
    constexpr float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
    float weighted_sum = 0.0f;
    int active_count = 0;

    for (int i = 0; i < 5; ++i) {
        if (sensors[i]) {
            weighted_sum += weights[i];
            active_count++;
        }
    }

    if (active_count == 0) {
        return 0.0f;
    }
    return (weighted_sum / (2.0f * active_count));
}

Sensors::Sensors(int hz) : m_running(false), m_hz(hz) {}

Sensors::~Sensors() {
    stop();
}

bool Sensors::init() {
    std::cout << "[Sensors] Initializing service at " << m_hz << " Hz..." << std::endl;

#ifdef __linux__
    m_driver = std::make_unique<PiCarHatsSensorDriver>();
#else
    m_driver = std::make_unique<MockSensorDriver>();
#endif

    m_pipeline = std::make_unique<UltrasonicPipeline>(5, 0.25f);
    m_pipeline->reset(100.0f);

    try {
        m_shm_mgr = std::make_unique<SharedMemoryManager>();
        m_telemetry = m_shm_mgr->open_shm(true);
    } catch (const std::exception& e) {
        std::cerr << "[Sensors] Warning: Failed to open POSIX SHM: " << e.what() << std::endl;
        m_telemetry = nullptr;
    }

    try {
        m_mem_server = std::make_unique<shared::services::memory::MemoryShareComponentServer>("Sensors");
    } catch (const std::exception& e) {
        std::cerr << "[Sensors] Warning: Failed to init MemoryShareComponentServer: " << e.what() << std::endl;
    }

    m_running.store(true);
    return true;
}

void Sensors::run() {
    const auto period_ms = std::chrono::milliseconds(1000 / m_hz);
    const float dt = 1.0f / static_cast<float>(m_hz);
    uint64_t loop_count = 0;

    std::cout << "[Sensors] Service running at " << m_hz << " Hz." << std::endl;

    while (m_running.load()) {
        auto start_time = std::chrono::steady_clock::now();

        float raw_dist = m_driver ? m_driver->read_raw_ultrasonic_cm() : 100.0f;
        uint8_t line_sensors[5] = {0, 0, 1, 0, 0};
        if (m_driver) {
            m_driver->read_line_sensors(line_sensors);
        }

        float current_speed = 0.0f;
        if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            current_speed = m_telemetry->speed;
            pthread_mutex_unlock(&(m_telemetry->mtx));
        }

        float filtered_dist = m_pipeline ? m_pipeline->process(raw_dist, dt, current_speed) : raw_dist;
        float line_error = compute_line_error(line_sensors);

        // Update legacy / fast SHM
        if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            m_telemetry->filtered_dist = filtered_dist;
            m_telemetry->line_error = line_error;
            for (int i = 0; i < 5; ++i) {
                m_telemetry->line_sensors[i] = line_sensors[i];
            }
            m_telemetry->heartbeat++;
            pthread_mutex_unlock(&(m_telemetry->mtx));
        }

        // Update decoupled Pub/Sub SHM
        if (m_mem_server && m_mem_server->isValid()) {
            shared::sensors::SensorsTelemetry telem{};
            telem.ultrasonicSensor.calculatedDistance = filtered_dist;
            for (int i = 0; i < 5; ++i) {
                telem.lineFollower.sensorsData[i].rawADCValue = line_sensors[i] ? 1023 : 0;
                telem.lineFollower.sensorsData[i].normalizedValue = line_sensors[i] ? 1.0f : 0.0f;
            }
            telem.flags = shared::sensors::flags::DISCOVERABLE;
            m_mem_server->publish(telem);
        }

        loop_count++;
        if (loop_count % m_hz == 0) {
            std::cout << "[Sensors] Distance: " << filtered_dist 
                      << " cm (raw: " << raw_dist 
                      << " cm) | Line error: " << line_error << std::endl;
        }

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < period_ms) {
            std::this_thread::sleep_for(period_ms - elapsed);
        }
    }

    std::cout << "[Sensors] Service loop terminated." << std::endl;
}

void Sensors::stop() {
    if (m_running.load()) {
        std::cout << "[Sensors] Stopping service..." << std::endl;
        m_running.store(false);
    }
}

} // namespace services
