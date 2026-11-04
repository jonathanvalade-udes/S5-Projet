#include "Sensors.hpp"
#include "control/picar_config.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

namespace services {

float Sensors::compute_line_error(const uint8_t sensors[5], float fallback_error) {
    constexpr float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
    float weighted_sum = 0.0f;
    int active_count = 0;

    for (int i = 0; i < 5; ++i) {
        if (sensors[i]) {
            weighted_sum += weights[i];
            active_count++;
        }
    }

    // Anti-décrochage : si aucun capteur ne voit la ligne, conserver la dernière erreur
    // au lieu de renvoyer 0.0f (qui ferait faussement croire que le robot est au centre !)
    if (active_count == 0) {
        return fallback_error;
    }
    return (weighted_sum / (2.0f * active_count));
}

Sensors::Sensors(int hz, int trig, int echo) 
    : m_running(false), m_hz(hz), m_trig_pin(trig), m_echo_pin(echo), m_last_line_error(0.0f) {}

Sensors::~Sensors() {
    stop();
}

bool Sensors::init() {
    std::cout << "[Sensors] Initializing service at " << m_hz << " Hz (Trig=" << m_trig_pin << ", Echo=" << m_echo_pin << ")..." << std::endl;

    m_driver = std::make_unique<PiCarHatsSensorDriver>(m_trig_pin, m_echo_pin);

    m_pipeline = std::make_unique<UltrasonicPipeline>(5, 0.25f);
    m_pipeline->reset(100.0f);

    try {
        m_shm_mgr = std::make_unique<SharedMemoryManager>();
        m_telemetry = m_shm_mgr->open_shm(true);
        if (m_telemetry) {
            auto initial_cfg = config::PicarConfig::read_config();
            config::PicarConfig::apply_to_shm(initial_cfg, m_telemetry);
        }
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
        static int consecutive_us_errors = 0;
        if (raw_dist < 0.0f) {
            consecutive_us_errors++;
        } else {
            consecutive_us_errors = 0;
        }

        // Lecture analogique (noirceur continue [0,1]) ; fallback strict sur 0.0 si aucun capteur
        float line_darkness[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        if (m_driver) {
            m_driver->read_line_darkness(line_darkness);
        }

        uint8_t line_sensors[5];
        bool has_line = false;
        for (int i = 0; i < 5; ++i) {
            line_sensors[i] = (line_darkness[i] > 0.5f) ? 1 : 0;
            if (line_sensors[i] != 0) has_line = true;
        }

        float current_speed = 0.0f;
        if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            current_speed = m_telemetry->speed;
            pthread_mutex_unlock(&(m_telemetry->mtx));
        }

        float filtered_dist = m_pipeline ? m_pipeline->process(raw_dist, dt, current_speed) : raw_dist;
        m_last_line_error = compute_line_error(line_sensors, m_last_line_error);
        float line_error = m_last_line_error;

        // Update legacy / fast SHM
        if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            m_telemetry->filtered_dist = filtered_dist;
            m_telemetry->line_error = line_error;
            for (int i = 0; i < 5; ++i) {
                m_telemetry->line_sensors[i] = line_sensors[i];
                m_telemetry->line_darkness[i] = line_darkness[i];
            }
            m_telemetry->heartbeat++;
            pthread_mutex_unlock(&(m_telemetry->mtx));
        }

        // Update decoupled Pub/Sub SHM
        if (m_mem_server && m_mem_server->isValid()) {
            shared::sensors::SensorsTelemetry telem{};
            telem.ultrasonicSensor.rawDistanceCm = raw_dist;
            telem.ultrasonicSensor.calculatedDistance = filtered_dist;
            for (int i = 0; i < 5; ++i) {
                telem.lineFollower.sensorsData[i].rawADCValue = static_cast<uint32_t>(line_darkness[i] * 1023.0f);
                telem.lineFollower.sensorsData[i].normalizedValue = line_darkness[i]; // noirceur [0,1]
            }
            telem.lineFollower.flags = has_line ? shared::sensors::lineFollower::flags::LINE_FOLLOWED
                                                : shared::sensors::lineFollower::flags::LINE_LOST;
            telem.flags = shared::sensors::flags::DISCOVERABLE;
            if (consecutive_us_errors >= 5) {
                telem.flags |= shared::sensors::flags::DRIVER_ERROR;
                telem.ultrasonicSensor.flags = shared::sensors::ultrasonic::flags::ANOMALIES_DETECTED;
            }
            if (!has_line) {
                telem.flags |= shared::sensors::flags::WARNING_DETECTED;
            }
            m_mem_server->publish(telem);
        }

        loop_count++;
        if (loop_count % m_hz == 0) {
            std::cout << "[Sensors] Distance: " << filtered_dist 
                      << " cm | Line: [" 
                      << (int)line_sensors[0] << " "
                      << (int)line_sensors[1] << " "
                      << (int)line_sensors[2] << " "
                      << (int)line_sensors[3] << " "
                      << (int)line_sensors[4] << "] dark: ["
                      << line_darkness[0] << " " << line_darkness[1] << " " << line_darkness[2] << " "
                      << line_darkness[3] << " " << line_darkness[4] << "] (err: " << line_error << ")"
                      << (has_line ? "" : " [PERTE DE LIGNE]") << std::endl;
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
