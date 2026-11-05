#include "Sensors.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

namespace services {

Sensors::Sensors(int hz, int /*trig*/, int /*echo*/) 
    : m_running(false),
      m_hz(hz > 0 ? hz : 100),
      m_filter_alpha(0.25f),
      m_median_window(5),
      m_last_line_error(0.0f),
      m_acquisition(TRIG_PIN, ECHO_PIN, 0.25f, 5),
      m_ipc_handler("sensors.conf", "/tmp/sensors.sock") {}

Sensors::~Sensors() {
    stop();
}

bool Sensors::init() {
    std::cout << "[Sensors] Initializing service at " << m_hz << " Hz (Trig=" << TRIG_PIN << ", Echo=" << ECHO_PIN << ")..." << std::endl;

    if (!m_ipc_handler.init([this](const std::string& key, float val) {
        return apply_config_key(key, val);
    })) {
        std::cerr << "[Sensors] Failed to initialize IPC handler!" << std::endl;
        return false;
    }

    m_hz = std::clamp(static_cast<int>(m_ipc_handler.get_float("HZ", static_cast<float>(m_hz))), 10, 10000);
    m_filter_alpha = m_ipc_handler.get_float("FILTER_ALPHA", m_filter_alpha);
    m_median_window = static_cast<size_t>(m_ipc_handler.get_float("MEDIAN_WINDOW", static_cast<float>(m_median_window)));

    if(!m_acquisition.init()){
        std::cerr << "[SENSORS] Failed to initialize the acquisition" << std::endl;
        return false;
    }
    
    m_acquisition.update_filter_params(m_filter_alpha, m_median_window);

    if (!m_shm_bridge.init()) {
        std::cerr << "[Sensors] Failed to initialize SHM bridge!" << std::endl;
        return false;
    }

    m_running.store(true);
    return true;
}

void Sensors::run() {
    const auto period = std::chrono::nanoseconds(1000000000LL / m_hz);
    auto next_deadline = std::chrono::steady_clock::now();
    const float dt = 1.0f / static_cast<float>(m_hz);
    uint64_t loop_count = 0;

    std::cout << "[Sensors] Service running at " << m_hz << " Hz." << std::endl;

    while (m_running.load(std::memory_order_relaxed)) {
        next_deadline += period;

        // 1. Process non-blocking IPC commands from /tmp/sensors.sock
        m_ipc_handler.process_pending_commands();

        // 2. Read line sensors (analog darkness continuous [0,1], rate-isolated)
        float line_darkness[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        uint8_t line_sensors[5] = {0, 0, 0, 0, 0};
        bool has_line = false;
        m_acquisition.read_line(line_darkness, line_sensors, has_line, loop_count, m_hz);

        // 3. Acquire distance with synchronous rate-isolation
        float current_speed = m_shm_bridge.read_current_speed();
        float filtered_dist = m_acquisition.acquire_distance(loop_count, m_hz, dt, current_speed);

        // 4. Compute line centroid error
        m_last_line_error = LineEstimator::compute_line_error_analog(line_darkness, m_last_line_error);
        float line_error = m_last_line_error;

        // 5. Publish to pub/sub SHM
        m_shm_bridge.publish_telemetry(filtered_dist, line_darkness, has_line, m_acquisition.get_consecutive_us_errors());

        // 7. 1 Hz periodic log
        loop_count++;
        if (loop_count % m_hz == 0) {
            std::cout << "[Sensors " << m_hz << "Hz] Distance: " << filtered_dist 
                      << " cm | Line: [" 
                      << (int)line_sensors[0] << " " << (int)line_sensors[1] << " "
                      << (int)line_sensors[2] << " " << (int)line_sensors[3] << " "
                      << (int)line_sensors[4] << "] (err: " << line_error << ")"
                      << (has_line ? "" : " [PERTE DE LIGNE]") << std::endl;
        }

        // 8. Deterministic anti-drift deadline pacing
        auto now = std::chrono::steady_clock::now();
        if (now < next_deadline) {
            std::this_thread::sleep_until(next_deadline);
        } else {
            next_deadline = now;
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

bool Sensors::apply_config_key(const std::string& key, float value) {
    if (key == "HZ") {
        m_hz = std::clamp(static_cast<int>(value), 10, 10000);
        m_ipc_handler.set_float("HZ", static_cast<float>(m_hz));
        return true;
    }
    if (key == "FILTER_ALPHA") {
        m_filter_alpha = std::clamp(value, 0.01f, 1.0f);
        m_acquisition.update_filter_params(m_filter_alpha, m_median_window);
        m_ipc_handler.set_float("FILTER_ALPHA", m_filter_alpha);
        return true;
    }
    if (key == "MEDIAN_WINDOW") {
        m_median_window = std::clamp(static_cast<size_t>(value), static_cast<size_t>(1), static_cast<size_t>(15));
        m_acquisition.update_filter_params(m_filter_alpha, m_median_window);
        m_ipc_handler.set_float("MEDIAN_WINDOW", static_cast<float>(m_median_window));
        return true;
    }
    return false;
}

} // namespace services
