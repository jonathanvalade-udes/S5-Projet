#include "SensorAcquisition.hpp"
#include <iostream>

namespace services {

SensorAcquisition::SensorAcquisition(int trig_pin, int echo_pin, float filter_alpha, size_t median_window)
    : m_trig_pin(trig_pin),
      m_echo_pin(echo_pin),
      m_filter_alpha(filter_alpha),
      m_median_window(median_window) {}

bool SensorAcquisition::init() {
    try {
        m_driver = std::make_unique<PiCarHatsSensorDriver>(m_trig_pin, m_echo_pin);
        std::cout << "[SensorAcquisition] Hardware driver initialized (Trig=" << m_trig_pin << ", Echo=" << m_echo_pin << ")." << std::endl;
    } catch (const std::exception& e) {
        std::cout << "[SensorAcquisition] Note: Hardware sensor driver fallback (" << e.what() << ")." << std::endl;
        m_driver.reset();
    }

    m_pipeline = std::make_unique<UltrasonicPipeline>(m_median_window, m_filter_alpha);
    reset_pipeline(100.0f);
    return true;
}

void SensorAcquisition::reset_pipeline(float initial_dist) {
    if (m_pipeline) {
        m_pipeline->reset(initial_dist);
    }
    m_last_raw_dist = initial_dist;
    m_last_filtered_dist = initial_dist;
}

void SensorAcquisition::read_line(float darkness[5], uint8_t binarized[5], bool& has_line, uint64_t loop_tick, int hz) {
    // Rate-isolate physical I2C read at ~100 Hz to prevent I2C bus congestion and timeouts
    const uint64_t line_interval = (hz > 100) ? static_cast<uint64_t>(hz / 100) : 1;
    if (loop_tick % line_interval == 0 && m_driver) {
        m_driver->read_line_darkness(m_cached_darkness);
    }

    has_line = false;
    for (int i = 0; i < 5; ++i) {
        darkness[i] = m_cached_darkness[i];
        binarized[i] = (m_cached_darkness[i] > 0.5f) ? 1 : 0;
        if (binarized[i] != 0) has_line = true;
    }
}

float SensorAcquisition::acquire_distance(uint64_t loop_tick, int hz, float dt, float current_speed) {
    if (!m_driver) {
        return m_last_filtered_dist;
    }

    // Rate-isolate physical ultrasonic trigger/echo at ~50 Hz to prevent 5ms thread stalls
    const uint64_t us_interval = (hz > 50) ? static_cast<uint64_t>(hz / 50) : 1;
    if (loop_tick % us_interval == 0) {
        float raw = m_driver->read_raw_ultrasonic_cm();
        if (raw < 0.0f) {
            m_consecutive_us_errors++;
        } else {
            m_consecutive_us_errors = 0;
            m_last_raw_dist = raw;
        }

        if (m_pipeline) {
            m_last_filtered_dist = m_pipeline->process(m_last_raw_dist, dt * static_cast<float>(us_interval), current_speed);
        } else {
            m_last_filtered_dist = m_last_raw_dist;
        }
    }

    return m_last_filtered_dist;
}

void SensorAcquisition::update_filter_params(float alpha, size_t window) {
    m_filter_alpha = alpha;
    m_median_window = window;
    m_pipeline = std::make_unique<UltrasonicPipeline>(m_median_window, m_filter_alpha);
    reset_pipeline(m_last_filtered_dist);
}

} // namespace services
