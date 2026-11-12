#include "SensorAcquisition.hpp"
#include <iostream>

namespace services {

SensorAcquisition::SensorAcquisition(int trig_pin, int echo_pin, float filter_alpha, size_t median_window)
    : m_trig_pin(trig_pin),
      m_echo_pin(echo_pin),
      m_filter_alpha(filter_alpha),
      m_median_window(median_window) {}

bool SensorAcquisition::init(bool require_hardware) {
    const char* env_req = std::getenv("PICAR_REQUIRE_HARDWARE");
    if (env_req && (std::string(env_req) == "1" || std::string(env_req) == "true")) {
        require_hardware = true;
    }

    try {
        m_driver = std::make_unique<PiCarHatsSensorDriver>(m_trig_pin, m_echo_pin);
        std::cout << "[SensorAcquisition] Hardware driver initialized (Trig=" << m_trig_pin << ", Echo=" << m_echo_pin << ")." << std::endl;
        if (auto* picar_driver = dynamic_cast<PiCarHatsSensorDriver*>(m_driver.get())) {
            m_calibration = picar_driver->get_calibration();
            for (int i = 0; i < 5; ++i) {
                m_thresholds[i] = static_cast<uint16_t>((m_calibration.black_adc[i] + m_calibration.white_adc[i]) * 0.5f);
            }
        }
    } catch (const std::exception& e) {
        std::cout << "[SensorAcquisition] Note: Hardware sensor driver fallback (" << e.what() << ")." << std::endl;
        m_driver.reset();
        if (require_hardware) {
            std::cerr << "[SensorAcquisition] ERROR: Hardware sensor driver required but failed to initialize!" << std::endl;
            return false;
        }
    }

    try {
        m_line_device = std::make_unique<picar::hal::I2CDevice>(
            picar::i2c::ADDR_LINE_FOLLOWER, picar::i2c::ADDR_LINE_FOLLOWER_ALT);
        if (m_line_device->is_connected()) {
            std::cout << "[SensorAcquisition] Suiveur de ligne I2C prêt sur 0x"
                      << std::hex << static_cast<int>(m_line_device->get_active_address()) << std::dec << std::endl;
        } else {
            std::cout << "[SensorAcquisition] Suiveur de ligne I2C non détecté, mode fallback actif." << std::endl;
            if (require_hardware && !m_driver) {
                std::cerr << "[SensorAcquisition] ERROR: Hardware I2C line follower required but not detected!" << std::endl;
                return false;
            }
        }
    } catch (const std::exception& e) {
        std::cout << "[SensorAcquisition] I2C exception (" << e.what() << "), fallback actif." << std::endl;
        m_line_device.reset();
        if (require_hardware) {
            std::cerr << "[SensorAcquisition] ERROR: Hardware I2C line follower exception (" << e.what() << ")!" << std::endl;
            return false;
        }
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

void SensorAcquisition::read_line(float darkness[5], uint8_t binarized[5], uint32_t raw_adc[5], bool& has_line, uint64_t loop_tick, int hz) {
    // Rate-isolate physical I2C read at ~100 Hz to prevent I2C bus congestion and timeouts
    const uint64_t line_interval = (hz > 100) ? static_cast<uint64_t>(hz / 100) : 1;
    if (loop_tick % line_interval == 0) {
        if (m_line_device && m_line_device->is_connected()) {
            uint8_t buffer[picar::i2c::line_follower::RAW_BUFFER_SIZE] = {0};
            if (m_line_device->read_block_data(picar::i2c::line_follower::REG_START, buffer, picar::i2c::line_follower::RAW_BUFFER_SIZE)) {
                uint16_t adc[5] = {0};
                picar::i2c::line_follower::parse_raw_adc(buffer, adc);
                for (int i = 0; i < 5; ++i) {
                    m_cached_raw_adc[i] = adc[i];
                }
                picar::i2c::line_follower::binarize_sensors(adc, m_thresholds, m_cached_binarized);
                picar::i2c::line_follower::convert_raw_to_darkness(adc, m_calibration, m_cached_darkness);
            }
        } else if (m_driver) {
            m_driver->read_line_darkness(m_cached_darkness);
            for (int i = 0; i < 5; ++i) {
                m_cached_binarized[i] = (m_cached_darkness[i] > 0.5f) ? 1 : 0;
                float span = m_calibration.white_adc[i] - m_calibration.black_adc[i];
                float estimated_adc = m_calibration.white_adc[i] - m_cached_darkness[i] * span;
                m_cached_raw_adc[i] = static_cast<uint32_t>(std::clamp(estimated_adc, 0.0f, 1023.0f));
            }
        }
    }

    has_line = false;
    for (int i = 0; i < 5; ++i) {
        darkness[i] = m_cached_darkness[i];
        binarized[i] = m_cached_binarized[i];
        raw_adc[i] = m_cached_raw_adc[i];
        if (binarized[i] != 0) has_line = true;
    }
}

float SensorAcquisition::acquire_distance(uint64_t loop_tick, int hz, float dt, float current_speed) {
    if (!m_driver) {
        return m_last_filtered_dist;
    }

    if (dt <= 0.0f) {
        dt = (hz > 0) ? (1.0f / static_cast<float>(hz)) : 0.02f;
    } else if (dt > 0.5f) {
        dt = 0.5f;
    }

    // Rate-isolate physical ultrasonic trigger/echo at ~50 Hz to prevent 5ms thread stalls
    const uint64_t us_interval = (hz > 50) ? static_cast<uint64_t>(hz / 50) : 1;
    if (loop_tick % us_interval == 0) {
        float raw = m_driver->read_raw_ultrasonic_cm();
        if (raw < 0.0f || raw > 400.0f || !std::isfinite(raw)) {
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
