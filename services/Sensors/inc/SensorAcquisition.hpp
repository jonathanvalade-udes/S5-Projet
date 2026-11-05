#pragma once

#include <memory>
#include <cstdint>
#include "filters/ultrasonic_pipeline.hpp"
#include "hal/sensor_driver.hpp"
#include "hal/i2c_device.hpp"
#include "hal/picar_i2c_registers.hpp"

namespace services {

class SensorAcquisition {
public:
    static constexpr int DEFAULT_TRIG_PIN = 12;
    static constexpr int DEFAULT_ECHO_PIN = 16;

    explicit SensorAcquisition(int trig_pin = DEFAULT_TRIG_PIN, int echo_pin = DEFAULT_ECHO_PIN, 
                               float filter_alpha = 0.25f, size_t median_window = 5);
    ~SensorAcquisition() = default;

    bool init();
    void reset_pipeline(float initial_dist = 100.0f);

    void read_line(float darkness[5], uint8_t binarized[5], uint32_t raw_adc[5], bool& has_line, uint64_t loop_tick = 0, int hz = 100);
    float acquire_distance(uint64_t loop_tick, int hz, float dt, float current_speed);

    int get_consecutive_us_errors() const { return m_consecutive_us_errors; }
    bool is_hardware_present() const { return m_driver != nullptr; }

    void update_filter_params(float alpha, size_t window);

private:
    int m_trig_pin{DEFAULT_TRIG_PIN};
    int m_echo_pin{DEFAULT_ECHO_PIN};
    float m_filter_alpha{0.25f};
    size_t m_median_window{5};

    std::unique_ptr<ISensorDriver> m_driver;
    std::unique_ptr<UltrasonicPipeline> m_pipeline;
    std::unique_ptr<picar::hal::I2CDevice> m_line_device;
    picar::i2c::line_follower::CalibrationData m_calibration;
    uint16_t m_thresholds[5]{picar::i2c::line_follower::DEFAULT_THRESHOLD,
                            picar::i2c::line_follower::DEFAULT_THRESHOLD,
                            picar::i2c::line_follower::DEFAULT_THRESHOLD,
                            picar::i2c::line_follower::DEFAULT_THRESHOLD,
                            picar::i2c::line_follower::DEFAULT_THRESHOLD};

    float m_last_raw_dist{100.0f};
    float m_last_filtered_dist{100.0f};
    float m_cached_darkness[5]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t m_cached_binarized[5]{0, 0, 0, 0, 0};
    uint32_t m_cached_raw_adc[5]{0, 0, 0, 0, 0};
    int m_consecutive_us_errors{0};
};

} // namespace services
