#pragma once

#include <atomic>
#include <memory>
#include <string>

// Shared headers from libs
#include "header/header.hpp"
#include "services/services_telemetry.hpp"
#include "sensors/sensors_telemetry.hpp"
#include "filters/ultrasonic_pipeline.hpp"
#include "hal/sensor_driver.hpp"
#include "ipc/shared_memory.hpp"
#include "shared-services/memory/server.hpp"

namespace services {

class Sensors {
public:
    explicit Sensors(int hz = 50, int trig = 12, int echo = 16);
    ~Sensors();

    bool init();
    void run();
    void stop();

    bool is_running() const { return m_running.load(); }

    static float compute_line_error(const uint8_t sensors[5], float fallback_error = 0.0f);
    static float compute_line_error_analog(const float darkness[5], float fallback_error = 0.0f);

private:
    std::atomic<bool> m_running{false};
    int m_hz{50};
    int m_trig_pin{12};
    int m_echo_pin{16};

    std::unique_ptr<ISensorDriver> m_driver;
    std::unique_ptr<UltrasonicPipeline> m_pipeline;
    std::unique_ptr<SharedMemoryManager> m_shm_mgr;
    RobotTelemetry* m_telemetry{nullptr};
    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_mem_server;

    float m_last_line_error{0.0f};
};

} // namespace services
