#pragma once

#include <memory>
#include <cstdint>
#include "ipc/shared_memory.hpp"
#include "sensors/sensors_telemetry.hpp"
#include "shared-services/memory/server.hpp"

namespace services {

class SensorShmBridge {
public:
    SensorShmBridge() = default;
    ~SensorShmBridge() = default;

    bool init();
    float read_current_speed();
    void update_legacy_telemetry(float filtered_dist, float line_error,
                                 const uint8_t line_sensors[5], const float line_darkness[5]);
    void publish_telemetry(float filtered_dist, const float line_darkness[5],
                           bool has_line, int consecutive_us_errors);

private:
    std::unique_ptr<SharedMemoryManager> m_shm_mgr;
    RobotTelemetry* m_telemetry{nullptr};
    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_mem_server;
};

} // namespace services
