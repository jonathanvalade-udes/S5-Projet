#pragma once

#include <memory>
#include <cstdint>
#include "sensors/sensors_telemetry.hpp"
#include "motors/motors_component.hpp"
#include "shared-services/memory/server.hpp"
#include "shared-services/memory/subscriber.hpp"

namespace services {

class SensorShmBridge {
public:
    SensorShmBridge() = default;
    ~SensorShmBridge() = default;

    bool init();
    float read_current_speed();
    void publish_telemetry(float filtered_dist, const float line_darkness[5],
                           const uint32_t raw_adc[5],
                           bool has_line, int consecutive_us_errors);

private:
    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_mem_server;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_motors_sub;
};

} // namespace services
