#pragma once

#include <memory>
#include <string>
#include <cstdint>
#include "ipc/shared_memory.hpp"
#include "filters/ultrasonic_pipeline.hpp"
#include "utils/simple_json.hpp"

namespace services {

class SimShmBridge {
public:
    SimShmBridge();
    ~SimShmBridge() = default;

    bool init();
    void reset_pipeline(float dist = 100.0f);

    void inject_sim_telemetry(const GodotTelemetryIn& telem);
    void read_control_targets(float& target_spd, float& target_steer, bool& emergency, float& cruise_cfg);
    void read_vehicle_sensors(float& dist, uint8_t line_sensors[5]);

    void set_emergency_stop(bool emergency, float target_spd = 0.0f);
    void apply_config_update(const std::string& key, float val);
    void forward_start_to_fifo();

private:
    std::unique_ptr<SharedMemoryManager> m_shm_mgr;
    RobotTelemetry* m_telemetry{nullptr};
    UltrasonicPipeline m_us_pipeline{5, 0.25f};
};

} // namespace services
