#pragma once

#include <memory>
#include <string>
#include <cstdint>
#include "filters/ultrasonic_pipeline.hpp"
#include "utils/simple_json.hpp"
#include "sensors/sensors_telemetry.hpp"
#include "fsm/fsm_command.hpp"
#include "motors/motors_component.hpp"
#include "shared-services/memory/server.hpp"
#include "shared-services/memory/subscriber.hpp"

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
    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_sensor_pub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_motor_pub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_fsm_cmd_sub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_sensor_sub;

    UltrasonicPipeline m_us_pipeline{5, 0.25f};
    float m_last_speed{0.0f};
};

} // namespace services
