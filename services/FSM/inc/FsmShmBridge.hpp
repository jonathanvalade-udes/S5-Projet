#pragma once

#include <memory>
#include <cstdint>
#include "fsm/machine_component.hpp"
#include "fsm/fsm_command.hpp"
#include "sensors/sensors_telemetry.hpp"
#include "shared-services/memory/subscriber.hpp"
#include "shared-services/memory/server.hpp"

namespace services {

class FsmShmBridge {
public:
    FsmShmBridge() = default;
    ~FsmShmBridge() = default;

    bool init(shared::fsm::MachineStates initial_state);

    bool read_sensors(float& dist, uint8_t line_sensors[5], float line_darkness[5],
                      bool& emergency, bool& sensors_alive);
    bool check_watchdog(int max_cycles);
    void reset_watchdog();

    void publish_control_commands(float speed_pct, float steering_deg, bool emergency,
                                 shared::fsm::MachineStates state, float steering_offset = 0.0f);
    void publish_fsm_telemetry(shared::fsm::MachineStates state);

    void emergency_stop_shm();

private:
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_sensor_sub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_cmd_pub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_fsm_pub;

    uint32_t m_fsm_heartbeat{0};
    uint32_t m_last_sensor_timestamp{0};
    int m_heartbeat_starvation_cycles{0};
};

} // namespace services
