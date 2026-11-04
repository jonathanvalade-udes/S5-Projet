#pragma once

#include <memory>
#include <cstdint>
#include "ipc/shared_memory.hpp"
#include "fsm/machine_component.hpp"
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

    void publish_control_commands(float speed_pct, float steering_deg, bool emergency,
                                 shared::fsm::MachineStates state);
    void publish_fsm_telemetry(shared::fsm::MachineStates state);

    void sync_config_to_shm(float max_accel, float cruise_spd, float min_curve_spd,
                            float safety_factor, float brake_gain, float kp, float kd);
    bool check_config_from_shm(float& max_accel, float& cruise_spd, float& min_curve_spd,
                               float& safety_factor, float& brake_gain, float& kp, float& kd);

    void emergency_stop_shm();

private:
    std::unique_ptr<SharedMemoryManager> m_shm_mgr;
    RobotTelemetry* m_telemetry{nullptr};

    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_sensor_sub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_fsm_pub;

    uint32_t m_current_cfg_version{0};
    uint64_t m_last_heartbeat{0};
    int m_heartbeat_starvation_cycles{0};
};

} // namespace services
