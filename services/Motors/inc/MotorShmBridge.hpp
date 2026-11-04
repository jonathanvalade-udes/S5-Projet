#pragma once

#include <memory>
#include <cstdint>
#include "ipc/shared_memory.hpp"
#include "motors/motors_component.hpp"
#include "servo/servo_telemetry.hpp"
#include "shared-services/memory/server.hpp"
#include "shared-services/memory/subscriber.hpp"

namespace services {

class MotorShmBridge {
public:
    MotorShmBridge() = default;
    ~MotorShmBridge() = default;

    bool init();
    bool read_fsm_command(float& speed, float& steer, float& shm_offset, bool& emergency, uint64_t& heartbeat);
    bool check_watchdog(uint64_t current_heartbeat, int max_misses);
    void update_telemetry_speed(float speed);
    void update_telemetry_failsafe();
    void publish_telemetry(float target_speed, float target_steering);

    int get_watchdog_misses() const { return m_watchdog_starvation_cycles; }

private:
    std::unique_ptr<SharedMemoryManager> m_shm_mgr;
    RobotTelemetry* m_telemetry{nullptr};

    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_mem_server;
    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_servo_mem_server;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_fsm_sub;

    uint64_t m_last_heartbeat{0};
    int m_watchdog_starvation_cycles{0};
};

} // namespace services
