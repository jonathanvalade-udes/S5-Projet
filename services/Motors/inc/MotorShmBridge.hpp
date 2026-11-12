#pragma once

#include <memory>
#include <cstdint>
#include "fsm/fsm_command.hpp"
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
    bool read_fsm_command(float& speed, float& steer, bool& emergency, uint64_t& heartbeat);
    bool check_watchdog(uint64_t current_heartbeat, int max_misses);
    void publish_telemetry(float target_speed, float target_steering, const shared::motors::MotorsConfig& config = {});

    int get_watchdog_misses() const { return m_watchdog_starvation_cycles; }

private:
    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_mem_server;
    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_servo_mem_server;
    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_fsm_cmd_sub;

    uint64_t m_last_heartbeat{0};
    int m_watchdog_starvation_cycles{0};
};

} // namespace services
