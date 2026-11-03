#pragma once

#include <atomic>
#include <memory>
#include <string>

// Shared headers from libs
#include "header/header.hpp"
#include "services/services_telemetry.hpp"
#include "motors/motors_component.hpp"
#include "hal/motor_driver.hpp"
#include "ipc/shared_memory.hpp"
#include "shared-services/memory/subscriber.hpp"
#include "shared-services/memory/server.hpp"

namespace services {

class Motors {
public:
    explicit Motors(int hz = 50);
    ~Motors();

    bool init();
    void run();
    void stop();

    bool is_running() const { return m_running.load(); }

private:
    std::atomic<bool> m_running{false};
    int m_hz{50};

    std::unique_ptr<IMotorDriver> m_driver;
    std::unique_ptr<SharedMemoryManager> m_shm_mgr;
    RobotTelemetry* m_telemetry{nullptr};

    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_mem_server;
};

} // namespace services
