#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "ipc/shared_memory.hpp"
#include "utils/simple_json.hpp"

namespace services {

class SimBridge {
public:
    explicit SimBridge(int port = 9000);
    ~SimBridge();

    bool init();
    void run();
    void stop();

    bool is_running() const { return m_running.load(); }

private:
    std::atomic<bool> m_running{false};
    int m_port{9000};
    int m_server_fd{-1};

    std::unique_ptr<SharedMemoryManager> m_shm_mgr;
    RobotTelemetry* m_telemetry{nullptr};
};

} // namespace services
