#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "ipc/shared_memory.hpp"
#include "utils/simple_json.hpp"

namespace services {

class SimBridge {
public:
    explicit SimBridge(int port = 9000, bool sim_mode = true);
    ~SimBridge();

    bool init();
    void run();
    void stop();

    bool is_running() const { return m_running.load(); }
    bool is_sim_mode() const { return m_sim_mode; }
    void set_sim_mode(bool sim_mode) { m_sim_mode = sim_mode; }

private:
    std::atomic<bool> m_running{false};
    int m_port{9000};
    bool m_sim_mode{true}; // true = Godot simulation mode, false = vehicle monitoring mode
    int m_server_fd{-1};

    std::unique_ptr<SharedMemoryManager> m_shm_mgr;
    RobotTelemetry* m_telemetry{nullptr};
};

} // namespace services
