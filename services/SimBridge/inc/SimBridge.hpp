#pragma once

#include <atomic>
#include <string>
#include "SimTcpServer.hpp"
#include "SimProtocolHandler.hpp"
#include "SimShmBridge.hpp"

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
    bool m_sim_mode{true};

    SimTcpServer m_tcp_server;
    SimShmBridge m_shm_bridge;
};

} // namespace services
