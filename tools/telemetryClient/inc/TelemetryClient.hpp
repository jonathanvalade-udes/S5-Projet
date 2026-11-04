#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <queue>

#include "TcpClient.hpp"
#include "ProtocolHandler.hpp"
#include "WebServer.hpp"

namespace telemetry {

struct TelemetryClientConfig {
    std::string robot_ip{"127.0.0.1"};
    int robot_port{91};
    int web_port{8080};
    bool enable_web{true};
    std::string web_root{"web"};
};

class TelemetryClient {
public:
    explicit TelemetryClient(const TelemetryClientConfig& config);
    ~TelemetryClient();

    bool init();
    void run();
    void stop();

    // Envoie une commande (asynchrone ou immédiat)
    void queue_command(const std::string& command_str);

private:
    void tcp_worker_loop();
    void process_pending_commands();

    TelemetryClientConfig m_config;
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_robot_connected{false};

    TcpClient m_tcp;
    ProtocolHandler m_proto;
    WebServer m_web;

    std::thread m_tcp_thread;

    std::mutex m_cmd_mutex;
    std::queue<std::string> m_pending_commands;
    uint16_t m_cmd_seq{100};
};

} // namespace telemetry
