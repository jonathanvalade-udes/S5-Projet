#pragma once

#include <string>
#include <functional>
#include "shared-services/config/service_config.hpp"
#include "shared-services/socket/service_ipc_socket.hpp"

namespace services {

class MotorIpcHandler {
public:
    explicit MotorIpcHandler(const std::string& config_file = "motors.conf",
                             const std::string& socket_path = "/tmp/motors.sock");
    ~MotorIpcHandler() = default;

    bool init(std::function<bool(const std::string&, float)> on_command);
    void process_pending_commands();

    float get_float(const std::string& key, float default_val) const;
    void set_float(const std::string& key, float val);

private:
    std::string m_config_file;
    std::string m_socket_path;
    shared::services::config::ServiceConfig m_config;
    shared::services::ipc::ServiceIPCSocketServer m_ipc_server;
};

} // namespace services
