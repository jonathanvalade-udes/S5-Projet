#pragma once

#include <string>
#include <functional>
#include "shared-services/config/service_config.hpp"
#include "shared-services/socket/service_ipc_socket.hpp"

namespace services {

class FsmIpcHandler {
public:
    explicit FsmIpcHandler(const std::string& config_file = "fsm.conf",
                           const std::string& socket_path = "/tmp/fsm.sock");
    ~FsmIpcHandler() = default;

    bool init(std::function<bool(const std::string&, float)> on_command);
    void process_pending_commands();

    float get_float(const std::string& key, float default_val) const;
    void set_float(const std::string& key, float val);
    const std::string& get_active_filepath() const { return m_config.get_active_filepath(); }

private:
    std::string m_config_file;
    std::string m_socket_path;
    shared::services::config::ServiceConfig m_config;
    shared::services::ipc::ServiceIPCSocketServer m_ipc_server;
};

} // namespace services
