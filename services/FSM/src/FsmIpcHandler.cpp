#include "FsmIpcHandler.hpp"
#include <iostream>

namespace services {

FsmIpcHandler::FsmIpcHandler(const std::string& config_file, const std::string& socket_path)
    : m_config_file(config_file), m_socket_path(socket_path), m_config(config_file) {}

bool FsmIpcHandler::init(std::function<bool(const std::string&, float)> on_command,
                        std::function<bool(bool)> on_active,
                        std::function<bool(uint32_t)> on_estop) {
    if (m_config.load(m_config_file)) {
        std::cout << "[FsmIpcHandler] Configuration loaded from " << m_config.get_active_filepath() << std::endl;
    } else {
        std::cout << "[FsmIpcHandler] No existing " << m_config_file << " found, using defaults." << std::endl;
    }

    if (!m_ipc_server.init(m_socket_path, shared::commands::ServiceID::FSM, on_command)) {
        std::cerr << "[FsmIpcHandler] Warning: Failed to init IPC socket at " << m_socket_path << std::endl;
    } else {
        std::cout << "[FsmIpcHandler] IPC Server listening on " << m_socket_path << std::endl;
    }

    if (on_active) {
        m_ipc_server.set_active_callback(std::move(on_active));
    }
    if (on_estop) {
        m_ipc_server.set_emergency_stop_callback(std::move(on_estop));
    }

    return true;
}

void FsmIpcHandler::process_pending_commands() {
    m_ipc_server.process_pending_commands();
}

float FsmIpcHandler::get_float(const std::string& key, float default_val) const {
    return m_config.get_float(key, default_val);
}

void FsmIpcHandler::set_float(const std::string& key, float val) {
    m_config.set_value(key, val);
    m_config.save_key(key, val);
}

} // namespace services
