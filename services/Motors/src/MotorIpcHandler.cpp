#include "MotorIpcHandler.hpp"
#include <iostream>

namespace services {

MotorIpcHandler::MotorIpcHandler(const std::string& config_file, const std::string& socket_path)
    : m_config_file(config_file), m_socket_path(socket_path), m_config(config_file) {}

bool MotorIpcHandler::init(std::function<bool(const std::string&, float)> on_command) {
    if (!m_config.load(m_config_file)) {
        std::cout << "[MotorIpcHandler] No existing " << m_config_file << " found, using defaults." << std::endl;
    }

    if (!m_ipc_server.init(m_socket_path, shared::commands::ServiceID::MOTORS, on_command)) {
        std::cerr << "[MotorIpcHandler] Warning: Failed to init IPC socket at " << m_socket_path << std::endl;
    } else {
        std::cout << "[MotorIpcHandler] IPC Server listening on " << m_socket_path << std::endl;
    }

    return true;
}

void MotorIpcHandler::process_pending_commands() {
    m_ipc_server.process_pending_commands();
}

float MotorIpcHandler::get_float(const std::string& key, float default_val) const {
    return m_config.get_float(key, default_val);
}

void MotorIpcHandler::set_float(const std::string& key, float val) {
    m_config.set_value(key, val);
    m_config.save_key(key, val);
}

} // namespace services
