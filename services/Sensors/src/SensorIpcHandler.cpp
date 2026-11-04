#include "SensorIpcHandler.hpp"
#include <iostream>

namespace services {

SensorIpcHandler::SensorIpcHandler(const std::string& config_file, const std::string& socket_path)
    : m_config_file(config_file), m_socket_path(socket_path), m_config(config_file) {}

bool SensorIpcHandler::init(std::function<bool(const std::string&, float)> on_command) {
    if (m_config.load(m_config_file)) {
        std::cout << "[SensorIpcHandler] Configuration loaded from " << m_config.get_active_filepath() << std::endl;
    } else {
        std::cout << "[SensorIpcHandler] No existing " << m_config_file << " found, using defaults." << std::endl;
    }

    if (!m_ipc_server.init(m_socket_path, shared::commands::ServiceID::SENSORS, on_command)) {
        std::cerr << "[SensorIpcHandler] Warning: Failed to init IPC socket at " << m_socket_path << std::endl;
    } else {
        std::cout << "[SensorIpcHandler] IPC Server listening on " << m_socket_path << std::endl;
    }

    return true;
}

void SensorIpcHandler::process_pending_commands() {
    m_ipc_server.process_pending_commands();
}

float SensorIpcHandler::get_float(const std::string& key, float default_val) const {
    return m_config.get_float(key, default_val);
}

void SensorIpcHandler::set_float(const std::string& key, float val) {
    m_config.set_value(key, val);
    m_config.save_key(key, val);
}

} // namespace services
