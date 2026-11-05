#pragma once

#include <string>
#include <functional>
#include <vector>
#include <cstdint>

// Shared headers
#include "header/header.hpp"
#include "commands/commands_header.hpp"
#include "commands/global_commands.hpp"
#include "commands/set_configs.hpp"
#include "commands/set_active.hpp"
#include "commands/emergency_stop.hpp"
#include "responses/global_responses.hpp"
#include "responses/nack.hpp"

namespace shared::services::ipc {

/**
 * @brief Serveur Socket IPC AF_UNIX non-bloquant pour la réception des commandes discrètes (SetConfigKey, SetActive, EmergencyStop).
 */
class ServiceIPCSocketServer {
public:
    using ConfigCallback = std::function<bool(const std::string& key, float value)>;
    using SetActiveCallback = std::function<bool(bool active)>;
    using EmergencyStopCallback = std::function<bool(uint32_t reason_code)>;

    ServiceIPCSocketServer() = default;
    ~ServiceIPCSocketServer();

    bool init(const std::string& socket_path, shared::commands::ServiceID service_id, ConfigCallback callback = nullptr);
    void set_config_callback(ConfigCallback callback) { m_config_callback = std::move(callback); }
    void set_active_callback(SetActiveCallback callback) { m_active_callback = std::move(callback); }
    void set_emergency_stop_callback(EmergencyStopCallback callback) { m_emergency_callback = std::move(callback); }

    void process_pending_commands();
    void stop();

    const std::string& get_socket_path() const { return m_socket_path; }
    bool is_listening() const { return m_server_fd >= 0; }

private:
    std::string m_socket_path;
    shared::commands::ServiceID m_service_id{shared::commands::ServiceID::FSM};
    ConfigCallback m_config_callback;
    SetActiveCallback m_active_callback;
    EmergencyStopCallback m_emergency_callback;
    int m_server_fd{-1};

    void handle_client(int client_fd);
    void send_ack(int client_fd, uint16_t seqNumber);
    void send_nack(int client_fd, uint16_t seqNumber, uint16_t nackCode = 1);
};

/**
 * @brief Client Socket IPC AF_UNIX pour l'envoi de commandes discrètes asynchrones (SetConfigKey, SetActive, EmergencyStop).
 */
class ServiceIPCSocketClient {
public:
    /**
     * @brief Envoie une commande SetConfigKey vers un service via son socket UNIX et attend la réponse ACK/NACK.
     */
    static bool send_set_config(
        const std::string& socket_path,
        shared::commands::ServiceID service_id,
        const std::string& key,
        float value,
        uint16_t seqNumber = 0,
        int timeout_ms = 500
    );

    /**
     * @brief Envoie une commande SetActive vers un service via son socket UNIX et attend la réponse ACK/NACK.
     */
    static bool send_set_active(
        const std::string& socket_path,
        shared::commands::ServiceID service_id,
        bool active,
        uint16_t seqNumber = 0,
        int timeout_ms = 500
    );

    /**
     * @brief Envoie une commande EmergencyStop vers un service via son socket UNIX et attend la réponse ACK/NACK.
     */
    static bool send_emergency_stop(
        const std::string& socket_path,
        shared::commands::ServiceID service_id,
        uint32_t reason_code = 0,
        uint16_t seqNumber = 0,
        int timeout_ms = 500
    );

private:
    static bool wait_for_ack(int fd, int timeout_ms);
};

} // namespace shared::services::ipc
