#pragma once

#include <string>
#include <functional>
#include <vector>
#include <cstdint>

// Shared headers (lecture seule, non modifiés)
#include "header/header.hpp"
#include "commands/commands_header.hpp"
#include "commands/global_commands.hpp"
#include "commands/set_configs.hpp"
#include "responses/global_responses.hpp"
#include "responses/nack.hpp"

namespace shared::services::ipc {

/**
 * @brief Serveur Socket IPC AF_UNIX non-bloquant pour la réception des commandes de configuration.
 */
class ServiceIPCSocketServer {
public:
    using ConfigCallback = std::function<bool(const std::string& key, float value)>;

    ServiceIPCSocketServer() = default;
    ~ServiceIPCSocketServer();

    bool init(const std::string& socket_path, shared::commands::ServiceID service_id, ConfigCallback callback);
    void process_pending_commands();
    void stop();

    const std::string& get_socket_path() const { return m_socket_path; }
    bool is_listening() const { return m_server_fd >= 0; }

private:
    std::string m_socket_path;
    shared::commands::ServiceID m_service_id{shared::commands::ServiceID::FSM};
    ConfigCallback m_callback;
    int m_server_fd{-1};

    void handle_client(int client_fd);
    void send_ack(int client_fd, uint16_t seqNumber);
    void send_nack(int client_fd, uint16_t seqNumber, uint16_t nackCode = 1);
};

/**
 * @brief Client Socket IPC AF_UNIX pour l'envoi de commandes de configuration.
 */
class ServiceIPCSocketClient {
public:
    /**
     * @brief Envoie une commande SetConfigKey vers un service via son socket UNIX et attend la réponse ACK/NACK.
     * @return true si ACK reçu, false si NACK ou erreur de communication.
     */
    static bool send_set_config(
        const std::string& socket_path,
        shared::commands::ServiceID service_id,
        const std::string& key,
        float value,
        uint16_t seqNumber = 0,
        int timeout_ms = 500
    );
};

} // namespace shared::services::ipc

