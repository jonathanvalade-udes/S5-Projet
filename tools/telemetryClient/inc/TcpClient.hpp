#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <atomic>

namespace telemetry {

class TcpClient {
public:
    TcpClient();
    ~TcpClient();

    // Connexion au serveur TCP
    bool connect_to(const std::string& host, int port);

    // Déconnexion
    void disconnect();

    // État de connexion
    bool is_connected() const { return m_connected.load(); }

    // Envoi de données binaires
    bool send_bytes(const uint8_t* data, size_t length);

    // Réception exacte d'un nombre d'octets avec timeout
    bool receive_exact(uint8_t* buffer, size_t length, int timeout_ms = 1000);

    // Vérifie si des données sont prêtes à être lues sans bloquer
    bool has_data_available(int timeout_ms = 0);

    // Getters pour les infos hôte
    std::string get_host() const { return m_host; }
    int get_port() const { return m_port; }

private:
    std::atomic<bool> m_connected{false};
    int m_socket_fd{-1};
    std::string m_host;
    int m_port{0};
};

} // namespace telemetry
