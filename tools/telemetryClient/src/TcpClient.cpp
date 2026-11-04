#include "TcpClient.hpp"

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>

namespace telemetry {

TcpClient::TcpClient() = default;

TcpClient::~TcpClient() {
    disconnect();
}

bool TcpClient::connect_to(const std::string& host, int port) {
    disconnect();

    m_host = host;
    m_port = port;

    struct addrinfo hints{};
    hints.ai_family = AF_INET; // IPv4
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    struct addrinfo* result = nullptr;
    std::string port_str = std::to_string(port);
    int s = getaddrinfo(host.c_str(), port_str.c_str(), &hints, &result);
    if (s != 0 || result == nullptr) {
        std::cerr << "[TcpClient] Échec de résolution d'adresse pour " << host << ":" << port 
                  << " (" << gai_strerror(s) << ")" << std::endl;
        return false;
    }

    int sock = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (sock < 0) {
        std::cerr << "[TcpClient] Impossible de créer le socket TCP." << std::endl;
        freeaddrinfo(result);
        return false;
    }

    // Options socket : TCP_NODELAY pour faible latence
    int nodelay = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

#if defined(SO_NOSIGPIPE)
    int nosig = 1;
    setsockopt(sock, SOL_SOCKET, SO_NOSIGPIPE, &nosig, sizeof(nosig));
#endif

    // Rendre le socket non-bloquant pour gérer un timeout de connexion précis
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    int conn_res = connect(sock, result->ai_addr, result->ai_addrlen);
    freeaddrinfo(result);

    if (conn_res < 0) {
        if (errno != EINPROGRESS) {
            close(sock);
            return false;
        }

        // Attente de connexion avec timeout (3 secondes)
        struct pollfd pfd{};
        pfd.fd = sock;
        pfd.events = POLLOUT;
        int poll_res = poll(&pfd, 1, 3000);
        if (poll_res <= 0) {
            close(sock);
            return false;
        }

        int so_error = 0;
        socklen_t len = sizeof(so_error);
        getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len);
        if (so_error != 0) {
            close(sock);
            return false;
        }
    }

    // Remettre en mode bloquant standard
    fcntl(sock, F_SETFL, flags);

    m_socket_fd = sock;
    m_connected.store(true);
    std::cout << "[TcpClient] Connecté avec succès à " << m_host << ":" << m_port << std::endl;
    return true;
}

void TcpClient::disconnect() {
    if (m_connected.load() || m_socket_fd >= 0) {
        if (m_socket_fd >= 0) {
            close(m_socket_fd);
            m_socket_fd = -1;
        }
        m_connected.store(false);
        std::cout << "[TcpClient] Déconnecté de " << m_host << ":" << m_port << std::endl;
    }
}

bool TcpClient::send_bytes(const uint8_t* data, size_t length) {
    if (!m_connected.load() || m_socket_fd < 0 || data == nullptr || length == 0) {
        return false;
    }

    size_t total_sent = 0;
    while (total_sent < length) {
        int flags = 0;
#if defined(MSG_NOSIGNAL)
        flags |= MSG_NOSIGNAL;
#endif
        ssize_t sent = send(m_socket_fd, data + total_sent, length - total_sent, flags);
        if (sent <= 0) {
            std::cerr << "[TcpClient] Erreur d'envoi TCP. Fermeture de connexion." << std::endl;
            disconnect();
            return false;
        }
        total_sent += sent;
    }
    return true;
}

bool TcpClient::receive_exact(uint8_t* buffer, size_t length, int timeout_ms) {
    if (!m_connected.load() || m_socket_fd < 0 || buffer == nullptr || length == 0) {
        return false;
    }

    size_t total_received = 0;
    while (total_received < length) {
        struct pollfd pfd{};
        pfd.fd = m_socket_fd;
        pfd.events = POLLIN;

        int poll_res = poll(&pfd, 1, timeout_ms);
        if (poll_res <= 0) {
            return false; // Timeout ou interruption
        }

        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            std::cerr << "[TcpClient] Événement d'erreur ou déconnexion sur le socket." << std::endl;
            disconnect();
            return false;
        }

        ssize_t rec = recv(m_socket_fd, buffer + total_received, length - total_received, 0);
        if (rec <= 0) {
            std::cerr << "[TcpClient] Connexion interrompue par le serveur." << std::endl;
            disconnect();
            return false;
        }
        total_received += rec;
    }

    return true;
}

bool TcpClient::has_data_available(int timeout_ms) {
    if (!m_connected.load() || m_socket_fd < 0) {
        return false;
    }

    struct pollfd pfd{};
    pfd.fd = m_socket_fd;
    pfd.events = POLLIN;

    int poll_res = poll(&pfd, 1, timeout_ms);
    if (poll_res > 0 && (pfd.revents & POLLIN)) {
        return true;
    }
    if (poll_res > 0 && (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
        disconnect();
    }
    return false;
}

} // namespace telemetry
