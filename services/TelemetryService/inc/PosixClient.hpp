#pragma once

#include <string>
#include <unordered_map>

namespace services {

/**
 * @brief Manages non-blocking POSIX IPC connections to services.
 * Supports routing packets/commands to specific service sockets (e.g. /tmp/fsm.sock,
 * /tmp/motors.sock, /tmp/sensors.sock) with automatic fallback to /tmp/peripherals.sock.
 */
class PosixClient {
public:
    explicit PosixClient(std::string default_socket_path = "/tmp/peripherals.sock");
    ~PosixClient();

    bool connect(const std::string& path = "");
    void disconnect(const std::string& path = "");

    bool send_packet(const void* data, size_t len, const std::string& target_path = "");
    bool send_command(const std::string& cmd, const std::string& target_path = "");
    void poll_and_drain();

    const std::string& get_socket_path() const { return m_default_socket_path; }

private:
    int get_or_connect(const std::string& path);

    std::string m_default_socket_path;
    std::unordered_map<std::string, int> m_connections;
};

} // namespace services
