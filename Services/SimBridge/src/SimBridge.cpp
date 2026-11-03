#include "SimBridge.hpp"
#include <iostream>
#include <cstring>
#include <vector>
#include <chrono>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>

namespace services {

SimBridge::SimBridge(int port) : m_running(false), m_port(port) {}

SimBridge::~SimBridge() {
    stop();
}

bool SimBridge::init() {
    std::cout << "[SimBridge] Initializing TCP Simulator Bridge on port " << m_port << "..." << std::endl;

    try {
        m_shm_mgr = std::make_unique<SharedMemoryManager>();
        m_telemetry = m_shm_mgr->open_shm(true);
    } catch (const std::exception& e) {
        std::cerr << "[SimBridge] Failed to open SHM: " << e.what() << std::endl;
        return false;
    }

    m_server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_server_fd < 0) {
        std::cerr << "[SimBridge] Socket creation failed: " << strerror(errno) << std::endl;
        return false;
    }

    int opt = 1;
    setsockopt(m_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(m_port);

    if (bind(m_server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "[SimBridge] Bind failed on port " << m_port << ": " << strerror(errno) << std::endl;
        close(m_server_fd);
        m_server_fd = -1;
        return false;
    }

    if (listen(m_server_fd, 1) < 0) {
        std::cerr << "[SimBridge] Listen failed: " << strerror(errno) << std::endl;
        close(m_server_fd);
        m_server_fd = -1;
        return false;
    }

    int flags = fcntl(m_server_fd, F_GETFL, 0);
    fcntl(m_server_fd, F_SETFL, flags | O_NONBLOCK);

    m_running.store(true);
    return true;
}

void SimBridge::run() {
    std::cout << "[SimBridge] Waiting for Godot / Webots simulator connections..." << std::endl;

    int client_fd = -1;
    std::string rx_buffer;
    rx_buffer.reserve(4096);
    uint64_t packet_count = 0;

    while (m_running.load()) {
        if (client_fd < 0) {
            sockaddr_in client_addr{};
            socklen_t addr_len = sizeof(client_addr);
            client_fd = accept(m_server_fd, (struct sockaddr*)&client_addr, &addr_len);

            if (client_fd < 0) {
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    continue;
                }
                std::cerr << "[SimBridge] Accept error: " << strerror(errno) << std::endl;
                break;
            }

            // Socket client setup
            fcntl(client_fd, F_SETFL, 0);
            struct timeval tv{};
            tv.tv_sec = 0;
            tv.tv_usec = 200000;
            setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

            int nodelay = 1;
            setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, (char*)&nodelay, sizeof(nodelay));

#ifdef SO_NOSIGPIPE
            int nosigpipe = 1;
            setsockopt(client_fd, SOL_SOCKET, SO_NOSIGPIPE, &nosigpipe, sizeof(nosigpipe));
#endif

            std::cout << "[SimBridge] Simulator connected from " 
                      << inet_ntoa(client_addr.sin_addr) << ":" << ntohs(client_addr.sin_port) << std::endl;
            rx_buffer.clear();
        }

        char recv_buf[1024];
        ssize_t bytes_read = recv(client_fd, recv_buf, sizeof(recv_buf) - 1, 0);

        if (bytes_read < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            std::cout << "[SimBridge] Socket read error / timeout: " << strerror(errno) << std::endl;
            close(client_fd);
            client_fd = -1;
            continue;
        }

        if (bytes_read == 0) {
            std::cout << "[SimBridge] Simulator disconnected (EOF)." << std::endl;
            close(client_fd);
            client_fd = -1;
            continue;
        }

        recv_buf[bytes_read] = '\0';
        rx_buffer.append(recv_buf, bytes_read);

        size_t newline_pos = std::string::npos;
        while ((newline_pos = rx_buffer.find('\n')) != std::string::npos) {
            std::string line = rx_buffer.substr(0, newline_pos);
            rx_buffer.erase(0, newline_pos + 1);

            if (line.empty()) continue;

            packet_count++;

            GodotTelemetryIn telem_in;
            if (SimpleJson::parse_godot_telemetry(line, telem_in)) {
                if (m_telemetry) {
                    pthread_mutex_lock(&(m_telemetry->mtx));
                    m_telemetry->filtered_dist = telem_in.distance;
                    for (int i = 0; i < 5; ++i) {
                        m_telemetry->line_sensors[i] = telem_in.line_sensors[i];
                    }
                    if (telem_in.ball_dropped) {
                        m_telemetry->emergency_stop = true;
                    }
                    m_telemetry->heartbeat++;

                    float target_spd = m_telemetry->target_speed;
                    float target_steer = m_telemetry->target_steering;
                    bool emergency = m_telemetry->emergency_stop;
                    pthread_mutex_unlock(&(m_telemetry->mtx));

                    GodotCommandOut cmd_out;
                    cmd_out.velocity = (target_spd / 50.0f) * 0.25f;
                    cmd_out.direction = target_steer / 45.0f;
                    cmd_out.brake = emergency;

                    std::string response = SimpleJson::serialize_command(cmd_out);
                    ssize_t sent = send(client_fd, response.c_str(), response.size(), 0);
                    if (sent < 0) {
                        std::cerr << "[SimBridge] Send failed: " << strerror(errno) << std::endl;
                        close(client_fd);
                        client_fd = -1;
                        break;
                    }

                    if (packet_count % 50 == 0) {
                        std::cout << "[SimBridge 50Hz] Packets: " << packet_count
                                  << " | Dist: " << telem_in.distance << " cm"
                                  << " | Vel: " << cmd_out.velocity << " m/s"
                                  << " | Steer: " << cmd_out.direction << std::endl;
                    }
                }
            }
        }
    }

    if (client_fd >= 0) close(client_fd);
    if (m_server_fd >= 0) close(m_server_fd);
    m_server_fd = -1;

    std::cout << "[SimBridge] Service stopped." << std::endl;
}

void SimBridge::stop() {
    if (m_running.load()) {
        m_running.store(false);
    }
}

} // namespace services
