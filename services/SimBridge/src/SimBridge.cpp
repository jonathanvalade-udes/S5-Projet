#include "SimBridge.hpp"
#include "control/picar_config.hpp"
#include <iostream>
#include <cstring>
#include <vector>
#include <chrono>
#include <thread>
#include <algorithm>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>

namespace services {

SimBridge::SimBridge(int port, bool sim_mode) 
    : m_running(false), m_port(port), m_sim_mode(sim_mode) {}

SimBridge::~SimBridge() {
    stop();
}

bool SimBridge::init() {
    std::cout << "[SimBridge] Initializing TCP Bridge on port " << m_port 
              << " (" << (m_sim_mode ? "SIMULATION MODE (Godot)" : "VEHICLE MONITOR MODE (Real car)") << ")..." << std::endl;

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
    std::cout << "[SimBridge] Waiting for TCP connections on port " << m_port << "..." << std::endl;

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

            std::cout << "[SimBridge] Client connected from " 
                      << inet_ntoa(client_addr.sin_addr) << ":" << ntohs(client_addr.sin_port) << std::endl;
            rx_buffer.clear();
            m_us_pipeline.reset(100.0f);
        }

        char recv_buf[1024];
        ssize_t bytes_read = recv(client_fd, recv_buf, sizeof(recv_buf) - 1, 0);

        if (bytes_read <= 0) {
            if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
                continue;
            }
            if (bytes_read == 0) {
                std::cout << "[SimBridge] Client disconnected (EOF)." << std::endl;
            } else {
                std::cout << "[SimBridge] Socket read error / timeout: " << strerror(errno) << std::endl;
            }
            close(client_fd);
            client_fd = -1;

            // Arrêt sécuritaire immédiat dès déconnexion
            if (m_telemetry) {
                pthread_mutex_lock(&(m_telemetry->mtx));
                m_telemetry->target_speed = 0.0f;
                m_telemetry->emergency_stop = true;
                pthread_mutex_unlock(&(m_telemetry->mtx));
            }
            continue;
        }

        recv_buf[bytes_read] = '\0';
        rx_buffer.append(recv_buf, bytes_read);

        size_t newline_pos = std::string::npos;
        while ((newline_pos = rx_buffer.find('\n')) != std::string::npos) {
            std::string line = rx_buffer.substr(0, newline_pos);
            rx_buffer.erase(0, newline_pos + 1);

            if (line.empty()) continue;

            // Commande de déclenchement FSM : "trigger_start" ou "start"
            if (line == "trigger_start" || line == "start" || line.find("trigger_start") != std::string::npos) {
                int fifo_fd = open("/tmp/peripherals.sock", O_WRONLY | O_NONBLOCK);
                if (fifo_fd >= 0) {
                    const char* cmd = "trigger_start\n";
                    ssize_t w = write(fifo_fd, cmd, strlen(cmd));
                    (void)w;
                    close(fifo_fd);
                    std::cout << "[SimBridge] Transmis 'trigger_start' vers /tmp/peripherals.sock" << std::endl;
                } else {
                    std::cerr << "[SimBridge] Note: FIFO /tmp/peripherals.sock (" << strerror(errno) << ")" << std::endl;
                }
                if (m_telemetry) {
                    pthread_mutex_lock(&(m_telemetry->mtx));
                    m_telemetry->emergency_stop = false;
                    pthread_mutex_unlock(&(m_telemetry->mtx));
                }
                std::string ack = "{\"status\":\"ok\",\"event\":\"trigger_start\"}\n";
                send(client_fd, ack.c_str(), ack.size(), 0);
                continue;
            }

            // Commande d'arrêt d'urgence : "emergency_stop" ou "stop"
            if (line == "emergency_stop" || line == "stop" || line.find("emergency_stop") != std::string::npos) {
                if (m_telemetry) {
                    pthread_mutex_lock(&(m_telemetry->mtx));
                    m_telemetry->emergency_stop = true;
                    m_telemetry->target_speed = 0.0f;
                    pthread_mutex_unlock(&(m_telemetry->mtx));
                }
                std::cout << "[SimBridge] Arrêt d'urgence déclenché depuis Godot !" << std::endl;
                std::string ack = "{\"status\":\"ok\",\"event\":\"emergency_stop\"}\n";
                send(client_fd, ack.c_str(), ack.size(), 0);
                continue;
            }

            // Commande de modification de constante par WiFi (format "PARAM=valeur" ou "set_param")
            if (line.find('=') != std::string::npos && line.find("line_follower") == std::string::npos && line.find("distance") == std::string::npos) {
                size_t eq = line.find('=');
                std::string k = line.substr(0, eq);
                std::string v = line.substr(eq + 1);
                k.erase(std::remove_if(k.begin(), k.end(), ::isspace), k.end());
                v.erase(std::remove_if(v.begin(), v.end(), ::isspace), v.end());
                try {
                    float fval = std::stof(v);
                    // 1. Écriture persistante dans le fichier .config
                    config::PicarConfig::write_config(k, fval);
                    // 2. Application immédiate en mémoire POSIX
                    if (m_telemetry) {
                        pthread_mutex_lock(&(m_telemetry->mtx));
                        if (k == "MAX_ACCELERATION") m_telemetry->cfg_max_acceleration = fval;
                        else if (k == "STRAIGHT_CRUISE_SPEED" || k == "CRUISE_SPEED") m_telemetry->cfg_cruise_speed = fval;
                        else if (k == "MIN_CURVE_SPEED") m_telemetry->cfg_min_curve_speed = fval;
                        else if (k == "CURVE_SAFETY_FACTOR") m_telemetry->cfg_curve_safety_factor = fval;
                        else if (k == "PREDICTIVE_BRAKE_GAIN") m_telemetry->cfg_predictive_brake_gain = fval;
                        else if (k == "LINE_KP" || k == "KP") m_telemetry->cfg_line_kp = fval;
                        else if (k == "LINE_KD" || k == "KD") m_telemetry->cfg_line_kd = fval;
                        m_telemetry->cfg_version++;
                        pthread_mutex_unlock(&(m_telemetry->mtx));
                    }
                    std::cout << "[SimBridge WiFi] Paramètre mis à jour : " << k << " = " << fval << std::endl;
                    std::string ack = "{\"status\":\"ok\",\"param\":\"" + k + "\",\"value\":" + std::to_string(fval) + "}\n";
                    send(client_fd, ack.c_str(), ack.size(), 0);
                    continue;
                } catch (...) {}
            }

            packet_count++;

            GodotTelemetryIn telem_in;
            if (SimpleJson::parse_godot_telemetry(line, telem_in)) {
                if (m_telemetry) {
                    pthread_mutex_lock(&(m_telemetry->mtx));

                    if (m_sim_mode) {
                        // Mode simulation Godot : injection des capteurs virtuels dans la mémoire partagée
                        m_telemetry->filtered_dist = m_us_pipeline.process(telem_in.distance);
                        float total_darkness = 0.0f;
                        float weighted_darkness = 0.0f;
                        constexpr float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

                        // Godot est en miroir de la convention du robot (HAL : braquage + = droite, capteur D = droite) :
                        // un braquage positif y tourne vers la gauche. On inverse donc l'ordre des capteurs et le signe
                        // du braquage ici, pour que "gauche" et "droite" aient le meme sens en simulation et sur le robot.
                        for (int i = 0; i < 5; ++i) {
                            const int j = 4 - i;
                            float d = std::clamp(telem_in.line_darkness[j], 0.0f, 1.0f);
                            m_telemetry->line_darkness[i] = d;
                            total_darkness += d;
                            weighted_darkness += weights[i] * d;
                            // Vue binaire : dérivée de l'analogique si présent, sinon trame binaire brute
                            m_telemetry->line_sensors[i] = telem_in.has_analog
                                ? static_cast<uint8_t>(d > 0.5f ? 1 : 0)
                                : telem_in.line_sensors[j];
                        }
                        if (total_darkness >= 0.20f) {
                            m_telemetry->line_error = (weighted_darkness / (2.0f * total_darkness));
                        }
                        if (telem_in.ball_dropped) {
                            m_telemetry->emergency_stop = true;
                        }
                    }
                    // Mode véhicule réel : NE PAS écraser les capteurs physiques (HC-SR04 et I2C) gérés par Sensors!

                    m_telemetry->heartbeat++;

                    float target_spd = m_telemetry->target_speed;
                    float target_steer = m_telemetry->target_steering;
                    bool emergency = m_telemetry->emergency_stop;
                    float current_dist = m_telemetry->filtered_dist;
                    float cruise_cfg = (m_telemetry->cfg_cruise_speed > 0.05f) ? m_telemetry->cfg_cruise_speed : 0.40f;
                    uint8_t current_lines[5];
                    for (int i = 0; i < 5; ++i) {
                        current_lines[i] = m_telemetry->line_sensors[i];
                    }
                    pthread_mutex_unlock(&(m_telemetry->mtx));

                    GodotCommandOut cmd_out;
                    // Vitesse physique réelle respectant STRAIGHT_CRUISE_SPEED dynamique de SHM
                    cmd_out.velocity = (target_spd / 50.0f) * cruise_cfg;
                    // Normalisation de l'angle de braquage [-1.0, 1.0] pour l'épure d'Ackermann de Godot (+-48°)
                    cmd_out.direction = -std::clamp(target_steer / 45.0f, -1.0f, 1.0f);  // miroir Godot (voir plus haut)
                    cmd_out.brake = emergency;

                    if (!m_sim_mode) {
                        // En mode véhicule réel, transmettre aussi les valeurs des capteurs réels vers le client/dashboard
                        cmd_out.distance = current_dist;
                        for (int i = 0; i < 5; ++i) {
                            cmd_out.line_sensors[i] = current_lines[i];
                        }
                        cmd_out.has_sensors = true;
                    }

                    std::string response = SimpleJson::serialize_command(cmd_out);
                    ssize_t sent = send(client_fd, response.c_str(), response.size(), 0);
                    if (sent < 0) {
                        std::cerr << "[SimBridge] Send failed: " << strerror(errno) << std::endl;
                        close(client_fd);
                        client_fd = -1;
                        break;
                    }

                    if (packet_count % 50 == 0) {
                        std::cout << "[SimBridge 50Hz] [" << (m_sim_mode ? "SIM" : "VEHICLE") << "] Packets: " << packet_count
                                  << " | Dist: " << current_dist << " cm"
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
