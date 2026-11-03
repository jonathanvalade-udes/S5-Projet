#include <iostream>
#include <atomic>
#include <csignal>
#include <string>
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

#include "../include/ipc/shared_memory.hpp"
#include "../include/utils/simple_json.hpp"

static std::atomic<bool> g_running{true};

void sig_handler(int signum) {
    (void)signum;
    g_running = false;
}

int main(int argc, char* argv[]) {
    int port = 9000;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = std::atoi(argv[++i]);
        }
    }

    std::cout << "==================================================" << std::endl;
    std::cout << " [PONT VIRTUEL] Godot 4 <--> C++ SHM Bridge       " << std::endl;
    std::cout << " Écoute TCP Directe sur le port : " << port << std::endl;
    std::cout << "==================================================" << std::endl;

    std::signal(SIGINT, sig_handler);
    std::signal(SIGTERM, sig_handler);
    // Ignore SIGPIPE pour éviter que le serveur ne crashe si Godot se déconnecte subitement
    std::signal(SIGPIPE, SIG_IGN);

    // 1. Connexion à la mémoire partagée
    SharedMemoryManager shm_mgr;
    RobotTelemetry* telemetry = nullptr;
    try {
        telemetry = shm_mgr.open_shm(true);
    } catch (const std::exception& e) {
        std::cerr << "[ERREUR CRITIQUE] Impossible d'ouvrir la SHM : " << e.what() << std::endl;
        return 1;
    }

    // 2. Création de la socket serveur TCP
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "[ERREUR] Échec de création du socket TCP : " << strerror(errno) << std::endl;
        return 1;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        std::cerr << "[ERREUR] Échec de bind sur le port " << port << " : " << strerror(errno) << std::endl;
        close(server_fd);
        return 1;
    }

    if (listen(server_fd, 5) < 0) {
        std::cerr << "[ERREUR] Échec de listen : " << strerror(errno) << std::endl;
        close(server_fd);
        return 1;
    }

    // Socket serveur non-bloquant pour permettre un arrêt propre par SIGINT
    fcntl(server_fd, F_SETFL, O_NONBLOCK);

    std::cout << "[PONT] Prêt et en attente de connexion du simulateur Godot 4 sur port " << port << "..." << std::endl;

    std::string rx_buffer;
    rx_buffer.reserve(4096);

    while (g_running) {
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd, reinterpret_cast<sockaddr*>(&client_addr), &client_len);

        if (client_fd < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            std::cerr << "[PONT] Erreur accept : " << strerror(errno) << std::endl;
            break;
        }

        std::cout << "[PONT] >>> Client Godot 4 connecté depuis " 
                  << inet_ntoa(client_addr.sin_addr) << ":" << ntohs(client_addr.sin_port) 
                  << " ! <<<" << std::endl;

        // Configuration du socket client :
        // 1. Réinitialiser en mode bloquant avec timeout de 200 ms
        fcntl(client_fd, F_SETFL, 0);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200000; // 200 ms timeout
        setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

        // 2. Désactiver l'algorithme de Nagle (TCP_NODELAY) pour zéro latence temps-réel
        int flag_nodelay = 1;
        setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, (char*)&flag_nodelay, sizeof(int));

#ifdef SO_NOSIGPIPE
        int flag_nosigpipe = 1;
        setsockopt(client_fd, SOL_SOCKET, SO_NOSIGPIPE, &flag_nosigpipe, sizeof(int));
#endif

        rx_buffer.clear();
        char recv_buf[1024];
        uint64_t packet_count = 0;

        while (g_running) {
            ssize_t bytes_read = recv(client_fd, recv_buf, sizeof(recv_buf) - 1, 0);
            
            if (bytes_read < 0) {
                // Si timeout ou interruption système : on continue normalement !
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                    continue;
                }
                std::cout << "[PONT] Déconnexion / Erreur socket : " << strerror(errno) << std::endl;
                break;
            }

            if (bytes_read == 0) {
                std::cout << "[PONT] Godot 4 a fermé la connexion proprement (EOF)." << std::endl;
                break;
            }

            recv_buf[bytes_read] = '\0';
            rx_buffer.append(recv_buf, bytes_read);

            // Traitement des lignes JSON délimitées par '\n'
            size_t newline_pos = std::string::npos;
            while ((newline_pos = rx_buffer.find('\n')) != std::string::npos) {
                std::string line = rx_buffer.substr(0, newline_pos);
                rx_buffer.erase(0, newline_pos + 1);

                if (line.empty()) continue;

                packet_count++;

                // 1. Parsing de la télémétrie Godot
                GodotTelemetryIn telem_in;
                if (SimpleJson::parse_godot_telemetry(line, telem_in)) {
                    // 2. Écriture atomique dans la mémoire partagée
                    pthread_mutex_lock(&(telemetry->mtx));
                    telemetry->filtered_dist = telem_in.distance;
                    for (int i = 0; i < 5; ++i) {
                        telemetry->line_sensors[i] = telem_in.line_sensors[i];
                    }
                    if (telem_in.ball_dropped) {
                        telemetry->emergency_stop = true;
                    }
                    telemetry->heartbeat++;

                    // 3. Lecture des consignes de picar_brain
                    float target_spd = telemetry->target_speed;
                    float target_steer = telemetry->target_steering;
                    bool emergency = telemetry->emergency_stop;
                    pthread_mutex_unlock(&(telemetry->mtx));

                    // 4. Génération de la commande vers Godot
                    GodotCommandOut cmd_out;
                    // Vitesse : 50% = 0.25 m/s
                    cmd_out.velocity = (target_spd / 50.0f) * 0.25f;
                    // Braquage : angle en degrés divisé par 45°
                    cmd_out.direction = target_steer / 45.0f;
                    cmd_out.brake = emergency;

                    std::string response = SimpleJson::serialize_command(cmd_out);
                    ssize_t sent = send(client_fd, response.c_str(), response.size(), 0);
                    if (sent < 0) {
                        std::cerr << "[PONT] Échec envoi vers Godot : " << strerror(errno) << std::endl;
                        break;
                    }

                    if (packet_count % 50 == 0) {
                        std::cout << "[PONT 50Hz] Paquets échangés : " << packet_count
                                  << " | Dist Godot: " << telem_in.distance << " cm"
                                  << " | Vitesse C++: " << cmd_out.velocity << " m/s"
                                  << " | Braquage C++: " << cmd_out.direction << std::endl;
                    }
                }
            }
        }

        close(client_fd);
        std::cout << "[PONT] Client déconnecté. En attente d'une nouvelle session..." << std::endl;
    }

    close(server_fd);
    std::cout << "[PONT] Arrêt du pont virtuel Godot." << std::endl;
    return 0;
}
