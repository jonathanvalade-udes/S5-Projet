#include "TelemetryClient.hpp"

#include <iostream>
#include <chrono>

namespace telemetry {

TelemetryClient::TelemetryClient(const TelemetryClientConfig& config)
    : m_config(config),
      m_web(config.web_port, config.web_root) {}

TelemetryClient::~TelemetryClient() {
    stop();
}

bool TelemetryClient::init() {
    std::cout << "==================================================" << std::endl;
    std::cout << "  FastDrink Telemetry Client (C++17)             " << std::endl;
    std::cout << "  Cible Robot : " << m_config.robot_ip << ":" << m_config.robot_port << std::endl;
    if (m_config.enable_web) {
        std::cout << "  Serveur Web : http://localhost:" << m_config.web_port << std::endl;
    }
    if (m_config.enable_csv) {
        std::cout << "  Fichier CSV : " << m_config.csv_path << std::endl;
    }
    std::cout << "==================================================" << std::endl;

    if (m_config.enable_csv) {
        m_csv_logger.open(m_config.csv_path);
        m_web.set_csv_filepath(m_config.csv_path);
    }

    if (m_config.enable_web) {
        m_web.set_command_callback([this](const std::string& cmd) {
            queue_command(cmd);
        });

        if (!m_web.start()) {
            std::cerr << "[TelemetryClient] Avertissement: Impossible de démarrer le serveur Web sur le port "
                      << m_config.web_port << std::endl;
        }
    }

    return true;
}

void TelemetryClient::run() {
    m_running.store(true);
    m_tcp_thread = std::thread(&TelemetryClient::tcp_worker_loop, this);

    while (m_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (m_tcp_thread.joinable()) {
        m_tcp_thread.join();
    }
}

void TelemetryClient::stop() {
    if (!m_running.load()) return;
    std::cout << "[TelemetryClient] Arrêt en cours..." << std::endl;
    m_running.store(false);

    m_tcp.disconnect();
    m_web.stop();
    if (m_config.enable_csv) {
        m_csv_logger.close();
    }

    if (m_tcp_thread.joinable()) {
        m_tcp_thread.join();
    }
}

void TelemetryClient::queue_command(const std::string& command_str) {
    std::lock_guard<std::mutex> lock(m_cmd_mutex);
    m_pending_commands.push(command_str);
    std::cout << "[TelemetryClient] Commande mise en file d'attente : " << command_str << std::endl;
}

void TelemetryClient::process_pending_commands() {
    std::queue<std::string> cmds;
    {
        std::lock_guard<std::mutex> lock(m_cmd_mutex);
        std::swap(cmds, m_pending_commands);
    }

    while (!cmds.empty() && m_tcp.is_connected()) {
        std::string cmd = cmds.front();
        cmds.pop();

        uint16_t seq = m_cmd_seq++;
        m_proto.send_command(m_tcp, cmd, seq);
    }
}

void TelemetryClient::tcp_worker_loop() {
    auto last_activity_time = std::chrono::steady_clock::now();
    auto last_broadcast_time = std::chrono::steady_clock::now();
    constexpr auto MIN_BROADCAST_INTERVAL = std::chrono::milliseconds(16); // ~60 Hz

    while (m_running.load()) {
        // 1. Tenter la connexion si non connecté
        if (!m_tcp.is_connected()) {
            if (m_robot_connected.load()) {
                m_robot_connected.store(false);
                if (m_config.enable_web) {
                    m_web.broadcast_status(false, m_config.robot_ip, m_config.robot_port);
                }
            }

            std::cout << "[TelemetryClient] Tentative de connexion vers " 
                      << m_config.robot_ip << ":" << m_config.robot_port << "..." << std::endl;

            if (!m_tcp.connect_to(m_config.robot_ip, m_config.robot_port)) {
                // Attente 1.5s avant de retenter
                for (int i = 0; i < 15 && m_running.load(); ++i) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                continue;
            }

            // 2. Poignée de main Ping-Pong obligatoire
            if (!m_proto.perform_handshake(m_tcp, 1)) {
                std::cerr << "[TelemetryClient] Échec du handshake. Déconnexion et nouvelle tentative..." << std::endl;
                m_tcp.disconnect();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            m_robot_connected.store(true);
            if (m_config.enable_web) {
                m_web.broadcast_status(true, m_config.robot_ip, m_config.robot_port);
            }
            last_activity_time = std::chrono::steady_clock::now();
            last_broadcast_time = last_activity_time;
        }

        // 3. Envoyer les commandes en attente
        process_pending_commands();

        // 4. Recevoir le prochain paquet de télémétrie (bloquant max 200ms)
        shared::car::CarTelemetry telem{};
        uint16_t seq = 0;
        bool ok = m_proto.read_telemetry_packet(m_tcp, telem, seq, 200);

        auto now = std::chrono::steady_clock::now();

        if (ok) {
            last_activity_time = now;

            // Enregistrement CSV continu et non-bloquant de TOUTES les données
            if (m_config.enable_csv) {
                m_csv_logger.log_record(seq, telem);
            }

            // Diffusion Web SSE régulée à ~60 Hz avec la dernière trame fraîche
            if (m_config.enable_web) {
                if (now - last_broadcast_time >= MIN_BROADCAST_INTERVAL) {
                    std::string json_str = ProtocolHandler::telemetry_to_json(telem, seq);
                    m_web.broadcast_telemetry(json_str);
                    last_broadcast_time = now;
                }
            }
        } else if (!m_tcp.is_connected()) {
            // Déconnexion détectée pendant la lecture
            m_robot_connected.store(false);
            if (m_config.enable_web) {
                m_web.broadcast_status(false, m_config.robot_ip, m_config.robot_port);
            }
            continue;
        }

        // 5. Keep-alive intelligent : n'envoyer un Ping que si AUCUN paquet n'a été reçu depuis 3 secondes
        if (m_tcp.is_connected() && std::chrono::duration_cast<std::chrono::seconds>(now - last_activity_time).count() >= 3) {
            m_proto.send_ping(m_tcp, m_cmd_seq++);
            last_activity_time = now;
        }
    }
}

} // namespace telemetry
