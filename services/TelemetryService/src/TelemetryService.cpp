#include "TelemetryService.hpp"
#include "control/picar_config.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <cmath>
#include <cstring>
#include <cerrno>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <ctime>
#include <fstream>

#if !defined(MSG_NOSIGNAL)
#define MSG_NOSIGNAL 0
#endif

namespace services {

static const char* state_to_string(uint32_t state) {
    switch (static_cast<shared::fsm::MachineStates>(state)) {
        case shared::fsm::MachineStates::Init: return "Init";
        case shared::fsm::MachineStates::Unarmed: return "Unarmed";
        case shared::fsm::MachineStates::Armed: return "Armed";
        case shared::fsm::MachineStates::LineFollowingForward: return "LineFollowingForward";
        case shared::fsm::MachineStates::CollisionRisk: return "CollisionRisk";
        case shared::fsm::MachineStates::Stopped: return "Stopped";
        case shared::fsm::MachineStates::CollisionOccured: return "CollisionOccured";
        case shared::fsm::MachineStates::RightDeviation: return "RightDeviation";
        case shared::fsm::MachineStates::LeftDeviation: return "LeftDeviation";
        case shared::fsm::MachineStates::Reverse: return "Reverse";
        case shared::fsm::MachineStates::LineSearching: return "LineSearching";
        case shared::fsm::MachineStates::LineDetected: return "LineDetected";
        case shared::fsm::MachineStates::LineSearchingTimedOut: return "LineSearchingTimedOut";
        case shared::fsm::MachineStates::Abort: return "Abort";
        case shared::fsm::MachineStates::Failure: return "Failure";
        default: return "Undefined";
    }
}

TelemetryService::TelemetryService(int hz, bool enable_ui, int tcp_port, int udp_port)
    : m_running(false),
      m_hz(hz),
      m_enable_ui(enable_ui),
      m_tcp_port(tcp_port),
      m_udp_port(udp_port) {}

TelemetryService::~TelemetryService() {
    stop();
    if (m_udp_fd >= 0) {
        close(m_udp_fd);
        m_udp_fd = -1;
    }
}

bool TelemetryService::init() {
    std::cout << "[TelemetryService] Initializing hub at " << m_hz << " Hz (TCP port "
              << m_tcp_port << ", UDP port " << m_udp_port << ")..." << std::endl;

    // 1. Ouvrir la mémoire partagée POSIX classique (rétrocompatibilité)
    try {
        m_shm_mgr = std::make_unique<SharedMemoryManager>();
        m_telemetry = m_shm_mgr->open_shm(true);
    } catch (const std::exception& e) {
        std::cerr << "[TelemetryService] Warning: Failed to open POSIX SHM: " << e.what() << std::endl;
        m_telemetry = nullptr;
    }

    // 2. Initialiser les souscriptions et publications SeqLock (/dev/shm/car/)
    try {
        m_telemetry_pub = std::make_unique<shared::services::memory::MemoryShareComponentServer>("Telemetry");
        m_sensor_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("Sensors");
        m_fsm_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("FSM");
        m_motor_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("Motors");
        m_servo_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("Servo");
    } catch (const std::exception& e) {
        std::cerr << "[TelemetryService] Warning: MemoryShare init: " << e.what() << std::endl;
    }

    // 3. Initialiser le serveur TCP (port 91 par défaut)
    if (m_tcp_port > 0) {
        init_tcp_server();
    }

    // 4. Initialiser le socket UDP broadcast
    if (m_udp_port > 0) {
        m_udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_udp_fd >= 0) {
            int broadcast = 1;
            setsockopt(m_udp_fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
            int flags = fcntl(m_udp_fd, F_GETFL, 0);
            fcntl(m_udp_fd, F_SETFL, flags | O_NONBLOCK);
        }
    }

    // 5. Initialiser le fichier CSV local dans /tmp/telemetry.csv
    try {
        m_csv_stream.open(m_local_csv_path, std::ios::out | std::ios::trunc);
        if (m_csv_stream.is_open()) {
            m_csv_stream << "timestamp_ms,uptime_s,fsm_state,speed_ms,accel_ms2,steer_deg,steer_offset_deg,dist_us_cm,line_0,line_1,line_2,line_3,line_4,odometry_m,emergency_stop,accel_violations\n";
            m_csv_stream.flush();
            std::cout << "[TelemetryService] Enregistrement continu en local initialisé : " << m_local_csv_path << std::endl;
        } else {
            std::cerr << "[TelemetryService] Attention : impossible d'ouvrir " << m_local_csv_path << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[TelemetryService] Erreur init CSV : " << e.what() << std::endl;
    }

    m_start_time = std::chrono::steady_clock::now();
    m_running.store(true);

    // Lancer le thread TCP pour écouter et streamer vers le client
    if (m_tcp_server_fd >= 0) {
        m_tcp_thread = std::thread(&TelemetryService::tcp_worker, this);
    }

    return true;
}

bool TelemetryService::init_tcp_server() {
    m_tcp_server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_tcp_server_fd < 0) {
        std::cerr << "[TelemetryService] Failed to create TCP socket: " << std::strerror(errno) << std::endl;
        return false;
    }

    int opt = 1;
    setsockopt(m_tcp_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(SO_REUSEPORT)
    setsockopt(m_tcp_server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

    struct sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(m_tcp_port);

    if (bind(m_tcp_server_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "[TelemetryService] Warning: Failed to bind TCP port " << m_tcp_port
                  << ": " << std::strerror(errno);
        if (errno == EACCES && m_tcp_port < 1024) {
            std::cerr << " (Root privileges required to bind port < 1024)";
        }
        std::cerr << std::endl;
        close(m_tcp_server_fd);
        m_tcp_server_fd = -1;
        return false;
    }

    if (listen(m_tcp_server_fd, 5) < 0) {
        std::cerr << "[TelemetryService] Failed to listen on TCP port " << m_tcp_port
                  << ": " << std::strerror(errno) << std::endl;
        close(m_tcp_server_fd);
        m_tcp_server_fd = -1;
        return false;
    }

    int flags = fcntl(m_tcp_server_fd, F_GETFL, 0);
    fcntl(m_tcp_server_fd, F_SETFL, flags | O_NONBLOCK);

    std::cout << "[TelemetryService] TCP Server listening on port " << m_tcp_port << "..." << std::endl;
    return true;
}

void TelemetryService::close_tcp_client() {
    if (m_tcp_client_fd >= 0) {
        close(m_tcp_client_fd);
        m_tcp_client_fd = -1;
    }
    m_client_connected.store(false);
    m_client_handshaked.store(false);
    m_new_data_available.store(false);
    m_stream_cv.notify_all();
}

void TelemetryService::close_tcp_server() {
    close_tcp_client();
    if (m_tcp_server_fd >= 0) {
        close(m_tcp_server_fd);
        m_tcp_server_fd = -1;
    }
}

bool TelemetryService::handle_ping_pong() {
    if (m_tcp_client_fd < 0) return false;

    struct pollfd pfd{};
    pfd.fd = m_tcp_client_fd;
    pfd.events = POLLIN;
    int poll_ret = poll(&pfd, 1, 50);
    if (poll_ret <= 0) {
        return false;
    }
    if (pfd.revents & (POLLHUP | POLLERR)) {
        close_tcp_client();
        return false;
    }

    // Lecture du header pour connaître la longueur
    shared::headers::PacketHeader rx_hdr{};
    ssize_t n = recv(m_tcp_client_fd, &rx_hdr, sizeof(rx_hdr), MSG_PEEK);
    if (n <= 0) {
        close_tcp_client();
        return false;
    }
    if (static_cast<size_t>(n) < sizeof(rx_hdr)) {
        return false;
    }

    bool has_crc = (rx_hdr.headerFlag & shared::headers::flags::CRC_ENABLED) != 0;
    size_t expected_packet_len = sizeof(shared::headers::PacketHeader) +
                                 rx_hdr.payloadLength +
                                 (has_crc ? sizeof(shared::crc::PacketCRC) : 0);

    std::vector<uint8_t> rx_buffer(expected_packet_len);
    size_t received_bytes = 0;
    while (received_bytes < expected_packet_len) {
        int r_ret = poll(&pfd, 1, 200);
        if (r_ret <= 0) {
            close_tcp_client();
            return false;
        }
        ssize_t r = recv(m_tcp_client_fd, rx_buffer.data() + received_bytes, expected_packet_len - received_bytes, 0);
        if (r <= 0) {
            close_tcp_client();
            return false;
        }
        received_bytes += r;
    }

    // Désérialisation et validation
    shared::headers::PacketHeader hdr{};
    std::vector<uint8_t> payload;
    uint32_t crc_val = 0;
    if (!shared::headers::deserializePacket(rx_buffer.data(), rx_buffer.size(), hdr, payload, crc_val)) {
        std::cerr << "[TelemetryService] Invalid packet received or CRC mismatch during handshake!" << std::endl;
        return false;
    }

    // Vérifier s'il s'agit d'une commande
    if (hdr.type == shared::headers::HeadersType::Command) {
        if (!payload.empty()) {
            std::string payload_str(payload.begin(), payload.end());

            // Commande de départ FSM
            if (payload_str.find("trigger_start") != std::string::npos || payload_str == "start") {
                int fifo_fd = open("/tmp/peripherals.sock", O_WRONLY | O_NONBLOCK);
                if (fifo_fd >= 0) {
                    const char* cmd = "trigger_start\n";
                    ssize_t w = write(fifo_fd, cmd, strlen(cmd));
                    (void)w;
                    close(fifo_fd);
                    std::cout << "[Telemetry WiFi] Transmis 'trigger_start' vers /tmp/peripherals.sock" << std::endl;
                }
                if (m_telemetry) {
                    pthread_mutex_lock(&(m_telemetry->mtx));
                    m_telemetry->emergency_stop = false;
                    pthread_mutex_unlock(&(m_telemetry->mtx));
                }
                return true;
            }

            // Commande d'arrêt d'urgence
            if (payload_str.find("emergency_stop") != std::string::npos || payload_str == "stop") {
                if (m_telemetry) {
                    pthread_mutex_lock(&(m_telemetry->mtx));
                    m_telemetry->emergency_stop = true;
                    m_telemetry->target_speed = 0.0f;
                    pthread_mutex_unlock(&(m_telemetry->mtx));
                }
                std::cout << "[Telemetry WiFi] Arrêt d'urgence déclenché par client !" << std::endl;
                return true;
            }

            // Commande d'export vers clé USB
            if (payload_str.find("export_usb") != std::string::npos || payload_str == "export") {
                export_to_usb();
                return true;
            }

            // Commande texte de configuration (ex: "MAX_ACCELERATION=0.55")
            size_t eq = payload_str.find('=');
            if (eq != std::string::npos) {
                std::string k = payload_str.substr(0, eq);
                std::string v = payload_str.substr(eq + 1);
                k.erase(std::remove_if(k.begin(), k.end(), ::isspace), k.end());
                v.erase(std::remove_if(v.begin(), v.end(), ::isspace), v.end());
                try {
                    float fval = std::stof(v);
                    // 1. Écriture dans le .config
                    config::PicarConfig::write_config(k, fval);
                    // 2. Application dans la mémoire partagée POSIX
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
                    std::cout << "[Telemetry WiFi] Paramètre mis à jour : " << k << " = " << fval << std::endl;
                    return true;
                } catch (...) {}
            }
        }

        std::cout << "[TelemetryService] Received Ping command (seq=" << hdr.seqNumber << "). Sending Pong response..." << std::endl;

        // Préparer la réponse Pong avec CRC calculé
        shared::headers::PacketHeader pong_hdr{};
        pong_hdr.type = shared::headers::HeadersType::Response;
        pong_hdr.headerFlag = shared::headers::flags::CRC_ENABLED;
        pong_hdr.seqNumber = hdr.seqNumber; // Règle: préserver le même numéro de séquence
        pong_hdr.payloadLength = sizeof(uint32_t);

        uint32_t pong_code = static_cast<uint32_t>(shared::responses::ResponseID::Pong);
        auto pong_packet = shared::headers::serializePacket(
            pong_hdr,
            reinterpret_cast<const uint8_t*>(&pong_code),
            sizeof(pong_code)
        );

        size_t sent = 0;
        while (sent < pong_packet.size()) {
            int flags = 0;
#if defined(MSG_NOSIGNAL)
            flags |= MSG_NOSIGNAL;
#endif
            ssize_t w = send(m_tcp_client_fd, pong_packet.data() + sent, pong_packet.size() - sent, flags);
            if (w <= 0) {
                close_tcp_client();
                return false;
            }
            sent += w;
        }

        std::cout << "[TelemetryService] Pong response sent (seq=" << pong_hdr.seqNumber 
                  << "). Handshake completed! Starting telemetry data stream..." << std::endl;
        m_client_handshaked.store(true);
        return true;
    }

    return false;
}

bool TelemetryService::stream_telemetry_packet() {
    if (m_tcp_client_fd < 0 || !m_client_handshaked.load()) {
        return false;
    }

    shared::car::CarTelemetry snapshot;
    {
        std::lock_guard<std::mutex> lock(m_telem_mutex);
        snapshot = m_car_telemetry;
    }

    shared::headers::PacketHeader hdr{};
    hdr.type = shared::headers::HeadersType::Telemetry;
    hdr.headerFlag = shared::headers::flags::CRC_ENABLED;
    hdr.seqNumber = m_tcp_seq_num++;
    hdr.payloadLength = sizeof(snapshot);

    auto packet = shared::headers::serializePacket(
        hdr,
        reinterpret_cast<const uint8_t*>(&snapshot),
        sizeof(snapshot)
    );

    size_t sent = 0;
    while (sent < packet.size()) {
        int flags = 0;
#if defined(MSG_NOSIGNAL)
        flags |= MSG_NOSIGNAL;
#endif
        ssize_t w = send(m_tcp_client_fd, packet.data() + sent, packet.size() - sent, flags);
        if (w <= 0) {
            std::cout << "[TelemetryService] TCP client disconnected during stream." << std::endl;
            close_tcp_client();
            return false;
        }
        sent += w;
    }

    return true;
}

void TelemetryService::tcp_worker() {
    std::cout << "[TelemetryService] TCP worker started on port " << m_tcp_port << "." << std::endl;

    while (m_running.load()) {
        // 1. Attente de connexion client si aucun client connecté
        if (m_tcp_client_fd < 0) {
            struct pollfd pfd{};
            pfd.fd = m_tcp_server_fd;
            pfd.events = POLLIN;

            int ret = poll(&pfd, 1, 100);
            if (ret > 0 && (pfd.revents & POLLIN)) {
                struct sockaddr_in client_addr{};
                socklen_t client_len = sizeof(client_addr);
                int cfd = accept(m_tcp_server_fd, (struct sockaddr*)&client_addr, &client_len);
                if (cfd >= 0) {
#if defined(SO_NOSIGPIPE)
                    int nosig = 1;
                    setsockopt(cfd, SOL_SOCKET, SO_NOSIGPIPE, &nosig, sizeof(nosig));
#endif
                    int nodelay = 1;
                    setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

#if defined(TCP_QUICKACK)
                    int quickack = 1;
                    setsockopt(cfd, IPPROTO_TCP, TCP_QUICKACK, &quickack, sizeof(quickack));
#endif
                    int sndbuf = 65536;
                    setsockopt(cfd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

                    char client_ip[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip, INET_ADDRSTRLEN);
                    std::cout << "[TelemetryService] Client connected from " << client_ip << ":"
                              << ntohs(client_addr.sin_port) << ". Waiting for ping-pong handshake..." << std::endl;

                    m_tcp_client_fd = cfd;
                    m_client_connected.store(true);
                    m_client_handshaked.store(false);
                }
            }
            continue;
        }

        // 2. Client connecté : attente et gestion du handshake ping-pong
        if (!m_client_handshaked.load()) {
            handle_ping_pong();
            continue;
        }

        // 3. Handshake terminé : attente de l'événement de mise à jour temps N (zéro délai supplémentaire)
        {
            std::unique_lock<std::mutex> lock(m_stream_mutex);
            m_stream_cv.wait_for(lock, std::chrono::milliseconds(20), [&]() {
                return !m_running.load() || m_new_data_available.load() || !m_client_connected.load();
            });
        }

        if (!m_running.load()) break;
        if (!m_client_connected.load()) continue;

        // Vérifier si le client a envoyé un message (ex: ping keepalive)
        struct pollfd pfd{};
        pfd.fd = m_tcp_client_fd;
        pfd.events = POLLIN;
        int ret = poll(&pfd, 1, 0); // Non-bloquant
        if (ret > 0 && (pfd.revents & POLLIN)) {
            handle_ping_pong();
        } else if (ret > 0 && (pfd.revents & (POLLHUP | POLLERR))) {
            close_tcp_client();
            continue;
        }

        // Envoi instantané des données de télémétrie dès qu'une mise à jour est prête
        if (m_new_data_available.exchange(false) && m_tcp_client_fd >= 0 && m_client_handshaked.load()) {
            if (!stream_telemetry_packet()) {
                continue;
            }
        }
    }

    close_tcp_client();
    std::cout << "[TelemetryService] TCP worker terminated." << std::endl;
}

void TelemetryService::collect_shared_memories() {
    // Lecture de toutes les régions partagées du système
    shared::sensors::SensorsTelemetry sensors_data{};
    bool has_sensors = m_sensor_sub && m_sensor_sub->read(sensors_data);

    shared::motors::MotorsComponentTelemetry motors_data{};
    bool has_motors = m_motor_sub && m_motor_sub->read(motors_data);

    shared::servos::ServoTelemetry servo_data{};
    bool has_servo = m_servo_sub && m_servo_sub->read(servo_data);

    shared::fsm::FSMTelemetry fsm_data{};
    bool has_fsm = m_fsm_sub && m_fsm_sub->read(fsm_data);

    std::lock_guard<std::mutex> lock(m_telem_mutex);

    if (has_sensors) {
        m_car_telemetry.sensors = sensors_data;
        m_metrics.filtered_dist_cm = sensors_data.ultrasonicSensor.calculatedDistance;
        for (int i = 0; i < 5; ++i) {
            m_metrics.line_sensors[i] = (sensors_data.lineFollower.sensorsData[i].normalizedValue > 0.5f ||
                                         sensors_data.lineFollower.sensorsData[i].rawADCValue > 500) ? 1 : 0;
        }
    }

    if (has_motors) {
        m_car_telemetry.motors = motors_data;
        m_metrics.target_speed_percent = motors_data.leftMotor.normalizedMotorThrottleApplied * 100.0f;
    }

    if (has_servo) {
        m_car_telemetry.servo = servo_data;
        m_metrics.target_steering_deg = servo_data.normalizedCurrentAngle * shared::servos::MAX_ANGLE;
    } else if (has_motors) {
        m_car_telemetry.servo = motors_data.steeringServo;
        m_metrics.target_steering_deg = motors_data.steeringServo.normalizedCurrentAngle * shared::servos::MAX_ANGLE;
    }

    if (has_fsm) {
        m_car_telemetry.fsm = fsm_data;
        m_metrics.fsm_state = static_cast<uint32_t>(fsm_data.currentState);
    }

    // Synchronisation et secours avec la mémoire partagée POSIX classique si des services ne publient pas encore
    if (m_telemetry) {
        pthread_mutex_lock(&(m_telemetry->mtx));
        if (!has_sensors) {
            m_metrics.filtered_dist_cm = m_telemetry->filtered_dist;
            for (int i = 0; i < 5; ++i) {
                m_metrics.line_sensors[i] = m_telemetry->line_sensors[i];
                m_car_telemetry.sensors.lineFollower.sensorsData[i].rawADCValue = m_telemetry->line_sensors[i] ? 1023 : 0;
                m_car_telemetry.sensors.lineFollower.sensorsData[i].normalizedValue = m_telemetry->line_sensors[i] ? 1.0f : 0.0f;
            }
            m_car_telemetry.sensors.ultrasonicSensor.calculatedDistance = m_metrics.filtered_dist_cm;
            m_car_telemetry.sensors.ultrasonicSensor.rawDistanceCm = m_metrics.filtered_dist_cm;
            m_car_telemetry.sensors.flags = shared::sensors::flags::DISCOVERABLE;
        }
        if (!has_motors) {
            m_metrics.target_speed_percent = m_telemetry->target_speed;
            m_car_telemetry.motors.leftMotor.normalizedMotorThrottleApplied = m_metrics.target_speed_percent / 100.0f;
            m_car_telemetry.motors.rightMotor.normalizedMotorThrottleApplied = m_metrics.target_speed_percent / 100.0f;
            m_car_telemetry.motors.leftMotor.normalizedMotorThrottleTargetted = m_metrics.target_speed_percent / 100.0f;
            m_car_telemetry.motors.rightMotor.normalizedMotorThrottleTargetted = m_metrics.target_speed_percent / 100.0f;
            m_car_telemetry.motors.state = (std::abs(m_metrics.target_speed_percent) > 1.0f)
                                           ? shared::motors::MotorStates::Running
                                           : shared::motors::MotorStates::Idle;
            m_car_telemetry.motors.componentFlags = shared::motors::flags::component::DISCOVERABLE;
        }
        if (!has_servo) {
            m_metrics.target_steering_deg = m_telemetry->target_steering;
            m_car_telemetry.servo.normalizedCurrentAngle = m_metrics.target_steering_deg / shared::servos::MAX_ANGLE;
            m_car_telemetry.servo.normalizedTargettedAngle = m_metrics.target_steering_deg / shared::servos::MAX_ANGLE;
            m_car_telemetry.servo.flags = shared::servos::flags::RDY;
            m_car_telemetry.motors.steeringServo = m_car_telemetry.servo;
        }
        if (!has_fsm) {
            m_metrics.fsm_state = m_telemetry->state;
            m_car_telemetry.fsm.currentState = static_cast<shared::fsm::MachineStates>(m_metrics.fsm_state);
            m_car_telemetry.fsm.requestedState = m_car_telemetry.fsm.currentState;
            m_car_telemetry.fsm.flags = shared::fsm::flags::RDY;
        }
        m_metrics.emergency_stop = m_telemetry->emergency_stop;
        m_metrics.heartbeat = m_telemetry->heartbeat;
        m_metrics.steering_offset_deg = m_telemetry->steering_offset;
        pthread_mutex_unlock(&(m_telemetry->mtx));
    }

    // Télémétrie des services du système
    m_car_telemetry.services.flags = shared::services::flags::SYSTEM_STARTED |
                                     shared::services::flags::SHARE_MEMORY_MOUNTED |
                                     shared::services::flags::SOCKET_ENABLED |
                                     shared::services::flags::DISCOVERABLE;
    if (m_metrics.watchdog_stalled) {
        m_car_telemetry.services.flags |= shared::services::flags::PROCESS_STALLED;
        m_car_telemetry.services.flags |= shared::services::flags::ANOMALIES_DETECTED;
        m_car_telemetry.fsm.flags |= shared::fsm::flags::STALLED;
    }
    if (m_metrics.accel_violations > 0 || m_metrics.emergency_stop) {
        m_car_telemetry.services.flags |= shared::services::flags::ANOMALIES_DETECTED;
    }

    auto now = std::chrono::steady_clock::now().time_since_epoch();
    m_car_telemetry.services.timestamp = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
    m_car_telemetry.services.handlerErrorCode = 0;
    m_car_telemetry.services.deamonsLenght = 4;

    // Statut des daemons
    m_car_telemetry.services.daemons[0].id = 1; // Sensors
    m_car_telemetry.services.daemons[0].status = has_sensors ? 1 : 0;
    m_car_telemetry.services.daemons[1].id = 2; // Motors
    m_car_telemetry.services.daemons[1].status = has_motors ? 1 : 0;
    m_car_telemetry.services.daemons[2].id = 3; // Servo
    m_car_telemetry.services.daemons[2].status = (has_servo || has_motors) ? 1 : 0;
    m_car_telemetry.services.daemons[3].id = 4; // FSM
    m_car_telemetry.services.daemons[3].status = has_fsm ? 1 : 0;
}

void TelemetryService::update_metrics(float dt) {
    collect_shared_memories();

    // Calcul vitesse en m/s (50% = 0.32 m/s vitesse nominale)
    m_metrics.current_speed_ms = (m_metrics.target_speed_percent / 50.0f) * 0.32f;

    // Calcul accélération a = dv/dt
    m_metrics.current_accel_ms2 = (m_metrics.current_speed_ms - m_last_speed_ms) / dt;

    // Détection dépassement seuil critique FastDrink (0.735 m/s² + tolérance discrétisation 0.05)
    if (std::abs(m_metrics.current_accel_ms2) > 0.785f) {
        m_metrics.accel_violations++;
    }
    m_last_speed_ms = m_metrics.current_speed_ms;

    // Intégration odométrique
    m_metrics.total_distance_m += std::abs(m_metrics.current_speed_ms) * dt;

    // Watchdog de vivacité (heartbeat)
    if (m_metrics.heartbeat == m_last_heartbeat) {
        m_heartbeat_stalled_cycles++;
        if (m_heartbeat_stalled_cycles >= (m_hz / 2)) {
            m_metrics.watchdog_stalled = true;
        }
    } else {
        m_heartbeat_stalled_cycles = 0;
        m_metrics.watchdog_stalled = false;
        m_last_heartbeat = m_metrics.heartbeat;
    }

    auto now = std::chrono::steady_clock::now();
    m_metrics.uptime_seconds = std::chrono::duration_cast<std::chrono::seconds>(now - m_start_time).count();

    // 1. Écriture continue du point de mesure dans /tmp/telemetry.csv
    log_csv_sample();

    // 2. Détection de fin de course ou arrêt d'urgence pour export automatique sur clé USB
    bool is_course_active = (m_metrics.fsm_state == static_cast<uint32_t>(shared::fsm::MachineStates::LineFollowingForward) ||
                             m_metrics.fsm_state == static_cast<uint32_t>(shared::fsm::MachineStates::CollisionRisk) ||
                             m_metrics.fsm_state == static_cast<uint32_t>(shared::fsm::MachineStates::RightDeviation) ||
                             m_metrics.fsm_state == static_cast<uint32_t>(shared::fsm::MachineStates::LeftDeviation) ||
                             m_metrics.fsm_state == static_cast<uint32_t>(shared::fsm::MachineStates::Reverse) ||
                             m_metrics.fsm_state == static_cast<uint32_t>(shared::fsm::MachineStates::LineSearching) ||
                             m_metrics.fsm_state == static_cast<uint32_t>(shared::fsm::MachineStates::LineDetected));

    if (is_course_active) {
        if (!m_was_running_course) {
            m_was_running_course = true;
            if (m_csv_stream.is_open()) {
                m_csv_stream.close();
            }
            m_csv_stream.open(m_local_csv_path, std::ios::out | std::ios::trunc);
            if (m_csv_stream.is_open()) {
                m_csv_stream << "timestamp_ms,uptime_s,fsm_state,speed_ms,accel_ms2,steer_deg,steer_offset_deg,dist_us_cm,line_0,line_1,line_2,line_3,line_4,odometry_m,emergency_stop,accel_violations\n";
                m_csv_stream.flush();
            }
            std::cout << "\n[TelemetryService] Début d'une nouvelle course -> Initialisation du log " << m_local_csv_path << std::endl;
        }
    } else if (m_was_running_course && (m_metrics.fsm_state == static_cast<uint32_t>(shared::fsm::MachineStates::Stopped) ||
                                       m_metrics.fsm_state == static_cast<uint32_t>(shared::fsm::MachineStates::Armed) ||
                                       m_metrics.fsm_state == static_cast<uint32_t>(shared::fsm::MachineStates::Failure) ||
                                       m_metrics.fsm_state == static_cast<uint32_t>(shared::fsm::MachineStates::Abort) ||
                                       m_metrics.fsm_state == static_cast<uint32_t>(shared::fsm::MachineStates::LineSearchingTimedOut) ||
                                       m_metrics.emergency_stop)) {
        std::cout << "\n[TelemetryService] Fin de parcours ou anomalie/arrêt détecté -> Export automatique sur la clé USB..." << std::endl;
        export_to_usb();
        m_was_running_course = false;
    }
}

void TelemetryService::publish_ipc() {
    if (m_telemetry_pub && m_telemetry_pub->isValid()) {
        shared::car::CarTelemetry telem;
        {
            std::lock_guard<std::mutex> lock(m_telem_mutex);
            telem = m_car_telemetry;
        }
        m_telemetry_pub->publish(telem);
    }
}

shared::car::CarTelemetry TelemetryService::get_car_telemetry() const {
    std::lock_guard<std::mutex> lock(m_telem_mutex);
    return m_car_telemetry;
}

void TelemetryService::broadcast_udp() {
    if (m_udp_fd < 0 || m_udp_port <= 0) return;

    std::ostringstream json;
    json << std::fixed << std::setprecision(2);
    json << "{"
         << "\"uptime\":" << m_metrics.uptime_seconds << ","
         << "\"speed_ms\":" << m_metrics.current_speed_ms << ","
         << "\"speed_pct\":" << m_metrics.target_speed_percent << ","
         << "\"accel_ms2\":" << m_metrics.current_accel_ms2 << ","
         << "\"steer_deg\":" << m_metrics.target_steering_deg << ","
         << "\"steer_offset_deg\":" << m_metrics.steering_offset_deg << ","
         << "\"dist_cm\":" << m_metrics.filtered_dist_cm << ","
         << "\"odometry_m\":" << m_metrics.total_distance_m << ","
         << "\"fsm_state\":\"" << state_to_string(m_metrics.fsm_state) << "\","
         << "\"line\":["
         << (int)m_metrics.line_sensors[0] << ","
         << (int)m_metrics.line_sensors[1] << ","
         << (int)m_metrics.line_sensors[2] << ","
         << (int)m_metrics.line_sensors[3] << ","
         << (int)m_metrics.line_sensors[4] << "],"
         << "\"heartbeat\":" << m_metrics.heartbeat << ","
         << "\"emergency\":" << (m_metrics.emergency_stop ? "true" : "false") << ","
         << "\"violations\":" << m_metrics.accel_violations
         << "}\n";

    std::string payload = json.str();

    struct sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(m_udp_port);
    dest_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    sendto(m_udp_fd, payload.data(), payload.size(), 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
}

void TelemetryService::render_dashboard() {
    std::string line_bar = "[";
    for (int i = 0; i < 5; ++i) {
        line_bar += m_metrics.line_sensors[i] ? "█" : "·";
        if (i < 4) line_bar += " ";
    }
    line_bar += "]";

    std::string accel_status = (std::abs(m_metrics.current_accel_ms2) <= 0.735f) ? "OK" : "ALERTE !";
    std::string watchdog_str = m_metrics.watchdog_stalled ? "STALLED !" : "ACTIF";

    std::cout << "\r\033[K"
              << "[TELEM] "
              << "FSM: " << std::left << std::setw(20) << state_to_string(m_metrics.fsm_state)
              << " | V: " << std::fixed << std::setprecision(2) << m_metrics.current_speed_ms << " m/s"
              << " | A: " << m_metrics.current_accel_ms2 << " m/s² (" << accel_status << ")"
              << " | Steer: " << m_metrics.target_steering_deg << "° (" << m_metrics.steering_offset_deg << "°)"
              << " | US: " << m_metrics.filtered_dist_cm << " cm"
              << " | Line: " << line_bar
              << " | Odo: " << m_metrics.total_distance_m << " m"
              << " | WD: " << watchdog_str
              << std::flush;
}

void TelemetryService::run() {
    const auto period = std::chrono::milliseconds(1000 / m_hz);
    const float dt = 1.0f / static_cast<float>(m_hz);

    uint64_t loop_count = 0;
    std::cout << "[TelemetryService] Started hub running at " << m_hz << " Hz." << std::endl;

    while (m_running.load()) {
        auto start_time = std::chrono::steady_clock::now();

        // 1. Mise à jour immédiate des métriques et agrégation SHM au temps N
        update_metrics(dt);
        publish_ipc();
        broadcast_udp();

        // 2. Déclenchement instantané de l'envoi TCP au client (temps N, zéro délai superflu)
        if (m_client_handshaked.load()) {
            m_new_data_available.store(true);
            m_stream_cv.notify_one();
        }

        if (m_enable_ui && (loop_count % m_ui_render_divider == 0)) {
            render_dashboard();
        }

        loop_count++;

        // 3. Réception synchrone au temps N : réveil instantané dès qu'un service publie
        uint32_t wait_timeout_ms = static_cast<uint32_t>(1000 / m_hz);
        bool woke_by_shm = false;
        if (m_sensor_sub && m_sensor_sub->isConnected()) {
            woke_by_shm = m_sensor_sub->waitForUpdate(wait_timeout_ms);
        } else if (m_fsm_sub && m_fsm_sub->isConnected()) {
            woke_by_shm = m_fsm_sub->waitForUpdate(wait_timeout_ms);
        }

        if (!woke_by_shm) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed < period) {
                std::this_thread::sleep_for(period - elapsed);
            }
        }
    }

    m_stream_cv.notify_all();
    std::cout << "\n[TelemetryService] Service loop terminated." << std::endl;
}

void TelemetryService::stop() {
    if (m_running.load()) {
        m_running.store(false);
    }
    m_stream_cv.notify_all();
    close_tcp_server();
    if (m_tcp_thread.joinable()) {
        m_tcp_thread.join();
    }
    if (m_csv_samples_count > 0) {
        if (m_was_running_course) {
            export_to_usb();
            m_was_running_course = false;
        }
        if (m_csv_stream.is_open()) {
            m_csv_stream.flush();
            m_csv_stream.close();
        }
    }
}

void TelemetryService::log_csv_sample() {
    if (!m_csv_stream.is_open()) {
        m_csv_stream.open(m_local_csv_path, std::ios::out | std::ios::app);
        if (!m_csv_stream.is_open()) return;
    }

    auto now = std::chrono::steady_clock::now().time_since_epoch();
    uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();

    m_csv_stream << now_ms << ","
                 << m_metrics.uptime_seconds << ","
                 << state_to_string(m_metrics.fsm_state) << ","
                 << std::fixed << std::setprecision(3)
                 << m_metrics.current_speed_ms << ","
                 << m_metrics.current_accel_ms2 << ","
                 << std::setprecision(1)
                 << m_metrics.target_steering_deg << ","
                 << m_metrics.steering_offset_deg << ","
                 << m_metrics.filtered_dist_cm << ","
                 << static_cast<int>(m_metrics.line_sensors[0]) << ","
                 << static_cast<int>(m_metrics.line_sensors[1]) << ","
                 << static_cast<int>(m_metrics.line_sensors[2]) << ","
                 << static_cast<int>(m_metrics.line_sensors[3]) << ","
                 << static_cast<int>(m_metrics.line_sensors[4]) << ","
                 << std::setprecision(2)
                 << m_metrics.total_distance_m << ","
                 << (m_metrics.emergency_stop ? 1 : 0) << ","
                 << m_metrics.accel_violations << "\n";

    m_csv_samples_count++;
    if (m_csv_samples_count % 20 == 0) {
        m_csv_stream.flush();
    }
}

std::string TelemetryService::detect_usb_mount_point() {
    // 1. Analyse de /proc/mounts pour détecter une clé USB montée automatiquement
    std::ifstream mounts("/proc/mounts");
    if (mounts.is_open()) {
        std::string line;
        while (std::getline(mounts, line)) {
            std::istringstream iss(line);
            std::string device, mount_point, fs_type;
            if (iss >> device >> mount_point >> fs_type) {
                if (mount_point == "/" || mount_point == "/mnt/wsl" || mount_point.rfind("/mnt/wslg", 0) == 0) {
                    continue;
                }
                if ((mount_point.rfind("/media/", 0) == 0 || mount_point.rfind("/mnt/", 0) == 0 ||
                     device.rfind("/dev/sd", 0) == 0) && mount_point != "/mnt") {
#if defined(_WIN32)
                    return mount_point;
#else
                    if (access(mount_point.c_str(), W_OK) == 0) {
                        return mount_point;
                    }
#endif
                }
            }
        }
    }

    // 2. Dossiers standards Raspberry Pi / Debian
    const std::vector<std::string> standard_paths = {
        "/media/usb", "/mnt/usb", "/media/pi", "/media/picar"
    };
    for (const auto& p : standard_paths) {
#if defined(_WIN32)
        return p;
#else
        if (access(p.c_str(), W_OK) == 0) {
            return p;
        }
#endif
    }

    return "";
}

bool TelemetryService::export_to_usb() {
    if (m_csv_stream.is_open()) {
        m_csv_stream.flush();
    }

    std::string usb_mount = detect_usb_mount_point();
    if (usb_mount.empty()) {
        std::cout << "[TelemetryService] Clé USB non détectée. Les données restent sauvegardées dans " 
                  << m_local_csv_path << std::endl;
        return false;
    }

    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm{};
#if defined(_WIN32)
    localtime_s(&now_tm, &now_c);
#else
    localtime_r(&now_c, &now_tm);
#endif
    char time_str[32];
    std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", &now_tm);

    std::string dest_path = usb_mount + "/telemetry_run_" + std::string(time_str) + ".csv";

    std::ifstream src(m_local_csv_path, std::ios::binary);
    if (!src.is_open()) {
        std::cerr << "[TelemetryService] Erreur : impossible d'ouvrir " << m_local_csv_path << std::endl;
        return false;
    }

    std::ofstream dst(dest_path, std::ios::binary);
    if (!dst.is_open()) {
        std::cerr << "[TelemetryService] Erreur : impossible d'écrire sur la clé USB à " << dest_path << std::endl;
        return false;
    }

    dst << src.rdbuf();
    dst.flush();
    if (!dst.good()) {
        std::cerr << "[TelemetryService] Erreur : écriture incomplète sur la clé USB (espace disque insuffisant ?)" << std::endl;
        dst.close();
        src.close();
        return false;
    }
    dst.close();
    src.close();

#if !defined(_WIN32)
    sync();
#endif

    std::cout << "\n==================================================================" << std::endl;
    std::cout << "[TelemetryService] ✓ SUCCÈS : Télémétrie exportée sur la clé USB !" << std::endl;
    std::cout << "[TelemetryService] Fichier : " << dest_path << std::endl;
    std::cout << "[TelemetryService] La clé USB peut être retirée en toute sécurité." << std::endl;
    std::cout << "==================================================================\n" << std::endl;

    return true;
}

} // namespace services
