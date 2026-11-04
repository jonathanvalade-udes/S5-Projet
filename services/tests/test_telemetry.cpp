#include <iostream>
#include <cassert>
#include <cmath>
#include <thread>
#include <chrono>
#include <cstring>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <dirent.h>
#include <sys/stat.h>

#include "ipc/shared_memory.hpp"
#include "../TelemetryService/inc/TelemetryService.hpp"
#include "header/header.hpp"
#include "crc/crc_global.hpp"
#include "commands/global_commands.hpp"
#include "responses/global_responses.hpp"
#include "car/full_packet.hpp"

static bool approx_equal(float a, float b, float eps = 0.02f) {
    return std::abs(a - b) <= eps;
}

void test_telemetry_metrics_and_watchdog() {
    std::cout << "[TEST] TelemetryService - Agrégation, Odométrie et Watchdog..." << std::endl;

    SharedMemoryManager shm_mgr;
    RobotTelemetry* telem = shm_mgr.open_shm(true);
    assert(telem != nullptr);

    // Initialisation SHM
    pthread_mutex_lock(&(telem->mtx));
    telem->filtered_dist = 42.5f;
    telem->target_speed = 50.0f;     // 50% = 0.32 m/s
    telem->target_steering = -12.0f; // Braquage -12°
    telem->steering_offset = 3.5f;   // Trim offset +3.5°
    telem->state = 3;               // LineFollowingForward
    telem->emergency_stop = false;
    telem->heartbeat = 100;
    telem->line_sensors[0] = 0;
    telem->line_sensors[1] = 1;
    telem->line_sensors[2] = 1;
    telem->line_sensors[3] = 0;
    telem->line_sensors[4] = 0;
    pthread_mutex_unlock(&(telem->mtx));

    // Instance de TelemetryService à 50 Hz, sans affichage UI console, tcp_port=0 (désactivé)
    services::TelemetryService telem_service(50, false, 0, 0);
    assert(telem_service.init());

    // Lance le service en arrière-plan
    std::thread th([&]() {
        telem_service.run();
    });

    // Laisse tourner pendant 250 ms (5 cycles à 20 Hz)
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    const auto& metrics = telem_service.get_metrics();
    assert(approx_equal(metrics.filtered_dist_cm, 42.5f));
    assert(approx_equal(metrics.target_speed_percent, 50.0f));
    assert(approx_equal(metrics.current_speed_ms, 0.32f));
    assert(approx_equal(metrics.target_steering_deg, -12.0f));
    assert(approx_equal(metrics.steering_offset_deg, 3.5f));
    assert(metrics.fsm_state == 3);
    assert(metrics.line_sensors[1] == 1 && metrics.line_sensors[2] == 1);
    std::cout << "  ✓ Agrégation des capteurs et états SHM validée." << std::endl;

    // Test Odométrie : après ~0.25s à 0.32 m/s, d ~= 0.08 m
    assert(metrics.total_distance_m > 0.05f && metrics.total_distance_m < 0.15f);
    std::cout << "  ✓ Intégration odométrique validée (d = " << metrics.total_distance_m << " m)." << std::endl;

    // Test Watchdog : heartbeat constant depuis > 500 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    const auto& metrics_stalled = telem_service.get_metrics();
    assert(metrics_stalled.watchdog_stalled == true);
    std::cout << "  ✓ Watchdog : Stagnation heartbeat correctement détectée (> 500 ms)." << std::endl;

    // Réanimation du Watchdog
    pthread_mutex_lock(&(telem->mtx));
    telem->heartbeat = 200;
    pthread_mutex_unlock(&(telem->mtx));

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    assert(telem_service.get_metrics().watchdog_stalled == false);
    std::cout << "  ✓ Watchdog : Réarmement après reprise du heartbeat validé." << std::endl;

    telem_service.stop();
    if (th.joinable()) th.join();
}

void test_telemetry_acceleration_alert() {
    std::cout << "[TEST] TelemetryService - Détection d'accélération critique (> 0.735 m/s²)..." << std::endl;

    SharedMemoryManager shm_mgr;
    RobotTelemetry* telem = shm_mgr.open_shm(true);
    assert(telem != nullptr);

    pthread_mutex_lock(&(telem->mtx));
    telem->target_speed = 0.0f;
    telem->heartbeat = 1;
    pthread_mutex_unlock(&(telem->mtx));

    services::TelemetryService telem_service(50, false, 0, 0);
    assert(telem_service.init());

    std::thread th([&]() {
        telem_service.run();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Injection d'une accélération brutale (changement instantané de 0% à 100% en un pas dt = 50ms)
    // dv = 0.64 m/s en 0.05s => a = 12.8 m/s² >> 0.735 m/s²
    pthread_mutex_lock(&(telem->mtx));
    telem->target_speed = 100.0f;
    telem->heartbeat++;
    pthread_mutex_unlock(&(telem->mtx));

    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    const auto& metrics = telem_service.get_metrics();
    assert(metrics.accel_violations > 0);
    std::cout << "  ✓ Alerte accélération critique : Violation détectée (" 
              << metrics.accel_violations << " violations, a = " 
              << metrics.current_accel_ms2 << " m/s²)." << std::endl;

    telem_service.stop();
    if (th.joinable()) th.join();
}

void test_packet_serialization_and_crc() {
    std::cout << "[TEST] PacketHeader & CRC32 - Sérialisation, Validation et Détection d'Erreur..." << std::endl;

    // 1. Validation de l'algorithme CRC32 contre le vecteur de test IEEE 802.3 ("123456789" = 0xCBF43926)
    const char* test_vector = "123456789";
    auto crc_res = shared::crc::crcCalc(reinterpret_cast<const uint8_t*>(test_vector), 9);
    assert(crc_res.value == 0xCBF43926);
    std::cout << "  ✓ Algorithme CRC32 conforme au standard IEEE 802.3 (0xCBF43926)." << std::endl;

    // 2. Sérialisation d'un paquet de commande avec CRC
    shared::headers::PacketHeader hdr{};
    hdr.type = shared::headers::HeadersType::Command;
    hdr.headerFlag = shared::headers::flags::REPLY_REQUESTED | shared::headers::flags::CRC_ENABLED;
    hdr.seqNumber = 1234;
    uint32_t payload_val = static_cast<uint32_t>(shared::commands::CommandID::Ping);
    hdr.payloadLength = sizeof(payload_val);

    auto serialized = shared::headers::serializePacket(
        hdr,
        reinterpret_cast<const uint8_t*>(&payload_val),
        sizeof(payload_val)
    );

    assert(serialized.size() == sizeof(shared::headers::PacketHeader) + sizeof(payload_val) + sizeof(shared::crc::PacketCRC));

    // 3. Désérialisation et validation du paquet
    shared::headers::PacketHeader out_hdr{};
    std::vector<uint8_t> out_payload;
    uint32_t out_crc = 0;
    bool ok = shared::headers::deserializePacket(serialized.data(), serialized.size(), out_hdr, out_payload, out_crc);
    assert(ok);
    assert(out_hdr.type == shared::headers::HeadersType::Command);
    assert(out_hdr.seqNumber == 1234);
    assert(out_payload.size() == 4);
    uint32_t rec_val = 0;
    std::memcpy(&rec_val, out_payload.data(), 4);
    assert(rec_val == static_cast<uint32_t>(shared::commands::CommandID::Ping));
    std::cout << "  ✓ Sérialisation et désérialisation du paquet validées." << std::endl;

    // 4. Altération d'un octet pour tester le rejet CRC
    serialized[sizeof(shared::headers::PacketHeader)] ^= 0xFF; // corruption du payload
    bool corrupted_ok = shared::headers::deserializePacket(serialized.data(), serialized.size(), out_hdr, out_payload, out_crc);
    assert(!corrupted_ok);
    std::cout << "  ✓ Rejet automatique des paquets altérés via CRC32 validé." << std::endl;
}

void test_tcp_server_ping_pong_and_stream() {
    std::cout << "[TEST] TelemetryService - Serveur TCP, Handshake Ping-Pong et Stream de Télémétrie..." << std::endl;

    // Préparer des données de capteurs dans la SHM
    SharedMemoryManager shm_mgr;
    RobotTelemetry* telem = shm_mgr.open_shm(true);
    assert(telem != nullptr);

    pthread_mutex_lock(&(telem->mtx));
    telem->filtered_dist = 55.0f;
    telem->target_speed = 40.0f;
    telem->target_steering = 15.0f;
    telem->state = 3;
    telem->heartbeat = 42;
    telem->line_sensors[2] = 1;
    pthread_mutex_unlock(&(telem->mtx));

    // Utiliser un port de test TCP unprivileged
    const int test_port = 9191;
    services::TelemetryService telem_service(50, false, test_port, 0);
    assert(telem_service.init());

    std::thread th([&]() {
        telem_service.run();
    });

    // Attendre que le serveur écoute
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Connexion du client TCP au serveur
    int client_sock = socket(AF_INET, SOCK_STREAM, 0);
    assert(client_sock >= 0);

    struct sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(test_port);
    serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    int conn_res = connect(client_sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    assert(conn_res == 0);
    std::cout << "  ✓ Client TCP connecté avec succès au serveur (port " << test_port << ")." << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    assert(telem_service.is_client_connected());
    assert(!telem_service.is_client_handshaked());

    // 1. Envoyer le message de Ping depuis le client vers le serveur
    shared::headers::PacketHeader ping_hdr{};
    ping_hdr.type = shared::headers::HeadersType::Command;
    ping_hdr.headerFlag = shared::headers::flags::REPLY_REQUESTED | shared::headers::flags::CRC_ENABLED;
    ping_hdr.seqNumber = 999;
    uint32_t ping_cmd = static_cast<uint32_t>(shared::commands::CommandID::Ping);
    ping_hdr.payloadLength = sizeof(ping_cmd);

    auto ping_pkt = shared::headers::serializePacket(
        ping_hdr,
        reinterpret_cast<const uint8_t*>(&ping_cmd),
        sizeof(ping_cmd)
    );

    ssize_t sent = send(client_sock, ping_pkt.data(), ping_pkt.size(), 0);
    assert(sent == static_cast<ssize_t>(ping_pkt.size()));
    std::cout << "  ✓ Message Ping envoyé au serveur (seq=999)." << std::endl;

    // 2. Attendre et lire la réponse Pong du serveur
    struct pollfd pfd{};
    pfd.fd = client_sock;
    pfd.events = POLLIN;
    int p_ret = poll(&pfd, 1, 1000);
    assert(p_ret > 0);

    size_t expected_pong_len = sizeof(shared::headers::PacketHeader) + sizeof(uint32_t) + sizeof(shared::crc::PacketCRC);
    std::vector<uint8_t> pong_rx(expected_pong_len);
    size_t rx_total = 0;
    while (rx_total < expected_pong_len) {
        ssize_t r = recv(client_sock, pong_rx.data() + rx_total, expected_pong_len - rx_total, 0);
        assert(r > 0);
        rx_total += r;
    }

    shared::headers::PacketHeader pong_resp_hdr{};
    std::vector<uint8_t> pong_resp_payload;
    uint32_t pong_crc = 0;
    bool pong_ok = shared::headers::deserializePacket(pong_rx.data(), pong_rx.size(), pong_resp_hdr, pong_resp_payload, pong_crc);
    assert(pong_ok);
    assert(pong_resp_hdr.type == shared::headers::HeadersType::Response);
    assert(pong_resp_hdr.seqNumber == 999); // Numéro de séquence identique
    assert(pong_resp_payload.size() == sizeof(uint32_t));
    uint32_t resp_id = 0;
    std::memcpy(&resp_id, pong_resp_payload.data(), 4);
    assert(resp_id == static_cast<uint32_t>(shared::responses::ResponseID::Pong));
    std::cout << "  ✓ Réponse Pong reçue et validée (seq=999, CRC OK, ResponseID::Pong)." << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    assert(telem_service.is_client_handshaked());
    std::cout << "  ✓ Poignée de main (Handshake) complétée avec succès !" << std::endl;

    // 3. Lire le flux de données en continu (streaming de télémétrie)
    size_t expected_telem_len = sizeof(shared::headers::PacketHeader) +
                                sizeof(shared::car::CarTelemetry) +
                                sizeof(shared::crc::PacketCRC);

    std::cout << "  ✓ Taille du paquet de télémétrie complet : " << expected_telem_len << " octets." << std::endl;

    // Recevoir 3 trames de télémétrie consécutives
    for (int frame = 0; frame < 3; ++frame) {
        p_ret = poll(&pfd, 1, 1000);
        assert(p_ret > 0);

        std::vector<uint8_t> telem_rx(expected_telem_len);
        rx_total = 0;
        while (rx_total < expected_telem_len) {
            ssize_t r = recv(client_sock, telem_rx.data() + rx_total, expected_telem_len - rx_total, 0);
            assert(r > 0);
            rx_total += r;
        }

        shared::headers::PacketHeader stream_hdr{};
        std::vector<uint8_t> stream_payload;
        uint32_t stream_crc = 0;
        bool stream_ok = shared::headers::deserializePacket(telem_rx.data(), telem_rx.size(), stream_hdr, stream_payload, stream_crc);
        assert(stream_ok);
        assert(stream_hdr.type == shared::headers::HeadersType::Telemetry);
        assert(stream_payload.size() == sizeof(shared::car::CarTelemetry));

        shared::car::CarTelemetry received_car_telem{};
        std::memcpy(&received_car_telem, stream_payload.data(), sizeof(received_car_telem));

        assert(approx_equal(received_car_telem.sensors.ultrasonicSensor.calculatedDistance, 55.0f));
        assert(approx_equal(received_car_telem.motors.leftMotor.normalizedMotorThrottleApplied, 0.4f));
        assert(received_car_telem.fsm.currentState == static_cast<shared::fsm::MachineStates>(3));
        assert((received_car_telem.services.flags & shared::services::flags::SOCKET_ENABLED) != 0);

        std::cout << "    [Frame " << frame + 1 << "] Seq=" << stream_hdr.seqNumber 
                  << " | Dist=" << received_car_telem.sensors.ultrasonicSensor.calculatedDistance << " cm"
                  << " | Throttle=" << received_car_telem.motors.leftMotor.normalizedMotorThrottleApplied * 100.0f << "%"
                  << " | CRC validé." << std::endl;
    }

    std::cout << "  ✓ Streaming de télémétrie en continu validé avec succès." << std::endl;

    close(client_sock);
    telem_service.stop();
    if (th.joinable()) th.join();
}

void test_usb_fifo_direct_streaming() {
    std::cout << "[TEST] TelemetryService - File FIFO et Streaming direct continu sur Clé USB (0 écriture carte SD)..." << std::endl;

    // Configurer un dossier de test simulant la clé USB
    const char* mock_usb = "/tmp/mock_usb";
#if !defined(_WIN32)
    mkdir(mock_usb, 0777);
    setenv("PICAR_USB_PATH", mock_usb, 1);
#else
    _mkdir(mock_usb);
    _putenv("PICAR_USB_PATH=/tmp/mock_usb");
#endif

    // S'assurer que /tmp/telemetry.csv n'existe pas
    unlink("/tmp/telemetry.csv");

    services::TelemetryService telem_service(20, false, 0, 0);
    assert(telem_service.init());

    std::thread th([&]() {
        telem_service.run();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    telem_service.stop();
    if (th.joinable()) th.join();

    // 1. Validation stricte : ZÉRO écriture sur carte SD (/tmp/telemetry.csv ne doit PAS exister)
    std::ifstream sd_file("/tmp/telemetry.csv");
    assert(!sd_file.is_open());
    std::cout << "  ✓ Sécurité carte SD : Aucune écriture locale effectuée." << std::endl;

    // 2. Validation Clé USB : Un fichier telemetry_run_*.csv a été écrit directement sur la clé
    bool found_usb_file = false;
    std::string found_path;
#if !defined(_WIN32)
    DIR* dir = opendir(mock_usb);
    if (dir) {
        struct dirent* ent;
        while ((ent = readdir(dir)) != nullptr) {
            std::string name(ent->d_name);
            if (name.find("telemetry_run_") != std::string::npos && name.find(".csv") != std::string::npos) {
                found_usb_file = true;
                found_path = std::string(mock_usb) + "/" + name;
                break;
            }
        }
        closedir(dir);
    }
#endif
    assert(found_usb_file);
    std::cout << "  ✓ Fichier USB créé directement : " << found_path << std::endl;

    // 3. Vérifier le contenu du fichier USB
    std::ifstream usb_file(found_path);
    assert(usb_file.is_open());
    std::string header_line;
    assert(std::getline(usb_file, header_line));
    assert(header_line.find("timestamp") != std::string::npos);
    assert(header_line.find("speed_ms") != std::string::npos);

    std::string sample_line;
    assert(std::getline(usb_file, sample_line));
    assert(!sample_line.empty());
    usb_file.close();

    std::cout << "  ✓ Entête et trames FIFO validées en continu sur la clé USB." << std::endl;

    // Nettoyage
    unlink(found_path.c_str());
    rmdir(mock_usb);
}

int main() {
    std::cout << "===============================================" << std::endl;
    std::cout << "   PiCar C++17 - Tests Unitaires Télémétrie   " << std::endl;
    std::cout << "===============================================" << std::endl;

    test_packet_serialization_and_crc();
    test_telemetry_metrics_and_watchdog();
    test_telemetry_acceleration_alert();
    test_tcp_server_ping_pong_and_stream();
    test_usb_fifo_direct_streaming();

    std::cout << "===============================================" << std::endl;
    std::cout << "   TOUS LES TESTS DE TÉLÉMÉTRIE ONT RÉUSSI !   " << std::endl;
    std::cout << "===============================================" << std::endl;

    return 0;
}
