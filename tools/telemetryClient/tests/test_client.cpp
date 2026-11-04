#include <iostream>
#include <cassert>
#include <thread>
#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>

#include "TcpClient.hpp"
#include "ProtocolHandler.hpp"
#include "WebServer.hpp"
#include "header/header.hpp"
#include "crc/crc_global.hpp"
#include "commands/global_commands.hpp"
#include "responses/global_responses.hpp"
#include "car/full_packet.hpp"

static const int TEST_PORT = 9192;

void mock_telemetry_server(std::atomic<bool>& server_running, std::atomic<bool>& handshake_done) {
    int sfd = socket(AF_INET, SOCK_STREAM, 0);
    assert(sfd >= 0);

    int opt = 1;
    setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(SO_REUSEPORT)
    setsockopt(sfd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr.sin_port = htons(TEST_PORT);

    assert(bind(sfd, (struct sockaddr*)&addr, sizeof(addr)) == 0);
    assert(listen(sfd, 1) == 0);

    struct sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);
    int cfd = accept(sfd, (struct sockaddr*)&client_addr, &client_len);
    assert(cfd >= 0);

    // 1. Attente du Ping
    size_t ping_len = sizeof(shared::headers::PacketHeader) + sizeof(uint32_t) + sizeof(shared::crc::PacketCRC);
    std::vector<uint8_t> rx(ping_len);
    size_t rec = 0;
    while (rec < ping_len) {
        ssize_t n = recv(cfd, rx.data() + rec, ping_len - rec, 0);
        assert(n > 0);
        rec += n;
    }

    shared::headers::PacketHeader rx_hdr{};
    std::vector<uint8_t> rx_payload;
    uint32_t rx_crc = 0;
    assert(shared::headers::deserializePacket(rx.data(), rx.size(), rx_hdr, rx_payload, rx_crc));
    assert(rx_hdr.type == shared::headers::HeadersType::Command);

    // 2. Envoi du Pong
    shared::headers::PacketHeader pong_hdr{};
    pong_hdr.type = shared::headers::HeadersType::Response;
    pong_hdr.headerFlag = shared::headers::flags::CRC_ENABLED;
    pong_hdr.seqNumber = rx_hdr.seqNumber;
    uint32_t pong_id = static_cast<uint32_t>(shared::responses::ResponseID::Pong);
    pong_hdr.payloadLength = sizeof(pong_id);

    auto pong_pkt = shared::headers::serializePacket(pong_hdr, reinterpret_cast<const uint8_t*>(&pong_id), sizeof(pong_id));
    assert(send(cfd, pong_pkt.data(), pong_pkt.size(), 0) == static_cast<ssize_t>(pong_pkt.size()));
    handshake_done.store(true);

    // 3. Envoi d'une trame de télémétrie
    shared::car::CarTelemetry telem{};
    telem.sensors.ultrasonicSensor.calculatedDistance = 37.5f;
    telem.sensors.ultrasonicSensor.rawDistanceCm = 38.0f;
    telem.motors.leftMotor.normalizedMotorThrottleApplied = 0.45f;
    telem.fsm.currentState = shared::fsm::MachineStates::LineFollowingForward;

    shared::headers::PacketHeader telem_hdr{};
    telem_hdr.type = shared::headers::HeadersType::Telemetry;
    telem_hdr.headerFlag = shared::headers::flags::CRC_ENABLED;
    telem_hdr.seqNumber = 42;
    telem_hdr.payloadLength = sizeof(telem);

    auto telem_pkt = shared::headers::serializePacket(telem_hdr, reinterpret_cast<const uint8_t*>(&telem), sizeof(telem));
    assert(send(cfd, telem_pkt.data(), telem_pkt.size(), 0) == static_cast<ssize_t>(telem_pkt.size()));

    // 4. Attente de la commande du client
    char cmd_buf[512];
    ssize_t cmd_n = recv(cfd, cmd_buf, sizeof(cmd_buf), 0);
    assert(cmd_n > 0);

    shared::headers::PacketHeader cmd_hdr{};
    std::vector<uint8_t> cmd_payload;
    uint32_t cmd_crc = 0;
    assert(shared::headers::deserializePacket(reinterpret_cast<const uint8_t*>(cmd_buf), cmd_n, cmd_hdr, cmd_payload, cmd_crc));
    std::string received_str(reinterpret_cast<const char*>(cmd_payload.data()));
    assert(received_str.find("trigger_start") != std::string::npos);

    close(cfd);
    close(sfd);
    server_running.store(false);
}

int main() {
    std::cout << "[TEST] Démarrage du test d'intégration TelemetryClient..." << std::endl;

    std::atomic<bool> server_running{true};
    std::atomic<bool> handshake_done{false};

    std::thread srv_thread([&]() {
        mock_telemetry_server(server_running, handshake_done);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Client
    telemetry::TcpClient client;
    assert(client.connect_to("127.0.0.1", TEST_PORT));
    assert(client.is_connected());
    std::cout << "  ✓ Connexion TCP réussie au serveur mock." << std::endl;

    telemetry::ProtocolHandler proto;
    assert(proto.perform_handshake(client, 1));
    assert(handshake_done.load());
    std::cout << "  ✓ Handshake Ping-Pong avec validation CRC32 validé." << std::endl;

    shared::car::CarTelemetry received_telem{};
    uint16_t seq = 0;
    assert(proto.read_telemetry_packet(client, received_telem, seq, 1000));
    assert(seq == 42);
    assert(received_telem.sensors.ultrasonicSensor.calculatedDistance > 37.0f);
    assert(received_telem.fsm.currentState == shared::fsm::MachineStates::LineFollowingForward);
    std::cout << "  ✓ Réception, désérialisation et CRC32 du paquet de télémétrie validés." << std::endl;

    // Test conversion JSON
    std::string json_str = telemetry::ProtocolHandler::telemetry_to_json(received_telem, seq);
    assert(json_str.find("\"state_name\":\"LineFollowingForward\"") != std::string::npos);
    assert(json_str.find("\"calc_dist_cm\":37.50") != std::string::npos);
    std::cout << "  ✓ Génération du flux JSON pour le dashboard Web validée." << std::endl;

    // Test envoi de commande
    assert(proto.send_command(client, "trigger_start", 200));
    std::cout << "  ✓ Transmission de commande avec padding multiple de 4 et CRC validée." << std::endl;

    if (srv_thread.joinable()) {
        srv_thread.join();
    }

    std::cout << "\n✅ Tous les tests unitaires et d'intégration du TelemetryClient sont VALIDÉS !" << std::endl;
    return 0;
}
