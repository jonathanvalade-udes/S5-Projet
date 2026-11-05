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
#include <limits>
#include <fstream>

#include "TcpClient.hpp"
#include "ProtocolHandler.hpp"
#include "WebServer.hpp"
#include "CsvLogger.hpp"
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
    handshake_done.store(true);
    assert(send(cfd, pong_pkt.data(), pong_pkt.size(), 0) == static_cast<ssize_t>(pong_pkt.size()));

    // 3. Envoi d'une trame de télémétrie complète avec registres et démons
    shared::car::CarTelemetry telem{};
    telem.fsm.currentState = shared::fsm::MachineStates::LineFollowingForward;
    telem.fsm.requestedState = shared::fsm::MachineStates::LineFollowingForward;
    telem.fsm.flags = 0x0201;
    telem.fsm.errorCode = 0;

    telem.sensors.ultrasonicSensor.calculatedDistance = 37.5f;
    telem.sensors.ultrasonicSensor.rawDistanceCm = 38.0f;
    telem.sensors.ultrasonicSensor.flags = 1;
    telem.sensors.ultrasonicSensor.errorCode = 0;

    telem.motors.state = shared::motors::MotorStates::Running;
    telem.motors.leftMotor.normalizedMotorThrottleApplied = 0.45f;
    telem.motors.leftMotor.motorEncoder = 1250;
    telem.motors.rightMotor.motorEncoder = 1248;
    telem.motors.steeringServo.currentCCR = 307;
    telem.motors.steeringServo.targetedCCR = 310;

    telem.services.deamonsLenght = 4;
    telem.services.daemons[0] = {1, 1, 0, 0};
    telem.services.daemons[1] = {2, 1, 0, 0};
    telem.services.daemons[2] = {3, 1, 0, 0};
    telem.services.daemons[3] = {4, 1, 0, 0};

    shared::headers::PacketHeader telem_hdr{};
    telem_hdr.type = shared::headers::HeadersType::Telemetry;
    telem_hdr.headerFlag = shared::headers::flags::CRC_ENABLED;
    telem_hdr.seqNumber = 42;
    telem_hdr.payloadLength = sizeof(telem);

    auto telem_pkt = shared::headers::serializePacket(telem_hdr, reinterpret_cast<const uint8_t*>(&telem), sizeof(telem));
    assert(send(cfd, telem_pkt.data(), telem_pkt.size(), 0) == static_cast<ssize_t>(telem_pkt.size()));

    // 4. Attente de la commande du client (SetActive trigger_start)
    char cmd_buf[512];
    ssize_t cmd_n = recv(cfd, cmd_buf, sizeof(cmd_buf), 0);
    assert(cmd_n > 0);

    shared::headers::PacketHeader cmd_hdr{};
    std::vector<uint8_t> cmd_payload;
    uint32_t cmd_crc = 0;
    assert(shared::headers::deserializePacket(reinterpret_cast<const uint8_t*>(cmd_buf), cmd_n, cmd_hdr, cmd_payload, cmd_crc));
    assert(cmd_hdr.type == shared::headers::HeadersType::Command);
    assert(cmd_payload.size() >= sizeof(shared::commands::CommandsHeader));

    const auto* ch = reinterpret_cast<const shared::commands::CommandsHeader*>(cmd_payload.data());
    assert(ch->serviceID == shared::commands::ServiceID::FSM);
    assert(ch->commandID == shared::commands::CommandID::SetActive);

    // 5. Attente de la commande SetConfigKey
    cmd_n = recv(cfd, cmd_buf, sizeof(cmd_buf), 0);
    assert(cmd_n > 0);
    assert(shared::headers::deserializePacket(reinterpret_cast<const uint8_t*>(cmd_buf), cmd_n, cmd_hdr, cmd_payload, cmd_crc));
    assert(cmd_payload.size() == sizeof(shared::commands::CommandsHeader) + sizeof(shared::commands::config::SetConfigPayload));
    ch = reinterpret_cast<const shared::commands::CommandsHeader*>(cmd_payload.data());
    assert(ch->serviceID == shared::commands::ServiceID::FSM);
    assert(ch->commandID == shared::commands::CommandID::SetConfigKey);

    const auto* cfg = reinterpret_cast<const shared::commands::config::SetConfigPayload*>(cmd_payload.data() + sizeof(shared::commands::CommandsHeader));
    assert(std::string(cfg->key) == "CURVE_SAFETY_FACTOR");
    assert(std::abs(cfg->value - 0.85f) < 0.001f);

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
    assert(received_telem.motors.leftMotor.motorEncoder == 1250);
    assert(received_telem.motors.steeringServo.currentCCR == 307);
    std::cout << "  ✓ Réception, désérialisation et CRC32 du paquet de télémétrie validés." << std::endl;

    // Test conversion JSON enrichie
    std::string json_str = telemetry::ProtocolHandler::telemetry_to_json(received_telem, seq);
    assert(json_str.find("\"state_name\":\"LineFollowingForward\"") != std::string::npos);
    assert(json_str.find("\"requested_state_name\":\"LineFollowingForward\"") != std::string::npos);
    assert(json_str.find("\"calc_dist_cm\":37.50") != std::string::npos);
    assert(json_str.find("\"encoder\":1250") != std::string::npos);
    assert(json_str.find("\"current_ccr\":307") != std::string::npos);
    assert(json_str.find("\"daemons\"") != std::string::npos);
    std::cout << "  ✓ Génération du flux JSON enrichi (registres, drapeaux, démons) validée." << std::endl;

    // Test conversion JSON avec valeurs NaN et Inf (sécurité RFC 8259)
    shared::car::CarTelemetry nan_telem = received_telem;
    nan_telem.sensors.ultrasonicSensor.calculatedDistance = std::numeric_limits<float>::quiet_NaN();
    nan_telem.sensors.ultrasonicSensor.rawDistanceCm = std::numeric_limits<float>::infinity();
    std::string nan_json = telemetry::ProtocolHandler::telemetry_to_json(nan_telem, 43);
    assert(nan_json.find("nan") == std::string::npos && nan_json.find("NaN") == std::string::npos);
    assert(nan_json.find("inf") == std::string::npos && nan_json.find("Infinity") == std::string::npos);
    assert(nan_json.find("\"calc_dist_cm\":0.00") != std::string::npos);
    std::cout << "  ✓ Protection contre les valeurs NaN / Inf dans le flux JSON validée." << std::endl;

    // Test CsvLogger
    {
        std::string test_csv = "test_telemetry.csv";
        unlink(test_csv.c_str());
        telemetry::CsvLogger logger;
        assert(logger.open(test_csv));
        assert(logger.is_open());
        logger.log_record(42, received_telem);
        logger.flush();
        assert(logger.get_record_count() == 1);
        logger.close();

        std::ifstream f(test_csv);
        assert(f.is_open());
        std::string header, data_line;
        std::getline(f, header);
        std::getline(f, data_line);
        assert(header.find("timestamp_ms,seq_number") != std::string::npos);
        assert(header.find("steering_current_ccr") != std::string::npos);
        assert(data_line.find(",42,") != std::string::npos);
        unlink(test_csv.c_str());
        std::cout << "  ✓ CsvLogger création d'en-tête, écriture de trames et flush validés." << std::endl;
    }

    // Test envoi de commandes conformes shared-headers avec CRC32
    assert(proto.send_command(client, "trigger_start", 200));
    assert(proto.send_set_config(client, shared::commands::ServiceID::FSM, "CURVE_SAFETY_FACTOR", 0.85f, 201));
    std::cout << "  ✓ Transmission de commandes binaires SetActive et SetConfigKey (CommandsHeader 72 o + CRC32) validée." << std::endl;

    if (srv_thread.joinable()) {
        srv_thread.join();
    }

    // Test WebServer diffusion non-bloquante avec SSE et route /api/telemetry et /api/export_csv
    {
        telemetry::WebServer web(8999, "web");
        web.set_csv_filepath("test_export.csv");
        {
            std::ofstream f("test_export.csv");
            f << "test,csv,data\n1,2,3\n";
        }

        assert(web.start());
        web.broadcast_telemetry("{\"test\":1}");
        web.broadcast_status(true, "127.0.0.1", 91);

        // Connexion client HTTP pour tester /api/telemetry
        int s = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in srv{};
        srv.sin_family = AF_INET;
        srv.sin_addr.s_addr = inet_addr("127.0.0.1");
        srv.sin_port = htons(8999);
        assert(connect(s, (struct sockaddr*)&srv, sizeof(srv)) == 0);
        std::string req = "GET /api/telemetry HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n";
        send(s, req.data(), req.size(), 0);
        char buf[512];
        ssize_t n = recv(s, buf, sizeof(buf) - 1, 0);
        assert(n > 0);
        buf[n] = '\0';
        std::string resp(buf);
        assert(resp.find("200 OK") != std::string::npos);
        assert(resp.find("{\"test\":1}") != std::string::npos);
        close(s);

        // Connexion client HTTP pour tester /api/export_csv
        int s2 = socket(AF_INET, SOCK_STREAM, 0);
        assert(connect(s2, (struct sockaddr*)&srv, sizeof(srv)) == 0);
        std::string req_csv = "GET /api/export_csv HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n";
        send(s2, req_csv.data(), req_csv.size(), 0);
        char buf2[512];
        ssize_t n2 = recv(s2, buf2, sizeof(buf2) - 1, 0);
        assert(n2 > 0);
        buf2[n2] = '\0';
        std::string resp2(buf2);
        assert(resp2.find("200 OK") != std::string::npos);
        assert(resp2.find("text/csv") != std::string::npos);
        assert(resp2.find("test,csv,data") != std::string::npos);
        close(s2);
        unlink("test_export.csv");

        web.stop();
        std::cout << "  ✓ WebServer diffusion SSE, route /api/telemetry et /api/export_csv validés." << std::endl;
    }

    std::cout << "\n✅ Tous les tests unitaires et d'intégration du TelemetryClient sont VALIDÉS !" << std::endl;
    return 0;
}
