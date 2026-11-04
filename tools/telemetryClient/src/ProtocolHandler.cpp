#include "ProtocolHandler.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cstring>

namespace telemetry {

ProtocolHandler::ProtocolHandler() = default;
ProtocolHandler::~ProtocolHandler() = default;

bool ProtocolHandler::perform_handshake(TcpClient& client, uint16_t seq_num) {
    if (!client.is_connected()) return false;

    std::cout << "[ProtocolHandler] Envoi du Ping (seq=" << seq_num << ") pour initialiser le handshake..." << std::endl;

    // 1. Envoi du Ping
    shared::headers::PacketHeader ping_hdr{};
    ping_hdr.type = shared::headers::HeadersType::Command;
    ping_hdr.headerFlag = shared::headers::flags::REPLY_REQUESTED | shared::headers::flags::CRC_ENABLED;
    ping_hdr.seqNumber = seq_num;
    uint32_t ping_cmd = static_cast<uint32_t>(shared::commands::CommandID::Ping);
    ping_hdr.payloadLength = sizeof(ping_cmd);

    auto ping_pkt = shared::headers::serializePacket(
        ping_hdr,
        reinterpret_cast<const uint8_t*>(&ping_cmd),
        sizeof(ping_cmd)
    );

    if (!client.send_bytes(ping_pkt.data(), ping_pkt.size())) {
        std::cerr << "[ProtocolHandler] Échec d'envoi du paquet Ping." << std::endl;
        return false;
    }

    // 2. Attente de la réponse Pong
    size_t expected_pong_len = sizeof(shared::headers::PacketHeader) + sizeof(uint32_t) + sizeof(shared::crc::PacketCRC);
    std::vector<uint8_t> rx_buffer(expected_pong_len);

    if (!client.receive_exact(rx_buffer.data(), rx_buffer.size(), 2000)) {
        std::cerr << "[ProtocolHandler] Timeout lors de l'attente du Pong." << std::endl;
        return false;
    }

    shared::headers::PacketHeader pong_hdr{};
    std::vector<uint8_t> payload;
    uint32_t pong_crc = 0;
    if (!shared::headers::deserializePacket(rx_buffer.data(), rx_buffer.size(), pong_hdr, payload, pong_crc)) {
        std::cerr << "[ProtocolHandler] Paquet Pong invalide ou CRC incorrect !" << std::endl;
        return false;
    }

    if (pong_hdr.type != shared::headers::HeadersType::Response || pong_hdr.seqNumber != seq_num || payload.size() != sizeof(uint32_t)) {
        std::cerr << "[ProtocolHandler] Réponse inattendue lors du handshake." << std::endl;
        return false;
    }

    uint32_t resp_id = 0;
    std::memcpy(&resp_id, payload.data(), sizeof(resp_id));
    if (resp_id != static_cast<uint32_t>(shared::responses::ResponseID::Pong)) {
        std::cerr << "[ProtocolHandler] Réponse non-Pong reçue : " << resp_id << std::endl;
        return false;
    }

    std::cout << "[ProtocolHandler] Handshake complété avec succès ! Flux de télémétrie déverrouillé." << std::endl;
    return true;
}

bool ProtocolHandler::read_telemetry_packet(TcpClient& client, shared::car::CarTelemetry& out_telem, uint16_t& out_seq, int timeout_ms) {
    if (!client.is_connected()) return false;

    // 1. Lire d'abord l'en-tête (8 octets)
    constexpr size_t HEADER_SIZE = sizeof(shared::headers::PacketHeader);
    uint8_t hdr_raw[HEADER_SIZE];

    if (!client.receive_exact(hdr_raw, HEADER_SIZE, timeout_ms)) {
        return false;
    }

    shared::headers::PacketHeader hdr{};
    std::memcpy(&hdr, hdr_raw, HEADER_SIZE);

    bool has_crc = (hdr.headerFlag & shared::headers::flags::CRC_ENABLED) != 0;
    size_t remaining_len = hdr.payloadLength + (has_crc ? sizeof(shared::crc::PacketCRC) : 0);

    // 2. Lire le payload restant et le CRC
    std::vector<uint8_t> full_pkt(HEADER_SIZE + remaining_len);
    std::memcpy(full_pkt.data(), hdr_raw, HEADER_SIZE);

    if (!client.receive_exact(full_pkt.data() + HEADER_SIZE, remaining_len, timeout_ms)) {
        return false;
    }

    // 3. Désérialiser et valider le CRC
    shared::headers::PacketHeader decoded_hdr{};
    std::vector<uint8_t> payload;
    uint32_t packet_crc = 0;

    if (!shared::headers::deserializePacket(full_pkt.data(), full_pkt.size(), decoded_hdr, payload, packet_crc)) {
        std::cerr << "[ProtocolHandler] Erreur de désérialisation ou rejet CRC du paquet de télémétrie." << std::endl;
        return false;
    }

    if (decoded_hdr.type != shared::headers::HeadersType::Telemetry) {
        // Paquet non télémétrique (ex: réponse de commande ou log)
        return false;
    }

    if (payload.size() < sizeof(shared::car::CarTelemetry)) {
        std::cerr << "[ProtocolHandler] Taille de payload de télémétrie insuffisante (" << payload.size() 
                  << " vs attendu " << sizeof(shared::car::CarTelemetry) << ")" << std::endl;
        return false;
    }

    std::memcpy(&out_telem, payload.data(), sizeof(shared::car::CarTelemetry));
    out_seq = decoded_hdr.seqNumber;
    return true;
}

bool ProtocolHandler::send_command(TcpClient& client, const std::string& cmd_str, uint16_t seq_num) {
    if (!client.is_connected() || cmd_str.empty()) return false;

    // Le protocole impose que payloadLength soit un multiple de 4
    size_t raw_len = cmd_str.size();
    size_t padded_len = ((raw_len + 3) / 4) * 4;

    std::vector<uint8_t> payload(padded_len, 0);
    std::memcpy(payload.data(), cmd_str.data(), raw_len);

    shared::headers::PacketHeader hdr{};
    hdr.type = shared::headers::HeadersType::Command;
    hdr.headerFlag = shared::headers::flags::CRC_ENABLED;
    hdr.seqNumber = seq_num;
    hdr.payloadLength = static_cast<uint32_t>(padded_len);

    auto pkt = shared::headers::serializePacket(hdr, payload.data(), payload.size());
    bool ok = client.send_bytes(pkt.data(), pkt.size());
    if (ok) {
        std::cout << "[ProtocolHandler] Commande transmise : \"" << cmd_str << "\" (seq=" << seq_num << ")" << std::endl;
    }
    return ok;
}

bool ProtocolHandler::send_ping(TcpClient& client, uint16_t seq_num) {
    if (!client.is_connected()) return false;

    shared::headers::PacketHeader hdr{};
    hdr.type = shared::headers::HeadersType::Command;
    hdr.headerFlag = shared::headers::flags::CRC_ENABLED;
    hdr.seqNumber = seq_num;
    uint32_t ping_cmd = static_cast<uint32_t>(shared::commands::CommandID::Ping);
    hdr.payloadLength = sizeof(ping_cmd);

    auto pkt = shared::headers::serializePacket(hdr, reinterpret_cast<const uint8_t*>(&ping_cmd), sizeof(ping_cmd));
    return client.send_bytes(pkt.data(), pkt.size());
}

std::string ProtocolHandler::fsm_state_to_string(shared::fsm::MachineStates state) {
    switch (state) {
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

std::string ProtocolHandler::telemetry_to_json(const shared::car::CarTelemetry& telem, uint16_t seq_num) {
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "{";
    ss << "\"timestamp\":" << now << ",";
    ss << "\"seq\":" << seq_num << ",";

    // FSM
    auto fsm_state = telem.fsm.currentState;
    ss << "\"fsm\":{";
    ss << "\"state_id\":" << static_cast<int>(fsm_state) << ",";
    ss << "\"state_name\":\"" << fsm_state_to_string(fsm_state) << "\"";
    ss << "},";

    // Sensors
    ss << "\"sensors\":{";
    ss << "\"ultrasonic\":{";
    ss << "\"raw_dist_cm\":" << telem.sensors.ultrasonicSensor.rawDistanceCm << ",";
    ss << "\"calc_dist_cm\":" << telem.sensors.ultrasonicSensor.calculatedDistance << ",";
    ss << "\"flags\":" << telem.sensors.ultrasonicSensor.flags;
    ss << "},";
    ss << "\"line_follower\":{";
    ss << "\"flags\":" << telem.sensors.lineFollower.flags << ",";
    ss << "\"sensors\":[";
    for (int i = 0; i < 5; ++i) {
        const auto& s = telem.sensors.lineFollower.sensorsData[i];
        ss << "{";
        ss << "\"raw\":" << s.rawADCValue << ",";
        ss << "\"normalized\":" << s.normalizedValue << ",";
        ss << "\"corrector\":" << s.correctorValue;
        ss << "}" << (i < 4 ? "," : "");
    }
    ss << "]";
    ss << "}";
    ss << "},";

    // Motors & Steering
    ss << "\"motors\":{";
    ss << "\"left\":{";
    ss << "\"target_throttle\":" << telem.motors.leftMotor.normalizedMotorThrottleTargetted << ",";
    ss << "\"applied_throttle\":" << telem.motors.leftMotor.normalizedMotorThrottleApplied << ",";
    ss << "\"encoder\":" << telem.motors.leftMotor.motorEncoder;
    ss << "},";
    ss << "\"right\":{";
    ss << "\"target_throttle\":" << telem.motors.rightMotor.normalizedMotorThrottleTargetted << ",";
    ss << "\"applied_throttle\":" << telem.motors.rightMotor.normalizedMotorThrottleApplied << ",";
    ss << "\"encoder\":" << telem.motors.rightMotor.motorEncoder;
    ss << "},";
    ss << "\"steering\":{";
    ss << "\"target_angle\":" << telem.motors.steeringServo.normalizedTargettedAngle << ",";
    ss << "\"current_angle\":" << telem.motors.steeringServo.normalizedCurrentAngle;
    ss << "}";
    ss << "},";

    // Services flags
    ss << "\"services\":{";
    ss << "\"flags\":" << telem.services.flags;
    ss << "}";

    ss << "}";
    return ss.str();
}

} // namespace telemetry
