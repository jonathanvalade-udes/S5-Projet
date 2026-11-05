#include "ProtocolHandler.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <cctype>

namespace telemetry {

ProtocolHandler::ProtocolHandler() {
    m_rx_buffer.resize(4096, 0);
}

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

    // Validation stricte de l'en-tête pour éviter toute corruption ou fuite mémoire
    uint8_t type_val = static_cast<uint8_t>(hdr.type);
    if (type_val < 0x01 || type_val > 0x03) {
        std::cerr << "[ProtocolHandler] En-tête corrompu : type invalide (0x" 
                  << std::hex << static_cast<int>(type_val) << std::dec << "). Déconnexion pour resynchronisation." << std::endl;
        client.disconnect();
        return false;
    }

    if (hdr.payloadLength > 4096 || (hdr.payloadLength % 4 != 0)) {
        std::cerr << "[ProtocolHandler] En-tête corrompu : payloadLength invalide (" 
                  << hdr.payloadLength << "). Déconnexion pour resynchronisation." << std::endl;
        client.disconnect();
        return false;
    }

    bool has_crc = (hdr.headerFlag & shared::headers::flags::CRC_ENABLED) != 0;
    size_t remaining_len = hdr.payloadLength + (has_crc ? sizeof(shared::crc::PacketCRC) : 0);
    size_t total_pkt_len = HEADER_SIZE + remaining_len;

    if (total_pkt_len > 4096) {
        std::cerr << "[ProtocolHandler] Taille de paquet excessive (" << total_pkt_len << " octets). Déconnexion." << std::endl;
        client.disconnect();
        return false;
    }

    // 2. Utilisation du buffer réutilisable préalloué
    if (m_rx_buffer.size() < total_pkt_len) {
        m_rx_buffer.resize(4096, 0);
    }
    std::memcpy(m_rx_buffer.data(), hdr_raw, HEADER_SIZE);

    if (!client.receive_exact(m_rx_buffer.data() + HEADER_SIZE, remaining_len, timeout_ms)) {
        std::cerr << "[ProtocolHandler] Échec de réception du payload (" << remaining_len << " octets). Déconnexion pour resynchronisation." << std::endl;
        client.disconnect();
        return false;
    }

    // 3. Désérialiser et valider le CRC
    shared::headers::PacketHeader decoded_hdr{};
    std::vector<uint8_t> payload;
    uint32_t packet_crc = 0;

    if (!shared::headers::deserializePacket(m_rx_buffer.data(), total_pkt_len, decoded_hdr, payload, packet_crc)) {
        std::cerr << "[ProtocolHandler] Erreur de désérialisation ou rejet CRC du paquet. Déconnexion pour resynchronisation." << std::endl;
        client.disconnect();
        return false;
    }

    if (decoded_hdr.type != shared::headers::HeadersType::Telemetry) {
        // Paquet non télémétrique (ex: Pong keep-alive reçu et absorbé)
        return false;
    }

    if (payload.size() < sizeof(shared::car::CarTelemetry)) {
        std::cerr << "[ProtocolHandler] Taille de payload de télémétrie insuffisante (" << payload.size() 
                  << " vs attendu " << sizeof(shared::car::CarTelemetry) << "). Déconnexion." << std::endl;
        client.disconnect();
        return false;
    }

    std::memcpy(&out_telem, payload.data(), sizeof(shared::car::CarTelemetry));
    out_seq = decoded_hdr.seqNumber;
    return true;
}

shared::commands::ServiceID ProtocolHandler::string_to_service_id(const std::string& srv_str) {
    std::string s = srv_str;
    for (auto& c : s) c = static_cast<char>(::toupper(c));
    if (s == "MOTORS" || s == "MOTOR" || s == "M") return shared::commands::ServiceID::MOTORS;
    if (s == "SENSORS" || s == "SENSOR" || s == "S") return shared::commands::ServiceID::SENSORS;
    if (s == "TELEMETRY" || s == "TELEM" || s == "T") return shared::commands::ServiceID::TELEMETRY;
    return shared::commands::ServiceID::FSM;
}

bool ProtocolHandler::send_set_config(TcpClient& client, shared::commands::ServiceID srv,
                                      const std::string& key, float value, uint16_t seq_num) {
    if (!client.is_connected() || key.empty()) return false;

    shared::commands::CommandsHeader cmd_hdr{};
    cmd_hdr.serviceID = srv;
    cmd_hdr.commandID = shared::commands::CommandID::SetConfigKey;
    cmd_hdr.payloadLen = sizeof(shared::commands::config::SetConfigPayload);

    constexpr size_t TOTAL_PAYLOAD_LEN = sizeof(cmd_hdr) + sizeof(shared::commands::config::SetConfigPayload);
    std::vector<uint8_t> payload_buf(TOTAL_PAYLOAD_LEN, 0);
    std::memcpy(payload_buf.data(), &cmd_hdr, sizeof(cmd_hdr));

    char key_buf[shared::commands::config::MAX_KEY_LEN] = {0};
    std::strncpy(key_buf, key.c_str(), shared::commands::config::MAX_KEY_LEN - 1);
    std::memcpy(payload_buf.data() + sizeof(cmd_hdr), key_buf, shared::commands::config::MAX_KEY_LEN);
    std::memcpy(payload_buf.data() + sizeof(cmd_hdr) + shared::commands::config::MAX_KEY_LEN, &value, sizeof(float));

    shared::headers::PacketHeader pkt_hdr{};
    pkt_hdr.type = shared::headers::HeadersType::Command;
    pkt_hdr.headerFlag = shared::headers::flags::CRC_ENABLED | shared::headers::flags::REPLY_REQUESTED;
    pkt_hdr.seqNumber = seq_num;
    pkt_hdr.payloadLength = static_cast<uint32_t>(TOTAL_PAYLOAD_LEN);

    auto packet = shared::headers::serializePacket(pkt_hdr, payload_buf.data(), payload_buf.size());
    bool ok = client.send_bytes(packet.data(), packet.size());
    if (ok) {
        std::cout << "[ProtocolHandler] Config transmise (CRC32 validé): Srv=" << static_cast<int>(srv)
                  << " " << key << "=" << value << " (seq=" << seq_num << ")" << std::endl;
    }
    return ok;
}

bool ProtocolHandler::send_set_active(TcpClient& client, shared::commands::ServiceID srv, bool active, uint16_t seq_num) {
    if (!client.is_connected()) return false;

    shared::commands::CommandsHeader cmd_hdr{};
    cmd_hdr.serviceID = srv;
    cmd_hdr.commandID = shared::commands::CommandID::SetActive;
    cmd_hdr.payloadLen = sizeof(shared::commands::SetActivePayload);

    constexpr size_t TOTAL_PAYLOAD_LEN = sizeof(cmd_hdr) + sizeof(shared::commands::SetActivePayload);
    std::vector<uint8_t> payload_buf(TOTAL_PAYLOAD_LEN, 0);
    std::memcpy(payload_buf.data(), &cmd_hdr, sizeof(cmd_hdr));

    shared::commands::SetActivePayload act_payload{};
    act_payload.active = active ? 1 : 0;
    std::memcpy(payload_buf.data() + sizeof(cmd_hdr), &act_payload, sizeof(act_payload));

    shared::headers::PacketHeader pkt_hdr{};
    pkt_hdr.type = shared::headers::HeadersType::Command;
    pkt_hdr.headerFlag = shared::headers::flags::CRC_ENABLED | shared::headers::flags::REPLY_REQUESTED;
    pkt_hdr.seqNumber = seq_num;
    pkt_hdr.payloadLength = static_cast<uint32_t>(TOTAL_PAYLOAD_LEN);

    auto packet = shared::headers::serializePacket(pkt_hdr, payload_buf.data(), payload_buf.size());
    bool ok = client.send_bytes(packet.data(), packet.size());
    if (ok) {
        std::cout << "[ProtocolHandler] SetActive transmis (CRC32 validé): Srv=" << static_cast<int>(srv)
                  << " Active=" << (active ? "1" : "0") << " (seq=" << seq_num << ")" << std::endl;
    }
    return ok;
}

bool ProtocolHandler::send_emergency_stop(TcpClient& client, shared::commands::ServiceID srv, uint32_t reason_code, uint16_t seq_num) {
    if (!client.is_connected()) return false;

    shared::commands::CommandsHeader cmd_hdr{};
    cmd_hdr.serviceID = srv;
    cmd_hdr.commandID = shared::commands::CommandID::EmergencyStop;
    cmd_hdr.payloadLen = sizeof(shared::commands::EmergencyStopPayload);

    constexpr size_t TOTAL_PAYLOAD_LEN = sizeof(cmd_hdr) + sizeof(shared::commands::EmergencyStopPayload);
    std::vector<uint8_t> payload_buf(TOTAL_PAYLOAD_LEN, 0);
    std::memcpy(payload_buf.data(), &cmd_hdr, sizeof(cmd_hdr));

    shared::commands::EmergencyStopPayload estop_payload{};
    estop_payload.reason_code = reason_code;
    std::memcpy(payload_buf.data() + sizeof(cmd_hdr), &estop_payload, sizeof(estop_payload));

    shared::headers::PacketHeader pkt_hdr{};
    pkt_hdr.type = shared::headers::HeadersType::Command;
    pkt_hdr.headerFlag = shared::headers::flags::CRC_ENABLED | shared::headers::flags::REPLY_REQUESTED;
    pkt_hdr.seqNumber = seq_num;
    pkt_hdr.payloadLength = static_cast<uint32_t>(TOTAL_PAYLOAD_LEN);

    auto packet = shared::headers::serializePacket(pkt_hdr, payload_buf.data(), payload_buf.size());
    bool ok = client.send_bytes(packet.data(), packet.size());
    if (ok) {
        std::cout << "[ProtocolHandler] EmergencyStop transmis (CRC32 validé): Srv=" << static_cast<int>(srv)
                  << " Reason=" << reason_code << " (seq=" << seq_num << ")" << std::endl;
    }
    return ok;
}

bool ProtocolHandler::send_command(TcpClient& client, const std::string& cmd_str, uint16_t seq_num) {
    if (!client.is_connected() || cmd_str.empty()) return false;

    // 1. Détection des actions globales standard
    if (cmd_str == "trigger_start" || cmd_str == "start") {
        return send_set_active(client, shared::commands::ServiceID::FSM, true, seq_num);
    }
    if (cmd_str == "emergency_stop" || cmd_str == "stop") {
        // Le FSM doit etre prevenu en premier : il passe en Abort et maintient l'arret (puissance a 0 + drapeau d'urgence en
        // memoire partagee). Si seul Motors recevait l'ordre, il ne s'arreterait qu'un cycle : le FSM, qui ignore l'arret,
        // continuerait de commander les moteurs et le robot repartirait aussitot. Motors est prevenu aussi pour couper tout de suite.
        const bool fsm_ok = send_emergency_stop(client, shared::commands::ServiceID::FSM, 1, seq_num);
        const bool motors_ok = send_emergency_stop(client, shared::commands::ServiceID::MOTORS, 1,
                                                   static_cast<uint16_t>(seq_num + 1));
        return fsm_ok || motors_ok;
    }

    // 2. Détection de commandes de configuration cle=valeur (ex: "CURVE_SAFETY_FACTOR=0.75" ou "MOTORS.STEERING_OFFSET=1.5")
    size_t eq_pos = cmd_str.find('=');
    if (eq_pos != std::string::npos) {
        std::string full_key = cmd_str.substr(0, eq_pos);
        std::string val_str = cmd_str.substr(eq_pos + 1);

        full_key.erase(std::remove_if(full_key.begin(), full_key.end(), ::isspace), full_key.end());
        val_str.erase(std::remove_if(val_str.begin(), val_str.end(), ::isspace), val_str.end());

        shared::commands::ServiceID target_srv = shared::commands::ServiceID::FSM;
        std::string param_key = full_key;

        size_t dot_pos = full_key.find('.');
        if (dot_pos != std::string::npos) {
            std::string srv_part = full_key.substr(0, dot_pos);
            param_key = full_key.substr(dot_pos + 1);
            target_srv = string_to_service_id(srv_part);
        } else {
            // Heuristique de service selon la clé
            if (param_key == "STEERING_OFFSET" || param_key == "OFFSET" || param_key == "INVERT") {
                target_srv = shared::commands::ServiceID::MOTORS;
            } else if (param_key == "FILTER_ALPHA" || param_key == "MEDIAN_WINDOW") {
                target_srv = shared::commands::ServiceID::SENSORS;
            } else if (param_key == "UDP_PORT") {
                target_srv = shared::commands::ServiceID::TELEMETRY;
            }
        }

        try {
            float fval = std::stof(val_str);
            return send_set_config(client, target_srv, param_key, fval, seq_num);
        } catch (...) {
            std::cerr << "[ProtocolHandler] Échec de conversion de valeur pour: " << cmd_str << std::endl;
        }
    }

    // 3. Fallback texte avec CRC_ENABLED pour d'autres commandes libres
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
        std::cout << "[ProtocolHandler] Commande brute transmise (CRC32 validé): \"" << cmd_str << "\" (seq=" << seq_num << ")" << std::endl;
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
        case shared::fsm::MachineStates::RightDeviation: return "RightDeviation";
        case shared::fsm::MachineStates::LeftDeviation: return "LeftDeviation";
        case shared::fsm::MachineStates::Abort: return "Abort";
        case shared::fsm::MachineStates::Failure: return "Failure";
        default: return "Undefined";
    }
}

std::string ProtocolHandler::motor_state_to_string(shared::motors::MotorStates state) {
    switch (state) {
        case shared::motors::MotorStates::Init: return "Init";
        case shared::motors::MotorStates::Idle: return "Idle";
        case shared::motors::MotorStates::Running: return "Running";
        case shared::motors::MotorStates::Abort: return "Abort";
        default: return "Undefined";
    }
}

static inline float safe_float(float val, float fallback = 0.0f) {
    if (std::isnan(val) || std::isinf(val)) {
        return fallback;
    }
    return val;
}

std::string ProtocolHandler::telemetry_to_json(const shared::car::CarTelemetry& telem, uint16_t seq_num) {
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "{";
    ss << "\"timestamp\":" << now << ",";
    ss << "\"seq\":" << seq_num << ",";

    // 1. FSM
    auto fsm_cur = telem.fsm.currentState;
    auto fsm_req = telem.fsm.requestedState;
    ss << "\"fsm\":{";
    ss << "\"state_id\":" << static_cast<int>(fsm_cur) << ",";
    ss << "\"state_name\":\"" << fsm_state_to_string(fsm_cur) << "\",";
    ss << "\"requested_state_id\":" << static_cast<int>(fsm_req) << ",";
    ss << "\"requested_state_name\":\"" << fsm_state_to_string(fsm_req) << "\",";
    ss << "\"flags\":" << telem.fsm.flags << ",";
    ss << "\"error_code\":" << telem.fsm.errorCode;
    ss << "},";

    // 2. Sensors
    ss << "\"sensors\":{";
    ss << "\"component_flags\":" << telem.sensors.flags << ",";
    ss << "\"component_error_code\":" << telem.sensors.errorCode << ",";
    ss << "\"ultrasonic\":{";
    ss << "\"raw_dist_cm\":" << safe_float(telem.sensors.ultrasonicSensor.rawDistanceCm) << ",";
    ss << "\"calc_dist_cm\":" << safe_float(telem.sensors.ultrasonicSensor.calculatedDistance) << ",";
    ss << "\"flags\":" << telem.sensors.ultrasonicSensor.flags << ",";
    ss << "\"error_code\":" << telem.sensors.ultrasonicSensor.errorCode;
    ss << "},";
    ss << "\"line_follower\":{";
    ss << "\"flags\":" << telem.sensors.lineFollower.flags << ",";
    ss << "\"sensors\":[";
    for (int i = 0; i < 5; ++i) {
        const auto& s = telem.sensors.lineFollower.sensorsData[i];
        ss << "{";
        ss << "\"raw\":" << s.rawADCValue << ",";
        ss << "\"normalized\":" << safe_float(s.normalizedValue) << ",";
        ss << "\"corrector\":" << safe_float(s.correctorValue) << ",";
        ss << "\"error_code\":" << s.errorCode;
        ss << "}" << (i < 4 ? "," : "");
    }
    ss << "]";
    ss << "}";
    ss << "},";

    // 3. Motors
    ss << "\"motors\":{";
    ss << "\"state_id\":" << static_cast<int>(telem.motors.state) << ",";
    ss << "\"state_name\":\"" << motor_state_to_string(telem.motors.state) << "\",";
    ss << "\"component_flags\":" << telem.motors.componentFlags << ",";
    ss << "\"error_code\":" << telem.motors.errorCode << ",";

    // Left Motor
    const auto& ml = telem.motors.leftMotor;
    ss << "\"left\":{";
    ss << "\"target_throttle\":" << safe_float(ml.normalizedMotorThrottleTargetted) << ",";
    ss << "\"applied_throttle\":" << safe_float(ml.normalizedMotorThrottleApplied) << ",";
    ss << "\"encoder\":" << ml.motorEncoder << ",";
    ss << "\"motor_flags\":" << static_cast<int>(ml.motorFlags) << ",";
    ss << "\"pid_flags\":" << static_cast<int>(ml.pidFlags) << ",";
    ss << "\"pid\":{";
    ss << "\"kp\":" << safe_float(ml.pidInfo.powerCoefficient) << ",";
    ss << "\"kd\":" << safe_float(ml.pidInfo.derivativeCoefficient) << ",";
    ss << "\"ki\":" << safe_float(ml.pidInfo.integralCoefficient);
    ss << "}";
    ss << "},";

    // Right Motor
    const auto& mr = telem.motors.rightMotor;
    ss << "\"right\":{";
    ss << "\"target_throttle\":" << safe_float(mr.normalizedMotorThrottleTargetted) << ",";
    ss << "\"applied_throttle\":" << safe_float(mr.normalizedMotorThrottleApplied) << ",";
    ss << "\"encoder\":" << mr.motorEncoder << ",";
    ss << "\"motor_flags\":" << static_cast<int>(mr.motorFlags) << ",";
    ss << "\"pid_flags\":" << static_cast<int>(mr.pidFlags) << ",";
    ss << "\"pid\":{";
    ss << "\"kp\":" << safe_float(mr.pidInfo.powerCoefficient) << ",";
    ss << "\"kd\":" << safe_float(mr.pidInfo.derivativeCoefficient) << ",";
    ss << "\"ki\":" << safe_float(mr.pidInfo.integralCoefficient);
    ss << "}";
    ss << "},";

    // Steering Servo
    const auto& st = telem.motors.steeringServo;
    ss << "\"steering\":{";
    ss << "\"target_angle\":" << safe_float(st.normalizedTargettedAngle) << ",";
    ss << "\"current_angle\":" << safe_float(st.normalizedCurrentAngle) << ",";
    ss << "\"target_ccr\":" << st.targetedCCR << ",";
    ss << "\"current_ccr\":" << st.currentCCR << ",";
    ss << "\"coefficient\":" << safe_float(st.coefficient) << ",";
    ss << "\"flags\":" << st.flags << ",";
    ss << "\"error_code\":" << st.errorCode;
    ss << "}";
    ss << "},";

    // 4. Aux Servo
    const auto& sv = telem.servo;
    ss << "\"aux_servo\":{";
    ss << "\"target_angle\":" << safe_float(sv.normalizedTargettedAngle) << ",";
    ss << "\"current_angle\":" << safe_float(sv.normalizedCurrentAngle) << ",";
    ss << "\"target_ccr\":" << sv.targetedCCR << ",";
    ss << "\"current_ccr\":" << sv.currentCCR << ",";
    ss << "\"coefficient\":" << safe_float(sv.coefficient) << ",";
    ss << "\"flags\":" << sv.flags << ",";
    ss << "\"error_code\":" << sv.errorCode;
    ss << "},";

    // 5. Services & Daemons
    ss << "\"services\":{";
    ss << "\"flags\":" << telem.services.flags << ",";
    ss << "\"timestamp\":" << telem.services.timestamp << ",";
    ss << "\"handler_error_code\":" << telem.services.handlerErrorCode << ",";
    ss << "\"daemons_count\":" << telem.services.deamonsLenght << ",";
    ss << "\"daemons\":[";
    uint16_t d_count = std::min<uint16_t>(telem.services.deamonsLenght, shared::services::MAX_DAMEONS);
    for (uint16_t i = 0; i < d_count; ++i) {
        const auto& d = telem.services.daemons[i];
        std::string d_name = "Inconnu";
        if (d.id == 1) d_name = "Sensors";
        else if (d.id == 2) d_name = "Motors";
        else if (d.id == 3) d_name = "Servo";
        else if (d.id == 4) d_name = "FSM";

        ss << "{";
        ss << "\"id\":" << static_cast<int>(d.id) << ",";
        ss << "\"name\":\"" << d_name << "\",";
        ss << "\"status\":" << static_cast<int>(d.status) << ",";
        ss << "\"error_code\":" << static_cast<int>(d.errorCode);
        ss << "}" << (i + 1 < d_count ? "," : "");
    }
    ss << "]";
    ss << "}";

    ss << "}";
    return ss.str();
}

} // namespace telemetry
