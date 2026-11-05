#pragma once

#include <string>
#include <vector>
#include <cstdint>

#include "TcpClient.hpp"
#include "header/header.hpp"
#include "commands/commands_header.hpp"
#include "commands/global_commands.hpp"
#include "commands/set_configs.hpp"
#include "commands/set_active.hpp"
#include "commands/emergency_stop.hpp"
#include "responses/global_responses.hpp"
#include "crc/crc_global.hpp"
#include "car/full_packet.hpp"
#include "fsm/machine_states.hpp"
#include "motors/motor_states.hpp"

namespace telemetry {

class ProtocolHandler {
public:
    ProtocolHandler();
    ~ProtocolHandler();

    // Réalise la poignée de main Ping-Pong exigée par TelemetryService
    bool perform_handshake(TcpClient& client, uint16_t seq_num = 1);

    // Lit le prochain paquet de télémétrie sur le socket (bloquant avec timeout)
    bool read_telemetry_packet(TcpClient& client, shared::car::CarTelemetry& out_telem, uint16_t& out_seq, int timeout_ms = 1000);

    // Envoi d'une commande SetConfigKey normalisée selon shared-headers avec CRC32
    bool send_set_config(TcpClient& client, shared::commands::ServiceID srv, const std::string& key, float value, uint16_t seq_num);

    // Envoi d'une commande SetActive (armer / désarmer) normalisée selon shared-headers avec CRC32
    bool send_set_active(TcpClient& client, shared::commands::ServiceID srv, bool active, uint16_t seq_num);

    // Envoi d'une commande EmergencyStop normalisée selon shared-headers avec CRC32
    bool send_emergency_stop(TcpClient& client, shared::commands::ServiceID srv, uint32_t reason_code, uint16_t seq_num);

    // Envoie une commande texte ou système formatée en paquet binaire conforme
    bool send_command(TcpClient& client, const std::string& cmd_str, uint16_t seq_num);

    // Envoie un Ping keep-alive
    bool send_ping(TcpClient& client, uint16_t seq_num);

    // Convertit une chaîne de nom de service ("FSM", "MOTORS", "SENSORS", "TELEMETRY") en ServiceID
    static shared::commands::ServiceID string_to_service_id(const std::string& srv_str);

    // Convertit une structure CarTelemetry en chaîne JSON pour l'interface Web
    static std::string telemetry_to_json(const shared::car::CarTelemetry& telem, uint16_t seq_num);

    // Noms textuels des états
    static std::string fsm_state_to_string(shared::fsm::MachineStates state);
    static std::string motor_state_to_string(shared::motors::MotorStates state);

private:
    std::vector<uint8_t> m_rx_buffer;
};

} // namespace telemetry
