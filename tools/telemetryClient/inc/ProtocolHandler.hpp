#pragma once

#include <string>
#include <vector>
#include <cstdint>

#include "TcpClient.hpp"
#include "header/header.hpp"
#include "commands/global_commands.hpp"
#include "responses/global_responses.hpp"
#include "crc/crc_global.hpp"
#include "car/full_packet.hpp"
#include "fsm/machine_states.hpp"

namespace telemetry {

class ProtocolHandler {
public:
    ProtocolHandler();
    ~ProtocolHandler();

    // Réalise la poignée de main Ping-Pong exigée par TelemetryService
    bool perform_handshake(TcpClient& client, uint16_t seq_num = 1);

    // Lit le prochain paquet de télémétrie sur le socket (bloquant avec timeout)
    bool read_telemetry_packet(TcpClient& client, shared::car::CarTelemetry& out_telem, uint16_t& out_seq, int timeout_ms = 1000);

    // Envoie une commande texte ou système formatée en paquet conforme
    bool send_command(TcpClient& client, const std::string& cmd_str, uint16_t seq_num);

    // Envoie un Ping keep-alive
    bool send_ping(TcpClient& client, uint16_t seq_num);

    // Convertit une structure CarTelemetry en chaîne JSON pour l'interface Web
    static std::string telemetry_to_json(const shared::car::CarTelemetry& telem, uint16_t seq_num);

    // Nom textuel d'un état FSM
    static std::string fsm_state_to_string(shared::fsm::MachineStates state);
};

} // namespace telemetry
