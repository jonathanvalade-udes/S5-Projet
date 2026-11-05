#pragma once

#include <cstdint>
#include "machine_states.hpp"

namespace shared::fsm {

    struct FsmCommandPayload {
        float target_speed;          // 4 bytes: consigne vitesse (m/s ou normalisée)
        float target_steering;       // 4 bytes: consigne braquage [-1.0, 1.0]
        float steering_offset;       // 4 bytes: trim offset appliqué
        uint8_t emergency_stop;      // 1 byte: 1 si arrêt d'urgence actif
        uint8_t reserved[3];         // 3 bytes: alignement 32 bits
        uint32_t heartbeat;          // 4 bytes: compteur incrémenté à chaque tick (watchdog)
        MachineStates state;         // 1 byte: état FSM courant
        uint8_t state_reserved[3];   // 3 bytes: padding pour clore à 24 octets (aligné 4)
    };
    static_assert(sizeof(FsmCommandPayload) == 24, "FsmCommandPayload must be 24 bytes");

} // namespace shared::fsm
