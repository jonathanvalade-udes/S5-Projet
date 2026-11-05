#pragma once

#include <cstdint>

namespace shared::commands {

    struct EmergencyStopPayload {
        uint32_t reason_code; // Code de raison d'arrêt d'urgence
    };
    static_assert(sizeof(EmergencyStopPayload) == 4, "EmergencyStopPayload must be 4 bytes");

} // namespace shared::commands
