#pragma once

#include <cstdint>

namespace shared::commands {

    struct SetActivePayload {
        uint8_t active;      // 1 = active, 0 = inactive
        uint8_t reserved[3]; // padding pour alignement naturel 32 bits
    };
    static_assert(sizeof(SetActivePayload) == 4, "SetActivePayload must be 4 bytes");

} // namespace shared::commands