#pragma once

#include <cstdint>

namespace shared::commands {
    enum class CommandID: uint8_t {
        Ping = 0x01,
        SetActive = 0x02,
        SetControlFlags = 0x03
    };
}