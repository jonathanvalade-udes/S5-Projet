#pragma once

#include <cstdint>

namespace shared::system {

    enum class SystemStates: uint8_t {
        Init = 0x00,
        PendingConnection = 0x01,
        Inactive = 0x02,
        Active = 0x03,
        Running = 0x04,
        Aborted = 0x05,
    };

}