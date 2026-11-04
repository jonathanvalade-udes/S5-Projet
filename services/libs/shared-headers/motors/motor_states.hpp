#pragma once

#include <cstdint>

namespace shared::motors{

    enum class MotorStates: uint16_t {
        Init = 0x00,
        Idle = 0x01,
        Running = 0x02,
        Abort = 0x03
    };
}