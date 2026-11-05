#pragma once

#include <cstdint>

namespace shared::fsm {

    enum class MachineStates : uint8_t {
        Init                 = 0x00,
        Unarmed              = 0x01,
        Armed                = 0x02,
        LineFollowingForward = 0x03,
        CollisionRisk        = 0x04,
        Stopped              = 0x05,
        RightDeviation       = 0x06,
        LeftDeviation        = 0x07,
        Abort                = 0x08,
        Failure              = 0x09,
        Undefined            = 0xFF
    };

}
