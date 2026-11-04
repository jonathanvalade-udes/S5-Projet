#pragma once

#include <cstdint>

namespace shared::fsm{


    enum class MachineStates: uint8_t{
        Init = 0x00,
        Unarmed = 0x01,
        Armed = 0x02,
        LineFollowingForward = 0x03,
        CollisionRisk = 0x04,
        Stopped = 0x05,
        CollisionOccured = 0x06,
        RightDeviation = 0x07,
        LeftDeviation = 0x08,
        Reverse = 0x09,
        LineSearching = 0x0A,
        LineDetected = 0x0B,
        LineSearchingTimedOut = 0x0C,
        Abort = 0x0D,
        Failure = 0x0E,
        Undefined = 0xFF
    };
}