#pragma once

#include <cstdint>
#include "global_commands.hpp"
/*
Goes inside the Packet's payload
*/
namespace shared::commands {

    enum class ServiceID:uint8_t{
        TELEMETRY = 0x00,
        SENSORS = 0x01,
        MOTORS = 0x02,
        FSM = 0x03
    };

    struct CommandsHeader{
        ServiceID serviceID;
        CommandID commandID;
        uint16_t payloadLen;
    };
};