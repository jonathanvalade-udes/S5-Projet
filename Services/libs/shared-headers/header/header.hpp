#pragma once

#include <cstdint>
#include "header_type.hpp"

namespace shared::headers{
    
    namespace flags{
        constexpr inline uint8_t REPLY_REQUESTED = 1;
        constexpr inline uint8_t  FIRE_AND_FORGET = 1 << 1;
        constexpr inline uint8_t CRC_ENABLED = 1 << 2;
    }

    struct PacketHeader {
        HeadersType type;
        uint8_t headerFlag;
        uint16_t seqNumber;
        uint32_t payloadLength;
    };

}