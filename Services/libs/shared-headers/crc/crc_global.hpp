#pragma once

#include <cstdint>

namespace shared::crc{
    constexpr inline uint32_t CRC_POLYNOMIAL = 0xEDB88320;

    constexpr inline uint32_t CRC_INIT = 0xFFFFFFFF;

    struct PacketCRC{
        uint32_t value;
    };
}