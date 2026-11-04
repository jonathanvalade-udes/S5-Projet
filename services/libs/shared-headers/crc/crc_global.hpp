#pragma once

#include <cstdint>
#include <cstddef>

namespace shared::crc{
    constexpr inline uint32_t CRC_POLYNOMIAL = 0xEDB88320;

    constexpr inline uint32_t CRC_INIT = 0xFFFFFFFF;

    struct PacketCRC{
        uint32_t value;
    };

    /**
     * @brief Computes standard IEEE 802.3 32-bit CRC.
     * Uses CRC_POLYNOMIAL (0xEDB88320) and CRC_INIT (0xFFFFFFFF).
     */
    inline PacketCRC crcCalc(const uint8_t* data, size_t length) noexcept {
        uint32_t crc = CRC_INIT;
        for (size_t i = 0; i < length; ++i) {
            crc ^= data[i];
            for (int b = 0; b < 8; ++b) {
                crc = (crc >> 1) ^ (CRC_POLYNOMIAL & (-(static_cast<int32_t>(crc & 1))));
            }
        }
        return PacketCRC{~crc};
    }

    inline PacketCRC calculateCRC(const uint8_t* data, size_t length) noexcept {
        return crcCalc(data, length);
    }
}
