#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <vector>
#include "header_type.hpp"
#include "crc/crc_global.hpp"

namespace shared::headers{
    
    namespace flags{
        constexpr inline uint8_t REPLY_REQUESTED = 1;
        constexpr inline uint8_t FIRE_AND_FORGET = 1 << 1;
        constexpr inline uint8_t CRC_ENABLED = 1 << 2;
    }

    struct PacketHeader {
        HeadersType type;
        uint8_t headerFlag;
        uint16_t seqNumber;
        uint32_t payloadLength;
    };

    /**
     * @brief Serialize a packet into a pre-allocated buffer matching the protocol:
     * [PacketHeader (8 bytes)] [Payload (payloadLen bytes)] [Optional CRC (4 bytes)]
     *
     * @return 1 on success, negative error code on failure.
     */
    inline int serializePacket(uint8_t* buffer, const PacketHeader& header, const uint8_t* payload, size_t payloadLen){
        constexpr size_t HEADER_SIZE = sizeof(PacketHeader);
        constexpr size_t CRC_SIZE = sizeof(shared::crc::PacketCRC);
        if(payloadLen % 4 != 0) return -1;
        std::memcpy(buffer, &header, HEADER_SIZE);
        if (payloadLen > 0 && payload != nullptr) {
            std::memcpy(buffer + HEADER_SIZE, payload, payloadLen);
        }
        
        if(header.headerFlag & shared::headers::flags::CRC_ENABLED){
            shared::crc::PacketCRC crc = shared::crc::crcCalc(buffer, HEADER_SIZE + payloadLen);
            std::memcpy(buffer + HEADER_SIZE + payloadLen, &crc.value, CRC_SIZE);
        }

        return 1;
    }

    /**
     * @brief Serialize a packet with bounds checking.
     * @return Total packet size in bytes on success, negative on error.
     */
    inline int serializePacket(uint8_t* buffer, size_t max_len, const PacketHeader& header, const uint8_t* payload, size_t payloadLen){
        constexpr size_t HEADER_SIZE = sizeof(PacketHeader);
        constexpr size_t CRC_SIZE = sizeof(shared::crc::PacketCRC);
        if(payloadLen % 4 != 0) return -1;
        bool has_crc = (header.headerFlag & shared::headers::flags::CRC_ENABLED) != 0;
        size_t total_size = HEADER_SIZE + payloadLen + (has_crc ? CRC_SIZE : 0);
        if(max_len < total_size) return -2;

        std::memcpy(buffer, &header, HEADER_SIZE);
        if (payloadLen > 0 && payload != nullptr) {
            std::memcpy(buffer + HEADER_SIZE, payload, payloadLen);
        }

        if(has_crc){
            shared::crc::PacketCRC crc = shared::crc::crcCalc(buffer, HEADER_SIZE + payloadLen);
            std::memcpy(buffer + HEADER_SIZE + payloadLen, &crc.value, CRC_SIZE);
        }

        return static_cast<int>(total_size);
    }

    /**
     * @brief Serialize a packet returning a byte vector.
     */
    inline std::vector<uint8_t> serializePacket(const PacketHeader& header, const uint8_t* payload, size_t payloadLen){
        constexpr size_t HEADER_SIZE = sizeof(PacketHeader);
        constexpr size_t CRC_SIZE = sizeof(shared::crc::PacketCRC);
        bool has_crc = (header.headerFlag & shared::headers::flags::CRC_ENABLED) != 0;
        size_t total_size = HEADER_SIZE + payloadLen + (has_crc ? CRC_SIZE : 0);
        std::vector<uint8_t> buffer(total_size);

        std::memcpy(buffer.data(), &header, HEADER_SIZE);
        if (payloadLen > 0 && payload != nullptr) {
            std::memcpy(buffer.data() + HEADER_SIZE, payload, payloadLen);
        }

        if(has_crc){
            shared::crc::PacketCRC crc = shared::crc::crcCalc(buffer.data(), HEADER_SIZE + payloadLen);
            std::memcpy(buffer.data() + HEADER_SIZE + payloadLen, &crc.value, CRC_SIZE);
        }

        return buffer;
    }

    /**
     * @brief Deserializes and validates a packet from a buffer.
     * @return true if packet is valid (and CRC matches if CRC_ENABLED), false otherwise.
     */
    inline bool deserializePacket(const uint8_t* buffer, size_t buffer_len, PacketHeader& out_header, std::vector<uint8_t>& out_payload, uint32_t& out_crc){
        constexpr size_t HEADER_SIZE = sizeof(PacketHeader);
        constexpr size_t CRC_SIZE = sizeof(shared::crc::PacketCRC);

        if(buffer_len < HEADER_SIZE) return false;

        std::memcpy(&out_header, buffer, HEADER_SIZE);
        bool has_crc = (out_header.headerFlag & flags::CRC_ENABLED) != 0;
        size_t expected_total = HEADER_SIZE + out_header.payloadLength + (has_crc ? CRC_SIZE : 0);

        if(buffer_len < expected_total) return false;

        out_payload.resize(out_header.payloadLength);
        if(out_header.payloadLength > 0){
            std::memcpy(out_payload.data(), buffer + HEADER_SIZE, out_header.payloadLength);
        }

        if(has_crc){
            std::memcpy(&out_crc, buffer + HEADER_SIZE + out_header.payloadLength, CRC_SIZE);
            shared::crc::PacketCRC computed = shared::crc::crcCalc(buffer, HEADER_SIZE + out_header.payloadLength);
            if(computed.value != out_crc){
                return false;
            }
        }

        return true;
    }

}
