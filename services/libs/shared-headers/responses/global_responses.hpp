#pragma once

#include <cstdint>


namespace shared::responses
{
    enum class ResponseID: uint32_t {
        Pong = 0x01,
        Ack = 0x02,
        Nack = 0x03
    };
} // namespace shared::responses
