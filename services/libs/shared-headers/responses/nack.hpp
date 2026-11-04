#pragma once

#include <cstdint>

namespace shared::responses::nack{
    struct NackResponse {
        uint16_t explanation_len;
        uint16_t nackCode;
    };
}