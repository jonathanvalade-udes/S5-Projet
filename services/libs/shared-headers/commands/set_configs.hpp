#pragma once
#include <cstdint>
namespace shared::commands::config{

    constexpr inline uint8_t MAX_KEY_LEN = 64; 
    struct SetConfigPayload{
        const char key[MAX_KEY_LEN];
        float value;
    };
};