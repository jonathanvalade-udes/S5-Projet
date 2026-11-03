#pragma once

#include <cstdint>

namespace shared::servos{
    namespace flags{
        constexpr inline uint16_t DISCOVERABLE = 1;
        constexpr inline uint16_t ANOMALIES_DETECTED = 1 << 1;
        constexpr inline uint16_t WARNING_DETECTED = 1 << 2;
        constexpr inline uint16_t CONFIG_ERROR = 1 << 3;
        constexpr inline uint16_t DRIVER_ERROR = 1 << 4;
        constexpr inline uint16_t SOCKET_ERROR = 1 << 5;
        constexpr inline uint16_t SHM_ERROR = 1 << 6;
        constexpr inline uint16_t PWM_DETECTED = 1 << 7;
        constexpr inline uint16_t RDY = 1 << 8;
        constexpr inline uint16_t ANGLE_FORCED = 1 << 9;
        constexpr inline uint16_t COEFFICIENT_APPLIED = 1 << 10;
    }

    constexpr inline float MAX_ANGLE = 90.0;

    struct ServoTelemetry{
        uint16_t flags;
        uint16_t reserved;
        float normalizedCurrentAngle;
        uint32_t currentCCR;
        float normalizedTargettedAngle;
        uint32_t targetedCCR;
        float coefficient;
        uint32_t errorCode;
    };
}