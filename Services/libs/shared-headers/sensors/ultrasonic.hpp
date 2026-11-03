#pragma once
#include <cstdint>

namespace shared::sensors::ultrasonic{
    
    namespace flags{
        constexpr inline uint32_t OBJECT_DETECTED = 1;
        constexpr inline uint32_t SENSOR_ACTIVATED = 1 << 1;
        constexpr inline uint32_t ANOMALIES_DETECTED = 1 << 2;
    }
    struct UltrasonicTelemetry{
        uint32_t rawADC;
        float calculatedDistance;
        uint32_t flags;
        uint32_t errorCode;
    };
}