#pragma once
#include <cstdint>

namespace shared::sensors::ultrasonic{
    
    namespace flags{
        constexpr inline uint32_t OBJECT_DETECTED = 1;
        constexpr inline uint32_t SENSOR_ACTIVATED = 1 << 1;
        constexpr inline uint32_t ANOMALIES_DETECTED = 1 << 2;
    }
    struct UltrasonicTelemetry{
        float rawDistanceCm;        // Mesure brute avant filtrage (cm)
        float calculatedDistance;   // Mesure filtrée après Médian + Passe-Bas EMA (cm)
        uint32_t flags;
        uint32_t errorCode;
    };
}
