#pragma once
#include <cstdint>

namespace shared::sensors::lineFollower{
    
    constexpr inline uint8_t NUMBER_OF_SENSORS = 5;
    constexpr inline float SENSOR_WEIGHTS[NUMBER_OF_SENSORS] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};


    struct LineSensor{
        uint32_t rawADCValue;
        float normalizedValue;
        float correctorValue;
        uint32_t errorCode;
    };

    namespace flags{
        constexpr inline uint32_t SENSOR_ACTIVATED = 1;
        constexpr inline uint32_t ANOMALIES_DETECTED = 1 << 2;
        constexpr inline uint32_t LINE_FOLLOWED = 1 << 3;
        constexpr inline uint32_t LINE_LOST = 1 << 4;

    }

    struct LineFollowerTelemetry{
        LineSensor sensorsData[NUMBER_OF_SENSORS];
        uint32_t flags;
    };
}