#pragma once

#include <cstdint>

#include "line_follower.hpp"
#include "ultrasonic.hpp"
#include "sensorsConfig.hpp"

namespace shared::sensors{

    namespace flags
    {
        constexpr inline uint32_t DISCOVERABLE = 1;
        constexpr inline uint16_t ANOMALIES_DETECTED = 1 << 1;
        constexpr inline uint16_t WARNING_DETECTED = 1 << 2;
        constexpr inline uint16_t CONFIG_ERROR = 1 << 3;
        constexpr inline uint16_t DRIVER_ERROR = 1 << 4;
        constexpr inline uint16_t SOCKET_ERROR = 1 << 5;
        constexpr inline uint16_t SHM_ERROR = 1 << 6;
    } // namespace flags
    
    struct SensorsTelemetry{
        lineFollower::LineFollowerTelemetry lineFollower;
        ultrasonic::UltrasonicTelemetry ultrasonicSensor;
        uint16_t flags;
        uint16_t errorCode;
        SensorsConfig config;
    };
}