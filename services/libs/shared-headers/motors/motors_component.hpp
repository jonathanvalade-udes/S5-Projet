#pragma once

#include "motor_states.hpp"
#include "motor_telemetry.hpp"
#include "servo/servo_telemetry.hpp"
#include "motorsConfig.hpp"

namespace shared::motors{

    namespace flags::component{
        constexpr inline uint16_t DISCOVERABLE = 1;
        constexpr inline uint16_t ANOMALIES_DETECTED = 1 << 1;
        constexpr inline uint16_t WARNING_DETECTED = 1 << 2;
        constexpr inline uint16_t CONFIG_ERROR = 1 << 3;
        constexpr inline uint16_t DRIVER_ERROR = 1 << 4;
        constexpr inline uint16_t SOCKET_ERROR = 1 << 5;
        constexpr inline uint16_t SHM_ERROR = 1 << 6;
    }

    struct MotorsComponentTelemetry{
        uint32_t errorCode;
        MotorTelemetry leftMotor;
        MotorTelemetry rightMotor;
        servos::ServoTelemetry steeringServo; // Direction intégrée directement dans le composant Motors
        MotorStates state;
        uint16_t componentFlags;
        MotorsConfig config;
    };
}
