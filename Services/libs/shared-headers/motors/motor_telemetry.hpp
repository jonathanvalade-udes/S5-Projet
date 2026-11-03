#pragma once

#include <stdint.h>

namespace shared::motors{

    namespace flags::motor{
        constexpr inline uint8_t MOTOR_DETECTED = 1;
        constexpr inline uint8_t ENCODER_DETECTED = 1 << 1;
        constexpr inline uint8_t MOTOR_RDY = 1 << 2;
        constexpr inline uint8_t PID_APPLIED = 1 << 3;
    }

    namespace flags::pid{
        constexpr inline uint8_t ALLOW_POWER = 1;
        constexpr inline uint8_t ALLOW_DERIVATIVE = 1 << 1;
        constexpr inline uint8_t ALLOW_INTEGRAL = 1 << 2;
        constexpr inline uint8_t CALIBRATED = 1 << 3;
        constexpr inline uint8_t VALUE_DEPENDANT = 1 << 4;
    }

    struct PIDInfo{
        float powerCoefficient;
        float derivativeCoefficient;
        float integralCoefficient;

    };

    struct MotorTelemetry{
        uint32_t motorEncoder;
        float normalizedMotorThrottleTargetted;
        float normalizedMotorThrottleApplied;
        uint8_t motorFlags;
        uint8_t pidFlags;
        uint16_t reserved;
        PIDInfo pidInfo;
    };
}