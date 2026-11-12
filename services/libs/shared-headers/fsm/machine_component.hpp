#pragma once

#include <cstdint>
#include "machine_states.hpp"
#include "fsmConfig.hpp"

namespace shared::fsm{
    namespace flags {
        constexpr inline uint16_t DISCOVERABLE = 1;
        constexpr inline uint16_t ANOMALIES_DETECTED = 1 << 1;
        constexpr inline uint16_t WARNING_DETECTED = 1 << 2;
        constexpr inline uint16_t CONFIG_ERROR = 1 << 3;
        constexpr inline uint16_t DRIVER_ERROR = 1 << 4;
        constexpr inline uint16_t SOCKET_ERROR = 1 << 5;
        constexpr inline uint16_t SHM_ERROR = 1 << 6;
        constexpr inline uint16_t STALLED = 1 << 7;
        constexpr inline uint16_t ALLOW_REQUEST_STATE = 1 << 8;
        constexpr inline uint16_t RDY = 1 << 9;
        constexpr inline uint16_t AUTOMATE_DEVIATION = 1 << 10;
        constexpr inline uint16_t HEADLESS_CTRL = 1 << 11;
    }


    struct FSMTelemetry{
        MachineStates currentState;
        MachineStates requestedState;
        uint16_t flags;
        uint32_t errorCode;
        FSMConfig config;
    };
}