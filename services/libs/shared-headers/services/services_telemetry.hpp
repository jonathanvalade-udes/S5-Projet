#pragma once

#include <cstdint>

namespace shared::services{
    constexpr inline uint8_t MAX_DAMEONS = 16;
    
    namespace flags{
        constexpr inline uint32_t DEVICE_MOUNTED = 1;
        constexpr inline uint32_t SHARE_MEMORY_MOUNTED = 1 << 1;
        constexpr inline uint32_t PROC_MOUNTED = 1 << 2;
        constexpr inline uint32_t SYSTEM_STARTED = 1 << 3;
        constexpr inline uint32_t DEAMONS_STARTED = 1 << 4;
        constexpr inline uint32_t ETH_PLUGGED = 1 << 5;
        constexpr inline uint32_t ETH_DEBUG_SERVICE_ENABLED = 1 << 6;
        constexpr inline uint32_t ANOMALIES_DETECTED = 1 << 7;
        constexpr inline uint32_t SOCKET_ENABLED = 1 << 8;
        constexpr inline uint32_t DISCOVERABLE = 1 << 9;
        constexpr inline uint32_t PROCESS_STALLED = 1 << 10;
        constexpr inline uint32_t SHELL_ENABLED = 1 << 11;
        constexpr inline uint32_t SSH_ENABLED = 1 << 12;
        constexpr inline uint32_t DHCP_ENABLED = 1 << 13;
    }

    struct DeamonTelemetry{
        uint8_t id;
        uint8_t status;
        uint8_t errorCode;
        uint8_t reserved;
    };

    struct ServiceTelemetry{
        uint32_t flags;
        uint32_t timestamp;
        uint32_t handlerErrorCode;
        uint16_t deamonsLenght;
        uint16_t reserved;
        DeamonTelemetry daemons[MAX_DAMEONS];
    };
}