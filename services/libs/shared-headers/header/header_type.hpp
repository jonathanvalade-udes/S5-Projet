#pragma once

#include <cstdint>

namespace shared::headers
{
    enum class HeadersType : uint8_t {
        Command = 0x1,
        Telemetry = 0x02,
        Response = 0x03
    };
} // namespace shared::headers
