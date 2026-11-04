#pragma once

#include <cstdint>

namespace services {

class LineEstimator {
public:
    static float compute_line_error(const uint8_t sensors[5], float fallback_error = 0.0f);
    static float compute_line_error_analog(const float darkness[5], float fallback_error = 0.0f);
};

} // namespace services
