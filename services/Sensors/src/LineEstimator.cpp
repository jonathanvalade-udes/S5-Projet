#include "LineEstimator.hpp"
#include <algorithm>

namespace services {

float LineEstimator::compute_line_error(const uint8_t sensors[5], float fallback_error) {
    constexpr float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
    float weighted_sum = 0.0f;
    int active_count = 0;

    for (int i = 0; i < 5; ++i) {
        if (sensors[i]) {
            weighted_sum += weights[i];
            active_count++;
        }
    }

    if (active_count == 0) {
        return fallback_error;
    }
    return (weighted_sum / (2.0f * active_count));
}

float LineEstimator::compute_line_error_analog(const float darkness[5], float fallback_error) {
    constexpr float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
    float total_darkness = 0.0f;
    float weighted_sum = 0.0f;

    for (int i = 0; i < 5; ++i) {
        float d = std::clamp(darkness[i], 0.0f, 1.0f);
        total_darkness += d;
        weighted_sum += weights[i] * d;
    }

    if (total_darkness < 0.20f) {
        return fallback_error;
    }
    return (weighted_sum / (2.0f * total_darkness));
}

} // namespace services
