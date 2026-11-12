#include "LineEstimator.hpp"
#include <algorithm>
#include <cmath>

namespace services {

LineEstimate LineEstimator::estimate_line(const float darkness[5], uint64_t timestamp_us) {
    constexpr float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

    LineEstimate est{};
    est.timestamp_us = timestamp_us;

    float total_darkness = 0.0f;
    float weighted_sum = 0.0f;
    float max_d = 0.0f;
    int dark_count = 0;

    for (int i = 0; i < 5; ++i) {
        float d = darkness[i];
        if (!std::isfinite(d)) {
            d = 0.0f;
        }
        d = std::clamp(d, 0.0f, 1.0f);
        
        total_darkness += d;
        weighted_sum += weights[i] * d;
        if (d > max_d) max_d = d;
        if (d >= T_STOP_DARK_THRESHOLD) dark_count++;
    }

    est.total_darkness = total_darkness;
    est.is_t_stop = (dark_count >= 4 || total_darkness >= T_STOP_TOTAL_DARKNESS);

    if (total_darkness < MIN_TOTAL_DARKNESS || max_d < 0.25f) {
        est.is_valid = false;
        est.is_line_lost = true;
        est.confidence = 0.0f;
        est.error = 0.0f;
        est.normalized_error = 0.0f;
        return est;
    }

    est.error = std::clamp(weighted_sum / total_darkness, -2.0f, 2.0f);
    est.normalized_error = est.error * 0.5f;
    est.is_valid = true;
    est.is_line_lost = false;

    // Confiance : élevée si le pic est net (ruban centré sur 1 ou 2 capteurs), plus faible si bruit uniforme
    float contrast = max_d - (total_darkness - max_d) * 0.25f;
    est.confidence = std::clamp(contrast, 0.0f, 1.0f);

    return est;
}

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
    // Erreur spatiale barycentrique [-2.0f, +2.0f]
    return (weighted_sum / static_cast<float>(active_count));
}

float LineEstimator::compute_line_error_analog(const float darkness[5], float fallback_error) {
    LineEstimate est = estimate_line(darkness);
    if (!est.is_valid) {
        return fallback_error;
    }
    return est.error;
}

} // namespace services
