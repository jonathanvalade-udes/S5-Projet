// Optimisé automatiquement par Optuna le 2026-11-04 11:34:27
// Temps de tour optimal estimé : 1800.11 s
#pragma once

namespace Config {
    constexpr float OPTIMIZED_KP = 0.9167f;
    constexpr float OPTIMIZED_KD = 0.0874f;
    constexpr float OPTIMIZED_KI = 0.2362f;
    constexpr float OPTIMIZED_CRUISE_SPEED = 0.3200f; // m/s
    constexpr float OPTIMIZED_MIN_CURVE_SPEED = 0.1375f; // m/s
    constexpr float OPTIMIZED_SAFETY_FACTOR = 0.7286f;
    constexpr float OPTIMIZED_BRAKE_GAIN = 0.0655f;
    constexpr float OPTIMIZED_ERROR_ALPHA = 0.3518f;
    constexpr float OPTIMIZED_CENTER_KNEE = 0.1966f;
}
