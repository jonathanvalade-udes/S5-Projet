// Optimisé automatiquement par Optuna le 2026-11-04 19:54:35
// Temps de tour optimal estimé : 1051.25 s
#pragma once

namespace Config {
    constexpr float OPTIMIZED_KP = 0.5500f;
    constexpr float OPTIMIZED_KD = 0.0350f;
    constexpr float OPTIMIZED_KI = 0.1000f;
    constexpr float OPTIMIZED_CRUISE_SPEED = 0.3800f; // m/s
    constexpr float OPTIMIZED_MIN_CURVE_SPEED = 0.2000f; // m/s
    constexpr float OPTIMIZED_SAFETY_FACTOR = 0.9000f;
    constexpr float OPTIMIZED_BRAKE_GAIN = 0.0000f;
    constexpr float OPTIMIZED_ERROR_ALPHA = 0.4000f;
    constexpr float OPTIMIZED_CENTER_KNEE = 0.1500f;
}
