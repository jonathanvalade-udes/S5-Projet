// Optimisé automatiquement par Optuna le 2026-11-04 11:37:24
// Temps de tour optimal estimé : 1875.02 s
#pragma once

namespace Config {
    constexpr float OPTIMIZED_KP = 1.1453f;
    constexpr float OPTIMIZED_KD = 0.1416f;
    constexpr float OPTIMIZED_KI = 0.1098f;
    constexpr float OPTIMIZED_CRUISE_SPEED = 0.3500f; // m/s
    constexpr float OPTIMIZED_MIN_CURVE_SPEED = 0.1741f; // m/s
    constexpr float OPTIMIZED_SAFETY_FACTOR = 0.9307f;
    constexpr float OPTIMIZED_BRAKE_GAIN = 0.1010f;
    constexpr float OPTIMIZED_ERROR_ALPHA = 0.3959f;
    constexpr float OPTIMIZED_CENTER_KNEE = 0.1503f;
}
