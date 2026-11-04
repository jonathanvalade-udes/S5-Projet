// Optimisé automatiquement par Optuna le 2026-11-04 11:31:11
// Temps de tour optimal estimé : 1800.14 s
#pragma once

namespace Config {
    constexpr float OPTIMIZED_KP = 0.9366f;
    constexpr float OPTIMIZED_KD = 0.0288f;
    constexpr float OPTIMIZED_KI = 0.0209f;
    constexpr float OPTIMIZED_CRUISE_SPEED = 0.3201f; // m/s
    constexpr float OPTIMIZED_MIN_CURVE_SPEED = 0.1395f; // m/s
    constexpr float OPTIMIZED_SAFETY_FACTOR = 0.8983f;
    constexpr float OPTIMIZED_BRAKE_GAIN = 0.0359f;
    constexpr float OPTIMIZED_ERROR_ALPHA = 0.5230f;
    constexpr float OPTIMIZED_CENTER_KNEE = 0.1328f;
}
