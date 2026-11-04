#pragma once

/**
 * @file picar_constants.hpp
 * @brief Valeurs constantes par défaut (fallback dur C++).
 * Utilisé comme valeur de secours si le fichier de configuration (picar.conf)
 * est absent ou si un paramètre est omis.
 */

namespace picar::constants {

// 1. Cinématique & Protection Bille (pour éviter que la bille ne tombe)
constexpr float DEFAULT_MAX_ACCELERATION      = 0.735f; // m/s² (seuil critique FastDrink)
constexpr float DEFAULT_STRAIGHT_CRUISE_SPEED = 0.32f;  // m/s en ligne droite
constexpr float DEFAULT_MIN_CURVE_SPEED       = 0.14f;  // m/s en virage serré
constexpr float DEFAULT_CURVE_SAFETY_FACTOR   = 0.75f;  // Marge de sécurité physique virage (75%)

// 2. Asservissement Suivi de Ligne (PD)
constexpr float DEFAULT_LINE_KP               = 0.65f;  // Gain proportionnel
constexpr float DEFAULT_LINE_KD               = 0.05f;  // Gain dérivé anti-secousse

// 3. Évitement & Sécurité
constexpr float DEFAULT_OBSTACLE_STOP_DIST_CM = 20.0f;  // Distance d'arrêt obstacle (cm)
constexpr float DEFAULT_REVERSE_SPEED         = 0.12f;  // Vitesse de recul guidé (m/s)

} // namespace picar::constants

