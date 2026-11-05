#pragma once

/**
 * @file picar_constants.hpp
 * @brief Valeurs constantes par défaut (fallback dur C++).
 * Utilisé comme valeur de secours si le fichier de configuration (picar.conf)
 * est absent ou si un paramètre est omis.
 */

namespace picar::constants {

// 0. Fréquence Système Temps Réel
constexpr int   DEFAULT_HZ                    = 200;    // 200 Hz (5 ms)

// 1. Cinématique & Protection Bille (pour éviter que la bille ne tombe)
constexpr float DEFAULT_MAX_ACCELERATION      = 0.735f; // m/s² (seuil critique FastDrink)
constexpr float DEFAULT_STRAIGHT_CRUISE_SPEED = 0.32f;  // m/s en ligne droite (optimisé tour 37s)
constexpr float DEFAULT_MIN_CURVE_SPEED       = 0.20f;  // m/s en virage serré (R = 120 mm)
constexpr float DEFAULT_CURVE_SAFETY_FACTOR   = 0.90f;  // Marge de sécurité physique virage
constexpr float DEFAULT_PREDICTIVE_BRAKE_GAIN = 0.00f;  // Désactivé (pilotage Ackermann pur)

// 2. Asservissement Suivi de Ligne (PD)
constexpr float DEFAULT_LINE_KP               = 0.58f;  // Gain proportionnel (réactif & lissé)
constexpr float DEFAULT_LINE_KD               = 0.0350f; // Gain dérivé anti-secousse

// 3. Évitement & Sécurité
constexpr float DEFAULT_OBSTACLE_STOP_DIST_CM = 20.0f;  // Distance d'arrêt obstacle (cm)
constexpr float DEFAULT_REVERSE_SPEED         = 0.12f;  // Vitesse de recul guidé (m/s)
constexpr float DEFAULT_AVOID_SIDE            = 1.0f;   // 1.0 = droite, -1.0 = gauche
constexpr float DEFAULT_AUTO_START_DELAY      = 0.0f;   // 0.0 = attente stricte du bouton

} // namespace picar::constants

