#pragma once

/**
 * @file picar_constants.hpp
 * @brief Valeurs constantes par défaut (fallback dur C++).
 * Utilisé comme valeur de secours si le fichier de configuration (picar.conf)
 * est absent ou si un paramètre est omis.
 */

namespace picar::constants {

// 1. Cinématique & Protection Bille (pour éviter que la bille ne tombe)
// Vitesse RÉELLE de la voiture à 50 % de puissance moteur (mesurée au capteur ultrason sur le robot : ~0,31 m/s).
// Le FSM convertit sa vitesse en puissance par "vitesse / vitesse de croisière x 50 %" : la croisière vaut donc toujours
// 50 % de puissance, quelle que soit sa valeur dans picar.conf, et la vitesse en m/s du FSM n'est PAS la vitesse réelle.
constexpr float PHYSICAL_SPEED_AT_HALF_THROTTLE = 0.32f;
constexpr float DEFAULT_MAX_ACCELERATION      = 0.735f; // m/s² (seuil critique FastDrink)
constexpr float DEFAULT_STRAIGHT_CRUISE_SPEED = 0.40f;  // m/s en ligne droite (limite physique : rampe MotionProfiler garantit a <= a_max)
constexpr float DEFAULT_MIN_CURVE_SPEED       = 0.14f;  // m/s en virage serré (R = 120 mm)
constexpr float DEFAULT_CURVE_SAFETY_FACTOR   = 0.85f;  // Marge de sécurité physique virage (85% — validé R >= 150 mm)
constexpr float DEFAULT_PREDICTIVE_BRAKE_GAIN = 0.06f;  // Gain feedforward prédictif sur d_error/dt (m/s par unit/s)

// 2. Asservissement Suivi de Ligne (PD)
constexpr float DEFAULT_LINE_KP               = 0.65f;  // Gain proportionnel
constexpr float DEFAULT_LINE_KD               = 0.05f;  // Gain dérivé anti-secousse

// 3. Évitement & Sécurité
constexpr float DEFAULT_OBSTACLE_STOP_DIST_CM = 20.0f;  // Distance d'arrêt obstacle (cm)
constexpr float DEFAULT_REVERSE_SPEED         = 0.12f;  // Vitesse de recul guidé (m/s)

} // namespace picar::constants

