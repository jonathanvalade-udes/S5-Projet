#pragma once

#include "VehicleModel.hpp"
#include <algorithm>
#include <cmath>

namespace services {

/**
 * @brief Planificateur cinématique de vitesse avec protection couplée de la bille (FastDrink).
 * Gère séparément l'accélération, le freinage, l'accélération latérale en virage,
 * l'accélération résultante (a_res = sqrt(ax² + ay²) <= a_ball_max) et la distance d'arrêt.
 */
class SpeedPlanner {
public:
    static constexpr float DEFAULT_BALL_MAX_ACCEL = 0.70f;     // m/s² (seuil de sécurité de la bille)
    static constexpr float DEFAULT_MAX_ACCEL      = 0.65f;     // m/s² (accélération longitudinale)
    static constexpr float DEFAULT_MAX_BRAKE      = 0.70f;     // m/s² (décélération de freinage)
    static constexpr float DEFAULT_MAX_LAT_ACCEL  = 0.65f;     // m/s² (accélération centripète)
    static constexpr float DEFAULT_MAX_JERK       = 6.0f;      // m/s³ (limitation des à-coups)
    static constexpr float DEFAULT_REACTION_TIME  = 0.05f;     // s (temps de réaction estimé)
    static constexpr float DEFAULT_STOP_MARGIN_M  = 0.04f;     // m (marge d'arrêt obstacle)

    explicit SpeedPlanner(float ball_max_accel = DEFAULT_BALL_MAX_ACCEL,
                          float max_accel = DEFAULT_MAX_ACCEL,
                          float max_brake = DEFAULT_MAX_BRAKE);

    void reset(float initial_speed = 0.0f);

    /**
     * @brief Met à jour le profil de vitesse en appliquant l'ensemble des contraintes couplées.
     * @param target_speed_m_s Vitesse demandée par la navigation.
     * @param steer_deg Braquage commandé aux roues.
     * @param dist_to_obstacle_m Distance mesurée devant l'obstacle (< 0 si pas d'obstacle).
     * @param obstacle_stop_dist_m Distance d'arrêt souhaitée devant l'obstacle.
     * @param dt Intervalle temporel mesuré (s).
     * @param curve_safety_factor Facteur de sécurité en virage.
     * @return Vitesse régulée admissible pour le pas suivant (m/s).
     */
    float update(float target_speed_m_s,
                 float steer_deg,
                 float dist_to_obstacle_m,
                 float obstacle_stop_dist_m,
                 float dt,
                 float curve_safety_factor = 0.85f);

    /**
     * @brief Calcule la distance minimale d'arrêt anticipée à la vitesse v :
     * d_arret = v * t_reaction + v² / (2 * a_brake) + d_marge
     */
    float calculate_stopping_distance(float speed_m_s) const;

    /**
     * @brief Calcule la vitesse maximale d'approche sécurisée pour s'arrêter à la distance cible.
     */
    float calculate_max_safe_approach_speed(float remaining_distance_to_stop_m) const;

    float get_current_speed() const { return m_current_speed; }
    float get_last_ax() const { return m_last_ax; }
    float get_last_ay() const { return m_last_ay; }
    float get_last_resultant_accel() const { return std::sqrt(m_last_ax * m_last_ax + m_last_ay * m_last_ay); }

    void set_ball_max_accel(float a) { m_ball_max_accel = std::max(0.1f, a); }
    void set_max_accel(float a) { m_max_accel = std::max(0.1f, a); }
    void set_max_brake(float a) { m_max_brake = std::max(0.1f, a); }
    void set_max_lat_accel(float a) { m_max_lat_accel = std::max(0.1f, a); }

    const VehicleModel& get_vehicle_model() const { return m_vehicle_model; }
    VehicleModel& get_vehicle_model() { return m_vehicle_model; }

private:
    float m_ball_max_accel{DEFAULT_BALL_MAX_ACCEL};
    float m_max_accel{DEFAULT_MAX_ACCEL};
    float m_max_brake{DEFAULT_MAX_BRAKE};
    float m_max_lat_accel{DEFAULT_MAX_LAT_ACCEL};

    float m_current_speed{0.0f};
    float m_last_ax{0.0f};
    float m_last_ay{0.0f};

    VehicleModel m_vehicle_model;
};

} // namespace services
