#pragma once

#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>

namespace services {

/**
 * @brief Modèle cinématique et physique du véhicule PiCar-S (châssis Ackermann sans encodeurs).
 * Centralise les conversions entre grandeurs physiques (m/s, rad, m) et commandes actionneurs (% PWM, degrés).
 */
class VehicleModel {
public:
    static constexpr float DEFAULT_WHEELBASE_M = 0.14f;      // Empattement L (distance essieu avant - essieu arrière)
    static constexpr float DEFAULT_DEADBAND_PWM = 20.0f;     // Seuil de zone morte sous lequel le moteur ne décolle pas (%)
    static constexpr float MAX_STEERING_ANGLE_DEG = 45.0f;   // Angle de braquage maximal admissible par le servo
    static constexpr float MIN_TURN_RADIUS_M = 0.12f;        // Rayon de braquage minimal physique

    struct CalibrationPoint {
        float pwm_pct;
        float speed_m_s;
    };

    VehicleModel();

    /**
     * @brief Convertit une vitesse cible en commande PWM moteur (%) par interpolation linéaire.
     */
    float speed_to_pwm(float target_speed_m_s) const;

    /**
     * @brief Estime la vitesse physique (m/s) à partir de la commande PWM appliquée.
     */
    float pwm_to_speed(float pwm_pct) const;

    /**
     * @brief Calcule le rayon de virage théorique d'Ackermann : R = L / tan(|delta|).
     * @param steer_deg Angle de braquage commandé aux roues en degrés.
     * @return Rayon R en mètres (ou infini si angle quasi-nul).
     */
    float calculate_turn_radius(float steer_deg) const;

    /**
     * @brief Calcule la vitesse limite en courbe pour ne pas dépasser l'accélération latérale autorisée :
     * v_max = sqrt(a_lat_max * R)
     */
    float calculate_curve_speed_limit(float steer_deg, float max_lat_accel, float safety_factor = 0.85f) const;

    /**
     * @brief Permet d'injecter une table de calibration personnalisée issue de mesures expérimentales réelles.
     */
    void set_calibration_table(const std::vector<CalibrationPoint>& table);

    void set_wheelbase(float l) { m_wheelbase_m = std::max(0.05f, l); }
    float get_wheelbase() const { return m_wheelbase_m; }

    void set_deadband_pwm(float db) { m_deadband_pwm = std::clamp(db, 0.0f, 50.0f); }
    float get_deadband_pwm() const { return m_deadband_pwm; }

private:
    float m_wheelbase_m{DEFAULT_WHEELBASE_M};
    float m_deadband_pwm{DEFAULT_DEADBAND_PWM};
    std::vector<CalibrationPoint> m_calibration_table;

    void init_default_table();
};

} // namespace services
