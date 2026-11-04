#pragma once

#include <cmath>
#include <algorithm>

/**
 * @brief Régulateur cinématique et générateur de profils de vitesse trapézoïdaux.
 * Conforme aux spécifications FastDrink S5-GI et au Calcul d'Accélération :
 * Garantit le respect strict du seuil de renversement de la bille : a <= 0.735 m/s².
 */
class MotionProfiler {
public:
    // Contrainte physique issue de la conservation d'énergie mécanique
    static constexpr float DEFAULT_MAX_ACCELERATION = 0.735f; // m/s²
    // Rayon de virage minimal imposé par le parcours (S3 : 120 mm)
    static constexpr float MIN_TURN_RADIUS = 0.12f;          // m
    // Empattement du véhicule PiCar-S (distance essieu avant - essieu arrière)
    static constexpr float WHEELBASE_M = 0.14f;              // m
    // Vitesse de croisière nominale en ligne droite (m/s)
    static constexpr float DEFAULT_CRUISE_SPEED = 0.25f;     // m/s

private:
    float max_accel;
    float max_speed;
    float current_speed;

public:
    explicit MotionProfiler(float max_acceleration = DEFAULT_MAX_ACCELERATION, 
                            float max_velocity = DEFAULT_CRUISE_SPEED)
        : max_accel(max_acceleration),
          max_speed(max_velocity),
          current_speed(0.0f) {}

    void reset(float initial_speed = 0.0f) {
        current_speed = initial_speed;
    }

    /**
     * @brief Calcule la vitesse limite en virage pour contrer l'accélération centrifuge :
     * a_c = v² / R <= a_max  ==>  v_max = sqrt(a_max * R)
     */
    float calculate_curve_speed_limit(float radius_m) const {
        if (radius_m <= 0.0f) return 0.0f;
        return std::sqrt(max_accel * radius_m);
    }

    /**
     * @brief Calcule la vitesse adaptative en virage selon la cinématique d'Ackermann et la limite centrifuge :
     * R = L / tan(|delta|)
     * v_max = safety_factor * sqrt(a_max * R)
     */
    float calculate_curve_speed_ackermann(float steer_deg, float cruise_spd, float min_spd, float safety_factor = 0.85f) const {
        constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
        float abs_steer_rad = std::abs(steer_deg) * DEG_TO_RAD;

        // Sous ~3 degrés : pas de virage significatif, pleine vitesse de croisière sans ralentissement
        if (abs_steer_rad < 0.052f) {
            return cruise_spd;
        }

        float tan_steer = std::tan(abs_steer_rad);
        if (tan_steer <= 1e-4f) {
            return cruise_spd;
        }
        float radius = WHEELBASE_M / tan_steer;

        // Limite physique critique a_c = v² / R <= a_max pondérée par le facteur de sécurité
        float v_phys = safety_factor * calculate_curve_speed_limit(radius);
        return std::clamp(v_phys, min_spd, cruise_spd);
    }

    /**
     * @brief Calcule la réduction de vitesse prédictive basée sur le taux de variation de l'erreur de ligne.
     *
     * Feedforward anticipatif : si l'erreur augmente rapidement (robot qui entre dans un virage),
     * on réduit la vitesse AVANT que l'angle de braquage soit maximal. Ceci compense la latence
     * systémique (~60 ms moteurs + 20 ms capteurs) et permet d'aborder les virages plus tôt.
     *
     * @param d_error_dt   Dérivée temporelle de l'erreur de ligne [pas_capteur/s], ex: [-8.0, +8.0]
     * @param current_spd  Vitesse actuelle avant ajustement (m/s)
     * @param brake_gain   Gain feedforward [m/s par unité d'erreur/s], default: 0.06
     * @return Vitesse plafonnée après correction prédictive (>= 0)
     */
    float apply_predictive_brake(float d_error_dt, float current_spd, float brake_gain = 0.06f) const {
        // |d_error/dt| mesure la vitesse d'entrée en virage : plus ça change vite, plus on freine tôt
        float brake_reduction = brake_gain * std::abs(d_error_dt);
        // Limitation : ne réduit jamais à plus de 35% de la vitesse cible (évite micro-saccades sur droites)
        brake_reduction = std::min(brake_reduction, 0.35f * current_spd);
        return std::max(0.0f, current_spd - brake_reduction);
    }

    /**
     * @brief Calcule la distance minimale d'arrêt à décélération maximale autorisée :
     * d = v² / (2 * a_max)
     */
    float calculate_stopping_distance(float speed_m_s) const {
        return (speed_m_s * speed_m_s) / (2.0f * max_accel);
    }

    /**
     * @brief Calcule la vitesse d'approche maximale garantissant un arrêt complet sans dépasser a_max :
     * v_max = sqrt(2 * a_max * d)
     */
    float calculate_max_safe_approach_speed(float distance_to_target_m) const {
        if (distance_to_target_m <= 0.0f) return 0.0f;
        return std::sqrt(2.0f * max_accel * distance_to_target_m);
    }

    /**
     * @brief Met à jour la vitesse actuelle vers la consigne en appliquant la rampe d'accélération/décélération.
     * @param target_speed Vitesse cible souhaitée (m/s)
     * @param dt Pas de temps écoulé (s)
     * @return Vitesse régulée admissible (m/s)
     */
    float update(float target_speed, float dt) {
        if (dt <= 0.0f) dt = 0.02f; // Évite division par zéro ou pas négatif

        // Plafonnement absolu de la vitesse cible
        float target_clamped = std::clamp(target_speed, -max_speed, max_speed);

        // Variation maximale permise sur le pas dt
        float max_delta_v = max_accel * dt;
        float delta_v = target_clamped - current_speed;

        if (delta_v > max_delta_v) {
            current_speed += max_delta_v; // Accélération bornée
        } else if (delta_v < -max_delta_v) {
            current_speed -= max_delta_v; // Freinage borné (évite la chute de la bille)
        } else {
            current_speed = target_clamped;
        }

        return current_speed;
    }

    float get_current_speed() const { return current_speed; }
    float get_max_accel() const { return max_accel; }
    float get_max_speed() const { return max_speed; }
    void set_max_speed(float speed) { max_speed = speed; }
    void set_max_accel(float accel) { max_accel = std::max(0.05f, accel); }
};
