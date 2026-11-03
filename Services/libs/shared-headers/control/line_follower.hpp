#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

struct LineFollowerOutput {
    float steer_normalized; // [-1.0f, +1.0f] (-1 = gauche max, +1 = droite max)
    float steer_degrees;    // [-45.0f, +45.0f] pour la HAL moteur
    float target_velocity;  // m/s
    bool is_finished;       // true si barre en T détectée (fin de parcours)
    bool is_line_lost;      // true si ligne perdue au-delà du timeout
};

/**
 * @brief Contrôleur Proportionnel-Dérivé (PD) de suivi de ligne optique.
 * Conforme aux spécifications S2 (ruban 18 mm), S3 (R >= 120 mm) et S10.
 */
class LineFollowerController {
public:
    static constexpr float DEFAULT_KP = 0.65f;
    static constexpr float DEFAULT_KD = 0.20f;
    static constexpr float CRUISE_SPEED = 0.22f;       // m/s en ligne droite
    static constexpr float MIN_CURVE_SPEED = 0.10f;     // m/s en virage serré (R = 120 mm)
    static constexpr float SPEED_REDUCTION_GAIN = 0.08f;
    static constexpr float LINE_LOSS_TIMEOUT = 1.20f;   // Secondes avant arrêt d'urgence

private:
    float kp;
    float kd;
    float cruise_speed;
    float min_curve_speed;

    float last_error;
    float last_steer;
    float line_lost_elapsed;
    float filtered_d_error;
    bool has_previous_error;

    const float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

public:
    explicit LineFollowerController(float p = DEFAULT_KP, 
                                    float d = DEFAULT_KD,
                                    float cruise = CRUISE_SPEED, 
                                    float min_speed = MIN_CURVE_SPEED)
        : kp(p), kd(d),
          cruise_speed(cruise),
          min_curve_speed(min_speed),
          last_error(0.0f),
          last_steer(0.0f),
          line_lost_elapsed(0.0f),
          filtered_d_error(0.0f),
          has_previous_error(false) {}

    void reset() {
        last_error = 0.0f;
        last_steer = 0.0f;
        line_lost_elapsed = 0.0f;
        filtered_d_error = 0.0f;
        has_previous_error = false;
    }

    /**
     * @brief Calcule l'erreur spatiale barycentrique de centrage sur le ruban noir.
     * @param sensors Tableau de 5 capteurs (1 = noir/ruban, 0 = blanc/sol).
     * @param out_error Erreur calculée dans [-2.0f, +2.0f].
     * @return true si au moins un capteur est actif, false si perte de ligne.
     */
    bool compute_barycentric_error(const uint8_t sensors[5], float& out_error) const {
        float total_weight = 0.0f;
        int active_count = 0;

        for (int i = 0; i < 5; ++i) {
            if (sensors[i] != 0) {
                total_weight += weights[i];
                active_count++;
            }
        }

        if (active_count == 0) {
            return false;
        }

        out_error = total_weight / static_cast<float>(active_count);
        return true;
    }

    /**
     * @brief Met à jour le régulateur à partir d'une position d'erreur continue (barycentre analogique).
     * @param error Erreur spatiale continue [-2.0f, +2.0f].
     * @param has_line True si la ligne est perçue par au moins un capteur.
     * @param is_t_stop True si la ligne d'arrêt en T est détectée.
     * @param dt Intervalle de temps écoulé (s).
     */
    LineFollowerOutput update_continuous(float error, bool has_line, bool is_t_stop, float dt) {
        if (dt <= 0.0f) dt = 0.02f;

        LineFollowerOutput out{};
        out.is_finished = false;
        out.is_line_lost = false;

        // Cas 0 : Ligne d'arrêt terminale en T
        if (is_t_stop) {
            out.steer_normalized = 0.0f;
            out.steer_degrees = 0.0f;
            out.target_velocity = 0.0f;
            out.is_finished = true;
            return out;
        }

        // Cas 1 : Ligne détectée
        if (has_line) {
            line_lost_elapsed = 0.0f;

            // Dérivée de l'erreur avec filtrage et limitation de taux physique (anti-kick)
            float d_error = 0.0f;
            if (has_previous_error) {
                float raw_d = (error - last_error) / dt;
                // Saturation de la dérivée pour interdire les chocs impulsionnels de discrétisation
                raw_d = std::clamp(raw_d, -8.0f, 8.0f);
                filtered_d_error = 0.20f * raw_d + 0.80f * filtered_d_error;
                d_error = filtered_d_error;
            } else {
                has_previous_error = true;
                filtered_d_error = 0.0f;
            }
            last_error = error;

            // Correction PD
            float steer = (kp * error) + (kd * d_error);
            steer = std::clamp(steer, -1.0f, 1.0f);
            last_steer = steer;

            // Modulation de vitesse selon la sévérité du virage (anti-déversement S6)
            float curve_severity = std::abs(error);
            float speed_penalty = SPEED_REDUCTION_GAIN * std::pow(curve_severity, 1.5f);
            float target_v = std::max(min_curve_speed, cruise_speed - speed_penalty);

            out.steer_normalized = steer;
            out.steer_degrees = steer * 45.0f;
            out.target_velocity = target_v;
            return out;
        }

        // Cas 2 : Perte de ligne temporaire
        line_lost_elapsed += dt;
        has_previous_error = false; // Évite tout pic de dérivée brutal à la réacquisition du ruban

        if (line_lost_elapsed <= LINE_LOSS_TIMEOUT) {
            // Maintenir fermement le braquage dans la direction du virage pour recouper la trajectoire
            // Ne JAMAIS redresser les roues en virage aveugle !
            float hold_steer = last_steer;
            if (std::abs(hold_steer) > 0.10f) {
                // Accentuer légèrement le braquage pour forcer la rentrée sur la piste
                hold_steer = std::copysign(std::max(std::abs(hold_steer), 0.85f), hold_steer);
            }
            out.steer_normalized = hold_steer;
            out.steer_degrees = hold_steer * 45.0f;
            out.target_velocity = min_curve_speed;
        } else {
            // Perte prolongée -> arrêt sécuritaire
            out.steer_normalized = 0.0f;
            out.steer_degrees = 0.0f;
            out.target_velocity = 0.0f;
            out.is_line_lost = true;
        }

        return out;
    }

    /**
     * @brief Met à jour le régulateur et génère la commande de direction et vitesse (capteurs discrets).
     * @param sensors État des 5 capteurs optiques (1 = noir, 0 = blanc).
     * @param dt Intervalle de temps écoulé (s).
     */
    LineFollowerOutput update(const uint8_t sensors[5], float dt) {
        int active_count = 0;
        for (int i = 0; i < 5; ++i) {
            if (sensors[i] != 0) active_count++;
        }

        bool is_t_stop = (active_count >= 4);
        float error = 0.0f;
        bool has_line = compute_barycentric_error(sensors, error);

        return update_continuous(error, has_line, is_t_stop, dt);
    }
};
