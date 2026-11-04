#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

struct LineFollowerOutput {
    float steer_normalized; // [-1.0f, +1.0f] (-1 = gauche max, +1 = droite max)
    float steer_degrees;    // [-45.0f, +45.0f] pour la HAL moteur
    float target_velocity;  // m/s (calculé par le régulateur interne — utilisé si Ackermann non disponible)
    float d_error_dt;       // Dérivée temporelle de l'erreur de ligne [pas/s] pour le feedforward prédictif
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
    static constexpr float DEFAULT_KD = 0.05f;         // Faible : avec la latence (~60 ms) un kd élevé fait osciller la direction en ligne droite
    static constexpr float DEFAULT_KI = 0.30f;         // Compense le biais fixe des roues (alignement), voir INTEGRAL_*
    static constexpr float INTEGRAL_LIMIT = 0.15f;     // Contribution max de l'intégrale au braquage normalisé (~7°)
    static constexpr float INTEGRAL_ACTIVE_BELOW = 0.6f; // N'intègre que près du centre (|erreur| en pas de capteur) : pas dans les virages
    static constexpr float CRUISE_SPEED = 0.22f;       // m/s fallback interne (le FSM utilise picar.conf via set_cruise_speed)
    static constexpr float MIN_CURVE_SPEED = 0.10f;     // m/s en virage serré (R = 120 mm)
    static constexpr float SPEED_REDUCTION_GAIN = 0.08f;
    static constexpr float LINE_LOSS_TIMEOUT = 1.20f;   // Secondes avant arrêt d'urgence
    static constexpr float ANALOG_DARK_THRESHOLD = 0.5f;       // Un capteur compte comme "noir" au-delà (barre d'arrivée)
    static constexpr float ANALOG_ERROR_ALPHA = 0.4f;          // Filtre EMA de l'erreur analogique (1 = aucun filtre)
    static constexpr float ANALOG_CENTER_KNEE = 0.15f;         // Erreur (en pas de capteur ~19 mm) sous laquelle on atténue
    static constexpr float ANALOG_MIN_TOTAL_DARKNESS = 0.25f;  // Noirceur totale minimale pour dire "ligne vue"

private:
    float kp;
    float kd;
    float cruise_speed;
    float min_curve_speed;

    float last_error;
    float last_steer;
    float line_lost_elapsed;
    float filtered_d_error;
    float integral = 0.0f;              // terme intégral du braquage (normalisé), compense le biais des roues
    bool has_previous_error;
    bool finish_detection_enabled = true;
    float analog_error_filtered = 0.0f; // erreur analogique filtrée (EMA) avant le terme dérivé
    bool has_analog_error = false;
    float last_d_error_dt = 0.0f;       // dérivée temporelle de l'erreur exposée pour le feedforward prédictif

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

    /// Correction de braquage (normalisée) apprise par le terme intégral : sur une droite, elle compense le biais
    /// des roues. Utilisée comme correction fixe pendant les manoeuvres en boucle ouverte (évitement).
    float steering_trim() const { return integral; }

    /// Désactive la détection de la barre d'arrivée (ex. juste après un évitement : la voiture recoupe la ligne en biais
    /// et plusieurs capteurs voient du noir en même temps, ce qui ressemble à la barre).
    void set_finish_detection(bool enabled) { finish_detection_enabled = enabled; }

    void reset() {
        last_error = 0.0f;
        last_steer = 0.0f;
        line_lost_elapsed = 0.0f;
        filtered_d_error = 0.0f;
        integral = 0.0f;
        has_previous_error = false;
        analog_error_filtered = 0.0f;
        has_analog_error = false;
        last_d_error_dt = 0.0f;
    }

    void set_kp(float p) { kp = std::max(0.0f, p); }
    void set_kd(float d) { kd = std::max(0.0f, d); }
    void set_cruise_speed(float s) { cruise_speed = std::max(0.05f, s); }
    void set_min_curve_speed(float s) { min_curve_speed = std::max(0.05f, s); }

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
        out.d_error_dt = last_d_error_dt; // Exposé pour le feedforward prédictif dans le FSM

        // Cas 0 : Ligne d'arrêt terminale en T
        if (is_t_stop) {
            out.steer_normalized = 0.0f;
            out.steer_degrees = 0.0f;
            out.target_velocity = 0.0f;
            out.d_error_dt = 0.0f;
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
            last_d_error_dt = d_error; // Mémorisé pour être exposé dans la prochaine sortie

            // Intégrale : seulement près du centre (ligne droite), limitée ; relâchée doucement dans les virages
            if (std::abs(error) < INTEGRAL_ACTIVE_BELOW) {
                integral += DEFAULT_KI * error * dt;
            } else {
                integral *= (1.0f - 0.5f * dt);
            }
            integral = std::clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

            // Correction PID
            float steer = (kp * error) + (kd * d_error) + integral;
            steer = std::clamp(steer, -1.0f, 1.0f);
            last_steer = steer;

            // Modulation de vitesse selon la sévérité du virage (anti-déversement S6)
            float curve_severity = std::abs(error);
            float speed_penalty = SPEED_REDUCTION_GAIN * std::pow(curve_severity, 1.5f);
            float target_v = std::max(min_curve_speed, cruise_speed - speed_penalty);

            out.steer_normalized = steer;
            out.steer_degrees = steer * 45.0f;
            out.target_velocity = target_v; // Vitesse régulée interne (le FSM applique Ackermann par-dessus)
            out.d_error_dt = d_error;
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
     * @brief Met à jour le régulateur à partir des valeurs ANALOGIQUES des capteurs (sans binarisation).
     *
     * L'erreur est le centre de gravité de la noirceur : sum(w_i * d_i) / sum(d_i). Quand le ruban passe
     * entre deux capteurs, l'erreur varie continuellement au lieu de sauter de 0.5 ou 1.0 d'un coup, ce qui
     * supprime les à-coups du braquage (plein débattement puis retour à 0).
     *
     * @param darkness 5 valeurs [0.0 = blanc, 1.0 = noir].
     * @param dt Intervalle de temps écoulé (s).
     */
    LineFollowerOutput update_analog(const float darkness[5], float dt) {
        float total = 0.0f;
        float weighted = 0.0f;
        int dark_count = 0;

        for (int i = 0; i < 5; ++i) {
            const float d = std::clamp(darkness[i], 0.0f, 1.0f);
            total += d;
            weighted += weights[i] * d;
            if (d > ANALOG_DARK_THRESHOLD) dark_count++;
        }

        const bool is_t_stop = finish_detection_enabled && (dark_count >= 4 || total >= 3.6f);
        const bool has_line = (total >= ANALOG_MIN_TOTAL_DARKNESS);
        float error = has_line ? (weighted / total) : 0.0f;

        // Filtrage passe-bas de l'erreur AVANT la dérivée : sans lui, le bruit de mesure (quantification,
        // ADC) est multiplié par 1/dt dans le terme D et fait trembler la direction en ligne droite.
        if (has_line) {
            if (!has_analog_error) {
                analog_error_filtered = error; // première mesure : pas de retard
                has_analog_error = true;
            } else {
                analog_error_filtered += ANALOG_ERROR_ALPHA * (error - analog_error_filtered);
            }
            error = analog_error_filtered;

            // Zone douce autour du centre : une petite erreur (quelques mm) est surtout du bruit, on l'atténue
            // (gain ~|e|/knee, nul au centre) au lieu de la corriger ; continu, et inchangé au-delà du knee.
            const float abs_e = std::abs(error);
            if (abs_e < ANALOG_CENTER_KNEE) {
                error *= abs_e / ANALOG_CENTER_KNEE;
            }
        } else {
            has_analog_error = false; // ligne perdue : ré-initialise le filtre à la réacquisition
        }

        return update_continuous(error, has_line, is_t_stop, dt);
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

        bool is_t_stop = finish_detection_enabled && (active_count >= 4);
        float error = 0.0f;
        bool has_line = compute_barycentric_error(sensors, error);

        return update_continuous(error, has_line, is_t_stop, dt);
    }
};
