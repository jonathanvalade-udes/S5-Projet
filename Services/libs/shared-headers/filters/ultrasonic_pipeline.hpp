#pragma once

#include "median_filter.hpp"
#include "low_pass_ema.hpp"

/**
 * @brief Pipeline complet de traitement du capteur ultrason HC-SR04.
 * Combine :
 *  1. Rejet d'aberrations extrêmes (0 ou > 400 cm)
 *  2. Filtre Médian Glissant (élimine les pics de bruit / glitches)
 *  3. Filtre Passe-Bas Exponentiel EMA (lissage sans à-coup du signal)
 */
class UltrasonicPipeline {
private:
    MedianFilter median;
    LowPassEMA ema;
    float last_valid_distance;

public:
    explicit UltrasonicPipeline(size_t median_window = 5, float ema_alpha = 0.25f)
        : median(median_window),
          ema(ema_alpha),
          last_valid_distance(100.0f) {}

    /**
     * @brief Traite un échantillon brut du capteur ultrason.
     * @param raw_distance_cm Distance mesurée brute en cm.
     * @param dt Intervalle de temps en secondes (conservé pour rétrocompatibilité d'interface).
     * @param robot_speed_cm_s Vitesse actuelle du robot (conservée pour rétrocompatibilité d'interface).
     * @return Distance filtrée et sécurisée en cm.
     */
    float process(float raw_distance_cm, float dt = 0.02f, float robot_speed_cm_s = 0.0f) {
        (void)dt;
        (void)robot_speed_cm_s;

        // 1. Rejet matériel immédiat des valeurs impossibles HC-SR04 (< 2 cm ou > 400 cm)
        float sanitized = raw_distance_cm;
        if (sanitized <= 0.0f || sanitized > 400.0f) {
            sanitized = last_valid_distance;
        }

        // 2. Filtre médian glissant (Anti-Glitch / Rejet des pics aberrants)
        float median_val = median.update(sanitized);

        // 3. Filtre passe-bas exponentiel EMA (Lissage régulier du signal)
        float filtered = ema.filter(median_val);

        last_valid_distance = filtered;
        return filtered;
    }

    void reset(float initial_distance = 100.0f) {
        median.reset();
        ema.reset(initial_distance);
        last_valid_distance = initial_distance;
    }

    float get_distance() const {
        return last_valid_distance;
    }

    float get_ema_value() const {
        return ema.get_value();
    }
};
