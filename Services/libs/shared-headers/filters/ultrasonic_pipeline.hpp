#pragma once

#include "median_filter.hpp"
#include "low_pass_ema.hpp"
#include "kalman_1d.hpp"

/**
 * @brief Pipeline complet de traitement du capteur ultrason HC-SR04.
 * Combine :
 *  1. Rejet d'aberrations extrêmes (0 ou > 400 cm)
 *  2. Filtre Médian Glissant (élimine les pics de bruit)
 *  3. Filtre de Kalman 1D ou EMA (lissage et fusion cinématique)
 */
class UltrasonicPipeline {
private:
    MedianFilter median;
    LowPassEMA ema;
    KalmanFilter1D kalman;
    float last_valid_distance;

public:
    explicit UltrasonicPipeline(size_t median_window = 5, float ema_alpha = 0.25f)
        : median(median_window),
          ema(ema_alpha),
          kalman(0.05f, 4.0f, 73.5f),
          last_valid_distance(100.0f) {}

    /**
     * @brief Traite un échantillon brut du capteur ultrason.
     * @param raw_distance_cm Distance mesurée brute en cm.
     * @param dt Intervalle de temps en secondes (ex: 0.02s pour 50 Hz).
     * @param robot_speed_cm_s Vitesse actuelle du robot (pour la prédiction Kalman).
     * @return Distance filtrée et sécurisée en cm.
     */
    float process(float raw_distance_cm, float dt = 0.02f, float robot_speed_cm_s = 0.0f) {
        // 1. Rejet matériel immédiat des valeurs impossibles HC-SR04 (< 2 cm ou > 400 cm)
        float sanitized = raw_distance_cm;
        if (sanitized <= 0.0f || sanitized > 400.0f) {
            sanitized = last_valid_distance;
        }

        // 2. Filtre médian glissant (Anti-Glitch)
        float median_val = median.update(sanitized);

        // 3. Prédiction Kalman basée sur le mouvement
        kalman.predict(dt, -robot_speed_cm_s); // Si le robot avance, la distance à l'obstacle diminue

        // 4. Correction Kalman avec la valeur médiane
        float filtered = kalman.update(median_val, dt);

        last_valid_distance = filtered;
        return filtered;
    }

    void reset(float initial_distance = 100.0f) {
        median.reset();
        ema.reset();
        kalman.reset(initial_distance);
        last_valid_distance = initial_distance;
    }

    float get_distance() const {
        return last_valid_distance;
    }
};
