#pragma once

#include <cmath>

/**
 * @brief Filtre de Kalman 1D avec rejet des accélérations impossibles.
 * Conforme à la Section 7 du PiCar Master Architecture & Engineering Manual.
 * Utilisé pour la fusion capteur (modèle moteur + mesure ultrasonore)
 * et l'anticipation de trajectoire avec rejet des dérivées d'accélération impossibles.
 */
class KalmanFilter1D {
private:
    float x; // Estimation de l'état (ex: distance en cm)
    float p; // Covariance de l'erreur d'estimation
    float q; // Bruit de processus (process noise)
    float r; // Bruit de mesure (measurement noise)
    float k; // Gain de Kalman

    float last_velocity;     // Dernière vitesse estimée (cm/s)
    float max_accel;         // Accélération maximale physique admissible (ex: 73.5 cm/s^2 <=> 0.735 m/s^2)
    bool initialized;

public:
    /**
     * @param process_noise Incertitude du modèle de mouvement (Q)
     * @param measurement_noise Variance du bruit de capteur (R)
     * @param max_physical_accel Accélération max en cm/s^2 (défaut: 73.5 cm/s^2 correspondant à 0.735 m/s^2)
     */
    KalmanFilter1D(float process_noise = 0.05f, 
                   float measurement_noise = 4.0f, 
                   float max_physical_accel = 73.5f)
        : x(0.0f),
          p(1.0f),
          q(process_noise),
          r(measurement_noise),
          k(0.0f),
          last_velocity(0.0f),
          max_accel(max_physical_accel),
          initialized(false) {}

    /**
     * @brief Phase de prédiction du modèle dynamique.
     * @param dt Intervalle de temps depuis la dernière itération (en secondes).
     * @param velocity_command Vitesse actuelle du robot (en cm/s).
     */
    void predict(float dt, float velocity_command = 0.0f) {
        if (!initialized) return;

        // Prédiction cinématique basée sur la commande moteur
        x += velocity_command * dt;
        p += q;
    }

    /**
     * @brief Phase de mise à jour / correction par la mesure du capteur.
     * @param measurement Mesure brute ou pré-filtrée (ex: ultrason en cm).
     * @param dt Intervalle de temps écoulé (en secondes).
     * @return Nouvelle estimation filtrée x.
     */
    float update(float measurement, float dt = 0.02f) {
        if (!initialized) {
            x = measurement;
            initialized = true;
            return x;
        }

        // Rejet des accélérations physiquement impossibles (gating)
        if (dt > 0.001f) {
            float apparent_velocity = (measurement - x) / dt;
            float apparent_accel = std::abs(apparent_velocity - last_velocity) / dt;

            // Si la variation implique une accélération 3x supérieure au maximum physique de la bille
            // (ex: glitch de réflexion ultrasonore non attrapé par le médian), on pondère lourdement
            if (apparent_accel > (max_accel * 3.0f)) {
                // Mesure suspecte : on n'augmente pas la confiance en la mesure (R effectif gonflé)
                float r_inflated = r * 10.0f;
                k = p / (p + r_inflated);
                x = x + k * (measurement - x);
                p = (1.0f - k) * p;
                return x;
            }

            last_velocity = apparent_velocity;
        }

        // Mise à jour standard de Kalman
        k = p / (p + r);
        x = x + k * (measurement - x);
        p = (1.0f - k) * p;

        return x;
    }

    void reset(float initial_value = 0.0f) {
        x = initial_value;
        p = 1.0f;
        last_velocity = 0.0f;
        initialized = false;
    }

    float get_estimate() const { return x; }
    float get_kalman_gain() const { return k; }
    bool is_initialized() const { return initialized; }
};
