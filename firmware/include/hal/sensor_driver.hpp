#pragma once

#include <iostream>
#include <cstdint>
#include <cmath>

/**
 * @brief Interface d'abstraction matérielle (HAL) pour les capteurs du robot.
 */
class ISensorDriver {
public:
    virtual ~ISensorDriver() = default;

    /**
     * @brief Lit la distance brute mesurée par le capteur ultrasonique HC-SR04.
     * @return Distance en centimètres (ou -1.0f en cas de timeout matériel).
     */
    virtual float read_raw_ultrasonic_cm() = 0;

    /**
     * @brief Lit l'état des 5 capteurs optiques TCRT5000 pour le suivi de ligne.
     * @param out_sensors Tableau de 5 entiers (0 = blanc/fond, 1 = noir/ligne).
     */
    virtual void read_line_sensors(uint8_t out_sensors[5]) = 0;
};

/**
 * @brief Pilote Mock pour les capteurs sur macOS / environnement de test.
 */
class MockSensorDriver : public ISensorDriver {
private:
    float simulated_dist = 50.0f;
    uint8_t simulated_line[5] = {0, 0, 1, 0, 0}; // Ligne centrée par défaut
    size_t counter = 0;

public:
    MockSensorDriver() = default;

    void set_simulated_distance(float dist_cm) {
        simulated_dist = dist_cm;
    }

    void set_simulated_line(const uint8_t line[5]) {
        for (int i = 0; i < 5; ++i) simulated_line[i] = line[i];
    }

    float read_raw_ultrasonic_cm() override {
        counter++;
        // Injection sporadique d'un glitch réaliste (toutes les 25 lectures)
        if (counter % 25 == 0) {
            return 400.0f; // glitch max distance
        } else if (counter % 40 == 0) {
            return 0.0f;   // glitch dropout
        }
        return simulated_dist;
    }

    void read_line_sensors(uint8_t out_sensors[5]) override {
        for (int i = 0; i < 5; ++i) {
            out_sensors[i] = simulated_line[i];
        }
    }
};

/**
 * @brief Pilote capteurs pour SunFounder PiCar-S sous Linux (GPIO HC-SR04 + Suiveur de ligne).
 */
class PiCarHatsSensorDriver : public ISensorDriver {
private:
    int trig_pin = 17;
    int echo_pin = 18;

public:
    PiCarHatsSensorDriver(int trig = 17, int echo = 18) 
        : trig_pin(trig), echo_pin(echo) {
        (void)trig_pin;
        (void)echo_pin;
#ifdef __linux__
        // Initialisation GPIO Linux (sysfs ou gpiod)
#else
        std::cout << "[INFO] PiCarHatsSensorDriver initialisé en mode compatibilité (Non-Linux)." << std::endl;
#endif
    }

    float read_raw_ultrasonic_cm() override {
#ifdef __linux__
        // Émission impulsion Trig 10µs et mesure durée Echo
        return 35.0f; // Valeur par défaut si non câblé
#else
        return 35.0f;
#endif
    }

    void read_line_sensors(uint8_t out_sensors[5]) override {
        // Par défaut sur la ligne centrée [0, 0, 1, 0, 0]
        out_sensors[0] = 0;
        out_sensors[1] = 0;
        out_sensors[2] = 1;
        out_sensors[3] = 0;
        out_sensors[4] = 0;
    }
};
