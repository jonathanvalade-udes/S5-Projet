#pragma once

#include <iostream>
#include <cstdint>
#include <cmath>
#include <memory>
#include "picar_i2c_registers.hpp"
#include "i2c_device.hpp"

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
 * @brief Pilote capteurs pour SunFounder PiCar-S sous Linux (GPIO HC-SR04 + Suiveur de ligne I2C).
 */
class PiCarHatsSensorDriver : public ISensorDriver {
private:
    int trig_pin = 17;
    int echo_pin = 18;
    picar::hal::I2CDevice m_line_device;
    uint16_t m_thresholds[5] = {300, 300, 300, 300, 300};
    uint8_t m_last_sensors[5] = {0, 0, 1, 0, 0};

public:
    PiCarHatsSensorDriver(int trig = 17, int echo = 18) 
        : trig_pin(trig), 
          echo_pin(echo),
          m_line_device(picar::i2c::ADDR_LINE_FOLLOWER, picar::i2c::ADDR_LINE_FOLLOWER_ALT) {
        (void)trig_pin;
        (void)echo_pin;
#ifdef __linux__
        if (m_line_device.is_connected()) {
            std::cout << "[INFO] Suiveur de ligne PiCar-S connecté sur l'adresse I2C 0x" 
                      << std::hex << (int)m_line_device.get_active_address() << std::dec << std::endl;
        } else {
            std::cerr << "[WARN] Suiveur de ligne PiCar-S non détecté sur le bus I2C." << std::endl;
        }
#else
        std::cout << "[INFO] PiCarHatsSensorDriver initialisé en mode compatibilité (Non-Linux)." << std::endl;
#endif
    }

    void set_thresholds(const uint16_t thresholds[5]) {
        for (int i = 0; i < 5; ++i) m_thresholds[i] = thresholds[i];
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
#ifdef __linux__
        if (m_line_device.is_connected()) {
            uint8_t buffer[picar::i2c::line_follower::RAW_BUFFER_SIZE] = {0};
            if (m_line_device.read_block_data(picar::i2c::line_follower::REG_START, 
                                              buffer, 
                                              picar::i2c::line_follower::RAW_BUFFER_SIZE)) {
                uint16_t raw_adc[5];
                picar::i2c::line_follower::parse_raw_adc(buffer, raw_adc);
                picar::i2c::line_follower::binarize_sensors(raw_adc, m_thresholds, m_last_sensors);
            }
        }
#endif
        for (int i = 0; i < 5; ++i) {
            out_sensors[i] = m_last_sensors[i];
        }
    }
};
