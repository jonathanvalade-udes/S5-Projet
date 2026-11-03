#pragma once

#include <iostream>
#include <cstdint>
#include <cmath>
#include <memory>
#include <chrono>
#include <cstring>
#include <cstdio>
#include "picar_i2c_registers.hpp"
#include "i2c_device.hpp"

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#endif

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
 * @brief Pilote Mock pour les capteurs sur macOS / Windows / environnement de test.
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
 * @brief Pilote capteurs pour SunFounder PiCar-S sous Linux :
 * - Bus I2C pour le suiveur de ligne 5 voies (TCRT5000 via I2CDevice)
 * - GPIO Trigger/Echo pour le sonar ultrason HC-SR04 avec timeout non-bloquant
 */
class PiCarHatsSensorDriver : public ISensorDriver {
private:
    int trig_pin = 17;
    int echo_pin = 18;
    picar::hal::I2CDevice m_line_device;
    uint16_t m_thresholds[5] = {picar::i2c::line_follower::DEFAULT_THRESHOLD,
                                picar::i2c::line_follower::DEFAULT_THRESHOLD,
                                picar::i2c::line_follower::DEFAULT_THRESHOLD,
                                picar::i2c::line_follower::DEFAULT_THRESHOLD,
                                picar::i2c::line_follower::DEFAULT_THRESHOLD};
    uint8_t m_last_sensors[5] = {0, 0, 1, 0, 0};

public:
    PiCarHatsSensorDriver(int trig = 17, int echo = 18) 
        : trig_pin(trig), 
          echo_pin(echo),
          m_line_device(picar::i2c::ADDR_LINE_FOLLOWER, picar::i2c::ADDR_LINE_FOLLOWER_ALT) {
#ifdef __linux__
        if (m_line_device.is_connected()) {
            std::cout << "[INFO] Suiveur de ligne PiCar-S connecté sur l'adresse I2C 0x" 
                      << std::hex << static_cast<int>(m_line_device.get_active_address()) << std::dec << std::endl;
        } else {
            std::cerr << "[WARN] Suiveur de ligne PiCar-S non détecté sur le bus I2C (mode fallback actif)." << std::endl;
        }

        // Préparation des GPIO pour HC-SR04 via sysfs
        int export_fd = open("/sys/class/gpio/export", O_WRONLY);
        if (export_fd >= 0) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d", trig_pin);
            write(export_fd, buf, strlen(buf));
            snprintf(buf, sizeof(buf), "%d", echo_pin);
            write(export_fd, buf, strlen(buf));
            close(export_fd);
        }
#else
        (void)trig_pin;
        (void)echo_pin;
        std::cout << "[INFO] PiCarHatsSensorDriver initialisé en mode compatibilité (Non-Linux)." << std::endl;
#endif
    }

    void set_thresholds(const uint16_t thresholds[5]) {
        for (int i = 0; i < 5; ++i) m_thresholds[i] = thresholds[i];
    }

    float read_raw_ultrasonic_cm() override {
#ifdef __linux__
        char trig_path[64];
        char echo_path[64];
        snprintf(trig_path, sizeof(trig_path), "/sys/class/gpio/gpio%d/value", trig_pin);
        snprintf(echo_path, sizeof(echo_path), "/sys/class/gpio/gpio%d/value", echo_pin);

        int t_fd = open(trig_path, O_WRONLY);
        int e_fd = open(echo_path, O_RDONLY);

        if (t_fd >= 0 && e_fd >= 0) {
            // Impulsion Trigger de 10 microsecondes
            write(t_fd, "0", 1);
            usleep(2);
            write(t_fd, "1", 1);
            usleep(10);
            write(t_fd, "0", 1);
            close(t_fd);

            // Attente du front montant (Echo haut) avec timeout de sécurité de 5 ms
            auto start_wait = std::chrono::steady_clock::now();
            char val = '0';
            while (val == '0') {
                lseek(e_fd, 0, SEEK_SET);
                read(e_fd, &val, 1);
                if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_wait).count() > 5) {
                    close(e_fd);
                    return -1.0f; // Timeout / pas d'écho
                }
            }

            // Mesure de la durée de l'Echo
            auto pulse_start = std::chrono::steady_clock::now();
            while (val == '1') {
                lseek(e_fd, 0, SEEK_SET);
                read(e_fd, &val, 1);
                if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - pulse_start).count() > 25) {
                    break; // Timeout de distance maximale (~400 cm)
                }
            }
            auto pulse_end = std::chrono::steady_clock::now();
            close(e_fd);

            float duration_us = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(pulse_end - pulse_start).count());
            float distance_cm = duration_us / 58.0f; // Formule acoustique : distance = temps / 58
            return distance_cm;
        }

        if (t_fd >= 0) close(t_fd);
        if (e_fd >= 0) close(e_fd);
        return 35.0f; // Repli par défaut si GPIO non câblés
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
