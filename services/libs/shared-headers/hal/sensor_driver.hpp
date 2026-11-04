#pragma once

#include <iostream>
#include <cstdint>
#include <cmath>
#include <memory>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <fstream>
#include <sstream>
#include <string>

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
     * @return Distance en centimètres (ou -1.0f en cas de timeout ou panne matérielle).
     */
    virtual float read_raw_ultrasonic_cm() = 0;

    /**
     * @brief Lit l'état des 5 capteurs optiques TCRT5000 pour le suivi de ligne.
     * @param out_sensors Tableau de 5 entiers (0 = blanc/fond, 1 = noir/ligne).
     */
    virtual void read_line_sensors(uint8_t out_sensors[5]) = 0;

    /**
     * @brief Lit la "noirceur" continue des 5 capteurs optiques (sans binarisation).
     * @param out_darkness 5 valeurs dans [0.0, 1.0] (0 = blanc/fond, 1 = noir/ligne).
     * Par défaut, dérivée de la lecture binaire (0 ou 1) : les pilotes qui lisent un ADC la surchargent.
     */
    virtual void read_line_darkness(float out_darkness[5]) {
        uint8_t binary[5] = {0, 0, 0, 0, 0};
        read_line_sensors(binary);
        for (int i = 0; i < 5; ++i) out_darkness[i] = binary[i] ? 1.0f : 0.0f;
    }
};

/**
 * @brief Pilote Mock pour les capteurs pour tests unitaires et intégration sans robot physique.
 */
class MockSensorDriver : public ISensorDriver {
private:
    float simulated_dist = 50.0f;
    uint8_t simulated_line[5] = {0, 0, 0, 0, 0}; // Par défaut, aucune ligne si non initialisé
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
        // Injection sporadique d'un glitch de test (toutes les 25 lectures)
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
    int trig_pin = 12;
    int echo_pin = 16;
    int actual_trig = 12;
    int actual_echo = 16;
    picar::hal::I2CDevice m_line_device;
    picar::i2c::line_follower::CalibrationData m_calibration;
    uint16_t m_thresholds[5] = {picar::i2c::line_follower::DEFAULT_THRESHOLD,
                                picar::i2c::line_follower::DEFAULT_THRESHOLD,
                                picar::i2c::line_follower::DEFAULT_THRESHOLD,
                                picar::i2c::line_follower::DEFAULT_THRESHOLD,
                                picar::i2c::line_follower::DEFAULT_THRESHOLD};
    uint8_t m_last_sensors[5] = {0, 0, 0, 0, 0};
    float m_last_darkness[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    int m_consecutive_i2c_errors = 0;

    static float env_float(const char* name, float fallback) {
        const char* v = std::getenv(name);
        return (v && v[0] != '\0') ? static_cast<float>(std::atof(v)) : fallback;
    }

    void load_calibration() {
        float env_b = env_float("PICAR_LINE_BLACK_ADC", picar::i2c::line_follower::DEFAULT_BLACK_ADC);
        float env_w = env_float("PICAR_LINE_WHITE_ADC", picar::i2c::line_follower::DEFAULT_WHITE_ADC);
        for (int i = 0; i < 5; ++i) {
            m_calibration.black_adc[i] = env_b;
            m_calibration.white_adc[i] = env_w;
            m_thresholds[i] = static_cast<uint16_t>((env_b + env_w) * 0.5f);
        }

        const char* paths[] = {"/etc/picar_line_calibration", nullptr};
        const char* home = std::getenv("HOME");
        std::string home_path = home ? (std::string(home) + "/.picar_line_calibration") : "";

        const char* target_path = nullptr;
        std::ifstream file;
        file.open(paths[0]);
        if (file.is_open()) {
            target_path = paths[0];
        } else if (!home_path.empty()) {
            file.open(home_path);
            if (file.is_open()) {
                target_path = home_path.c_str();
            }
        }

        if (target_path && file.is_open()) {
            std::string line;
            int count = 0;
            while (std::getline(file, line) && count < 5) {
                if (line.empty() || line[0] == '#') continue;
                std::istringstream iss(line);
                float v1 = 0.0f, v2 = 0.0f, v3 = 0.0f;
                if (iss >> v1 >> v2) {
                    if (iss >> v3) {
                        int idx = static_cast<int>(v1);
                        if (idx >= 0 && idx < 5) {
                            m_calibration.black_adc[idx] = v2;
                            m_calibration.white_adc[idx] = v3;
                            m_thresholds[idx] = static_cast<uint16_t>((v2 + v3) * 0.5f);
                        }
                    } else {
                        m_calibration.black_adc[count] = v1;
                        m_calibration.white_adc[count] = v2;
                        m_thresholds[count] = static_cast<uint16_t>((v1 + v2) * 0.5f);
                    }
                    count++;
                }
            }
            std::cout << "[INFO] Calibration suiveur de ligne chargée depuis " << target_path << std::endl;
        }
    }

public:
    PiCarHatsSensorDriver(int trig = 12, int echo = 16) 
        : trig_pin(trig), 
          echo_pin(echo),
          actual_trig(trig),
          actual_echo(echo),
          m_line_device(picar::i2c::ADDR_LINE_FOLLOWER, picar::i2c::ADDR_LINE_FOLLOWER_ALT) {
        int base = detect_gpio_base();
        actual_trig = base + trig_pin;
        actual_echo = base + echo_pin;

        if (m_line_device.is_connected()) {
            std::cout << "[INFO] Suiveur de ligne PiCar-S connecté sur l'adresse I2C 0x" 
                      << std::hex << static_cast<int>(m_line_device.get_active_address()) << std::dec << std::endl;
        } else {
            std::cerr << "[WARN] Suiveur de ligne PiCar-S non détecté sur le bus I2C." << std::endl;
        }

        // Préparation des GPIO pour HC-SR04 via sysfs
        int export_fd = open("/sys/class/gpio/export", O_WRONLY);
        if (export_fd >= 0) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d", actual_trig);
            write(export_fd, buf, strlen(buf));
            snprintf(buf, sizeof(buf), "%d", actual_echo);
            write(export_fd, buf, strlen(buf));
            close(export_fd);
        }

        usleep(50000); // 50 ms pour la création des nœuds sysfs

        // Configuration explicite des directions
        char dir_path[64];
        snprintf(dir_path, sizeof(dir_path), "/sys/class/gpio/gpio%d/direction", actual_trig);
        int d_fd = open(dir_path, O_WRONLY);
        if (d_fd >= 0) {
            write(d_fd, "out", 3);
            close(d_fd);
        } else {
            std::cerr << "[WARN] Impossible de configurer Trig GPIO" << actual_trig << " (BCM " << trig_pin << ") en OUT" << std::endl;
        }

        snprintf(dir_path, sizeof(dir_path), "/sys/class/gpio/gpio%d/direction", actual_echo);
        d_fd = open(dir_path, O_WRONLY);
        if (d_fd >= 0) {
            write(d_fd, "in", 2);
            close(d_fd);
        } else {
            std::cerr << "[WARN] Impossible de configurer Echo GPIO" << actual_echo << " (BCM " << echo_pin << ") en IN" << std::endl;
        }

        std::cout << "[INFO] HC-SR04 configuré sur Trig=GPIO" << actual_trig << " (BCM " << trig_pin 
                  << "), Echo=GPIO" << actual_echo << " (BCM " << echo_pin << ")" << std::endl;

        load_calibration();
    }

    const picar::i2c::line_follower::CalibrationData& get_calibration() const {
        return m_calibration;
    }

    void set_calibration(const picar::i2c::line_follower::CalibrationData& calib) {
        m_calibration = calib;
        for (int i = 0; i < 5; ++i) {
            m_thresholds[i] = static_cast<uint16_t>((calib.black_adc[i] + calib.white_adc[i]) * 0.5f);
        }
    }

    ~PiCarHatsSensorDriver() override {
        int unexport_fd = open("/sys/class/gpio/unexport", O_WRONLY);
        if (unexport_fd >= 0) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d", actual_trig);
            write(unexport_fd, buf, strlen(buf));
            snprintf(buf, sizeof(buf), "%d", actual_echo);
            write(unexport_fd, buf, strlen(buf));
            close(unexport_fd);
        }
    }

    static int detect_gpio_base() {
        DIR* dir = opendir("/sys/class/gpio");
        if (!dir) return 0;
        struct dirent* entry;
        int found_base = 0;
        while ((entry = readdir(dir)) != nullptr) {
            if (strncmp(entry->d_name, "gpiochip", 8) == 0) {
                char label_path[256];
                snprintf(label_path, sizeof(label_path), "/sys/class/gpio/%s/label", entry->d_name);
                int lfd = open(label_path, O_RDONLY);
                if (lfd >= 0) {
                    char label[64] = {0};
                    ssize_t n = read(lfd, label, sizeof(label) - 1);
                    close(lfd);
                    if (n > 0 && (strstr(label, "bcm2835") || strstr(label, "3f200000.gpio") || strstr(label, "fe200000.gpio"))) {
                        char base_path[256];
                        snprintf(base_path, sizeof(base_path), "/sys/class/gpio/%s/base", entry->d_name);
                        int bfd = open(base_path, O_RDONLY);
                        if (bfd >= 0) {
                            char base_str[32] = {0};
                            ssize_t bn = read(bfd, base_str, sizeof(base_str) - 1);
                            close(bfd);
                            if (bn > 0) {
                                found_base = std::atoi(base_str);
                                break;
                            }
                        }
                    }
                }
            }
        }
        closedir(dir);
        return found_base;
    }

    void set_thresholds(const uint16_t thresholds[5]) {
        for (int i = 0; i < 5; ++i) m_thresholds[i] = thresholds[i];
    }

    float read_raw_ultrasonic_cm() override {
        char trig_path[64];
        char echo_path[64];
        snprintf(trig_path, sizeof(trig_path), "/sys/class/gpio/gpio%d/value", actual_trig);
        snprintf(echo_path, sizeof(echo_path), "/sys/class/gpio/gpio%d/value", actual_echo);

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
        return -1.0f; // Panne matérielle ou GPIO inaccessibles
    }

    bool is_line_sensor_connected() const {
        return m_line_device.is_connected() && (m_consecutive_i2c_errors < 5);
    }

    void read_line_sensors(uint8_t out_sensors[5]) override {
        if (m_line_device.is_connected()) {
            uint8_t buffer[picar::i2c::line_follower::RAW_BUFFER_SIZE] = {0};
            if (m_line_device.read_block_data(picar::i2c::line_follower::REG_START, 
                                              buffer, 
                                              picar::i2c::line_follower::RAW_BUFFER_SIZE)) {
                m_consecutive_i2c_errors = 0;
                uint16_t raw_adc[5];
                picar::i2c::line_follower::parse_raw_adc(buffer, raw_adc);
                picar::i2c::line_follower::binarize_sensors(raw_adc, m_thresholds, m_last_sensors);
                picar::i2c::line_follower::convert_raw_to_darkness(raw_adc, m_calibration, m_last_darkness);
            } else {
                m_consecutive_i2c_errors++;
            }
        } else {
            m_consecutive_i2c_errors++;
        }

        // Si l'I2C échoue plus de 3 cycles (~60 ms), fallback strict sur 0 0 0 0 0 (ligne perdue)
        if (m_consecutive_i2c_errors >= 3) {
            for (int i = 0; i < 5; ++i) {
                m_last_sensors[i] = 0;
                m_last_darkness[i] = 0.0f;
            }
        }

        for (int i = 0; i < 5; ++i) {
            out_sensors[i] = m_last_sensors[i];
        }
    }

    /// Lecture analogique : valeurs ADC brutes converties en noirceur continue [0, 1].
    void read_line_darkness(float out_darkness[5]) override {
        uint8_t unused[5];
        read_line_sensors(unused); // rafraîchit m_last_darkness (garde la dernière valeur si l'I2C échoue)
        for (int i = 0; i < 5; ++i) {
            out_darkness[i] = m_last_darkness[i];
        }
    }
};
