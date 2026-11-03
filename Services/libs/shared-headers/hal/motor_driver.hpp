#pragma once

#include <iostream>
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>
#include <cstring>
#include <cstdlib>

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#endif

#include "picar_i2c_registers.hpp"
#include "i2c_device.hpp"

/**
 * @brief Interface d'abstraction matérielle (HAL) pour le contrôle des moteurs et de la direction.
 */
class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;

    /**
     * @brief Définit la consigne de vitesse du robot.
     * @param speed_percent Pourcentage de vitesse [-100.0f, +100.0f] (négatif = recul).
     */
    virtual void set_speed(float speed_percent) = 0;

    /**
     * @brief Définit l'angle de braquage de la direction.
     * @param angle_degrees Angle en degrés [-45.0f, +45.0f] (négatif = gauche, positif = droite).
     */
    virtual void set_steering(float angle_degrees) = 0;

    /**
     * @brief Coupe immédiatement l'alimentation des ponts en H et place les PWM à 0 V (Failsafe).
     */
    virtual void emergency_stop() = 0;
};

/**
 * @brief Pilote Mock / Simulation utilisé sur macOS pour tests et développement sans robot physique.
 */
class MockMotorDriver : public IMotorDriver {
private:
    float current_speed = 0.0f;
    float current_steering = 0.0f;
    bool stopped = true;
    bool verbose = false;

public:
    explicit MockMotorDriver(bool verbose_logs = false) : verbose(verbose_logs) {}

    void set_speed(float speed_percent) override {
        current_speed = std::clamp(speed_percent, -100.0f, 100.0f);
        stopped = (std::abs(current_speed) < 0.001f);
        if (verbose) {
            std::cout << "[MOCK_MOTOR] Vitesse fixée à : " << current_speed << "%" << std::endl;
        }
    }

    void set_steering(float angle_degrees) override {
        current_steering = std::clamp(angle_degrees, -45.0f, 45.0f);
        if (verbose) {
            std::cout << "[MOCK_MOTOR] Direction fixée à : " << current_steering << "°" << std::endl;
        }
    }

    void emergency_stop() override {
        current_speed = 0.0f;
        current_steering = 0.0f;
        stopped = true;
        std::cout << "[MOCK_MOTOR] >>> ARRÊT D'URGENCE FAILSAFE : Sorties forcées à 0V <<<" << std::endl;
    }

    float get_speed() const { return current_speed; }
    float get_steering() const { return current_steering; }
    bool is_stopped() const { return stopped; }
};

/**
 * @brief Pilote matériel officiel SunFounder PiCar-S (Robot HATS / PCA9685 PWM + TB6612).
 * Actif sur Raspberry Pi sous Linux.
 */
class PiCarHatsMotorDriver : public IMotorDriver {
private:
    picar::hal::I2CDevice m_i2c;
    int dir_a_pin = 17;
    int dir_b_pin = 27;
    int actual_dir_a = 17;
    int actual_dir_b = 27;
    int dir_a_fd = -1;
    int dir_b_fd = -1;
    bool m_invert_direction = false;

    static int detect_gpio_base() {
#ifdef __linux__
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
#else
        return 0;
#endif
    }

    void write_channel_pwm(uint8_t channel, uint16_t on_ticks, uint16_t off_ticks) {
        uint8_t base_reg = picar::i2c::pca9685::channel_base_reg(channel);
        uint8_t data[4] = {
            static_cast<uint8_t>(on_ticks & 0xFF),
            static_cast<uint8_t>((on_ticks >> 8) & 0x0F),
            static_cast<uint8_t>(off_ticks & 0xFF),
            static_cast<uint8_t>((off_ticks >> 8) & 0x0F)
        };
        m_i2c.write_block_data(base_reg, data, 4);
    }

public:
    explicit PiCarHatsMotorDriver(uint8_t addr = picar::i2c::ADDR_PCA9685, bool invert_direction = false) 
        : m_i2c(addr), m_invert_direction(invert_direction) {
#ifdef __linux__
        int base = detect_gpio_base();
        actual_dir_a = base + dir_a_pin;
        actual_dir_b = base + dir_b_pin;

        // Configuration des GPIO sysfs pour TB6612 Direction (GPIO 17 & 27)
        int export_fd = open("/sys/class/gpio/export", O_WRONLY);
        if (export_fd >= 0) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d", actual_dir_a);
            write(export_fd, buf, strlen(buf));
            snprintf(buf, sizeof(buf), "%d", actual_dir_b);
            write(export_fd, buf, strlen(buf));
            close(export_fd);
        }

        usleep(50000); // 50 ms pour la création des nœuds sysfs

        char dir_path[64];
        snprintf(dir_path, sizeof(dir_path), "/sys/class/gpio/gpio%d/direction", actual_dir_a);
        int d_fd = open(dir_path, O_WRONLY);
        if (d_fd >= 0) {
            write(d_fd, "out", 3);
            close(d_fd);
        }
        snprintf(dir_path, sizeof(dir_path), "/sys/class/gpio/gpio%d/direction", actual_dir_b);
        d_fd = open(dir_path, O_WRONLY);
        if (d_fd >= 0) {
            write(d_fd, "out", 3);
            close(d_fd);
        }

        char val_path[64];
        snprintf(val_path, sizeof(val_path), "/sys/class/gpio/gpio%d/value", actual_dir_a);
        dir_a_fd = open(val_path, O_WRONLY);

        snprintf(val_path, sizeof(val_path), "/sys/class/gpio/gpio%d/value", actual_dir_b);
        dir_b_fd = open(val_path, O_WRONLY);

        std::cout << "[INFO] Pont en H TB6612 configuré (DirA=GPIO" << actual_dir_a 
                  << ", DirB=GPIO" << actual_dir_b << ", Invert=" << (m_invert_direction ? "true" : "false") << ")" << std::endl;

        if (m_i2c.is_connected()) {
            // 1. Mise en veille pour autoriser la configuration du prescaler
            m_i2c.write_byte_data(picar::i2c::pca9685::MODE1, picar::i2c::pca9685::SLEEP);
            
            // 2. Réglage de la fréquence à 50 Hz pour les servomoteurs RC
            uint8_t prescale = picar::i2c::pca9685::calculate_prescale(50.0f);
            m_i2c.write_byte_data(picar::i2c::pca9685::PRESCALE, prescale);

            // 3. Réactivation avec auto-incrémentation d'adresses de registres
            m_i2c.write_byte_data(picar::i2c::pca9685::MODE1, picar::i2c::pca9685::AUTO_INC);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            // 4. Sorties en Totem-Pole (Push-Pull)
            m_i2c.write_byte_data(picar::i2c::pca9685::MODE2, picar::i2c::pca9685::OUTDRV);

            // 5. Positionnement par défaut : direction centrée (0° = 1.5ms)
            set_steering(0.0f);

            std::cout << "[INFO] PCA9685 configuré avec succès à 50 Hz sur I2C 0x" 
                      << std::hex << (int)addr << std::dec << std::endl;
        } else {
            std::cerr << "[WARN] PiCarHatsMotorDriver : PCA9685 non détecté sur le bus I2C." << std::endl;
        }
#else
        (void)addr;
        (void)dir_a_pin;
        (void)dir_b_pin;
        (void)actual_dir_a;
        (void)actual_dir_b;
        (void)dir_a_fd;
        (void)dir_b_fd;
        (void)m_invert_direction;
        std::cout << "[INFO] PiCarHatsMotorDriver initialisé en mode compatibilité (Non-Linux)." << std::endl;
#endif
    }

    ~PiCarHatsMotorDriver() override {
        emergency_stop();
#ifdef __linux__
        if (dir_a_fd >= 0) { close(dir_a_fd); dir_a_fd = -1; }
        if (dir_b_fd >= 0) { close(dir_b_fd); dir_b_fd = -1; }
        int unexport_fd = open("/sys/class/gpio/unexport", O_WRONLY);
        if (unexport_fd >= 0) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d", actual_dir_a);
            write(unexport_fd, buf, strlen(buf));
            snprintf(buf, sizeof(buf), "%d", actual_dir_b);
            write(unexport_fd, buf, strlen(buf));
            close(unexport_fd);
        }
#endif
    }

    void set_speed(float speed_percent) override {
        float clamped = std::clamp(speed_percent, -100.0f, 100.0f);
        uint16_t pwm_ticks = picar::i2c::pca9685::speed_percent_to_pwm(clamped);

#ifdef __linux__
        // Par défaut sur PiCar-S avec TB6612 :
        // Pour AVANCER (speed >= 0) : broches à HIGH ("1")
        // Pour RECULER (speed < 0)  : broches à LOW ("0")
        bool forward = (clamped >= 0.0f);
        if (m_invert_direction) {
            forward = !forward;
        }

        const char* val = forward ? "1" : "0";
        if (dir_a_fd >= 0) {
            pwrite(dir_a_fd, val, 1, 0);
        }
        if (dir_b_fd >= 0) {
            pwrite(dir_b_fd, val, 1, 0);
        }
#endif

        // Pilotage simultané des deux roues motrices arrière (Canaux 4 et 5)
        write_channel_pwm(picar::i2c::pca9685::CH_MOTOR_L_PWM, 0, pwm_ticks);
        write_channel_pwm(picar::i2c::pca9685::CH_MOTOR_R_PWM, 0, pwm_ticks);
    }

    void set_steering(float angle_degrees) override {
        uint16_t pwm_ticks = picar::i2c::pca9685::steering_angle_to_pwm(angle_degrees);

        // Pilotage du servo de direction (Canal 0)
        write_channel_pwm(picar::i2c::pca9685::CH_STEERING_SERVO, 0, pwm_ticks);
    }

    void emergency_stop() override {
        // Arrêt d'urgence matériel : forcer ALL_LED_OFF à 1
        m_i2c.write_byte_data(picar::i2c::pca9685::ALL_LED_OFF_H, 0x10);
        std::cout << "[FAILSAFE] PiCarHats : Moteurs et PWM coupés à 0 V." << std::endl;
    }
};
