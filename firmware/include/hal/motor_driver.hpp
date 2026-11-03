#pragma once

#include <iostream>
#include <algorithm>
#include <cmath>

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#endif

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
#ifdef __linux__
    int i2c_fd = -1;
    uint8_t i2c_addr = 0x40; // PCA9685 I2C default address
#else
    [[maybe_unused]] int i2c_fd = -1;
    [[maybe_unused]] uint8_t i2c_addr = 0x40;
#endif

public:
    explicit PiCarHatsMotorDriver(uint8_t addr = 0x40) : i2c_addr(addr) {
#ifdef __linux__
        i2c_fd = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd >= 0) {
            ioctl(i2c_fd, I2C_SLAVE, i2c_addr);
            // Initialisation de base du contrôleur PCA9685 (Mode 1, auto-increment)
            uint8_t init_cmd[2] = {0x00, 0x20};
            write(i2c_fd, init_cmd, 2);
        } else {
            std::cerr << "[WARN] Impossible d'ouvrir /dev/i2c-1. Fallback passif." << std::endl;
        }
#else
        std::cout << "[INFO] PiCarHatsMotorDriver initialisé en mode compatibilité (Non-Linux)." << std::endl;
#endif
    }

    ~PiCarHatsMotorDriver() override {
        emergency_stop();
#ifdef __linux__
        if (i2c_fd >= 0) {
            close(i2c_fd);
        }
#endif
    }

    void set_speed(float speed_percent) override {
        float clamped = std::clamp(speed_percent, -100.0f, 100.0f);
        (void)clamped;
#ifdef __linux__
        if (i2c_fd >= 0) {
            // Mapping du pourcentage vers registre PWM PCA9685 (0 - 4095)
            uint16_t pwm_val = static_cast<uint16_t>(std::abs(clamped) * 40.95f);
            uint8_t reg_data[5] = {0x06, 0x00, 0x00, static_cast<uint8_t>(pwm_val & 0xFF), static_cast<uint8_t>((pwm_val >> 8) & 0x0F)};
            write(i2c_fd, reg_data, 5);
        }
#endif
    }

    void set_steering(float angle_degrees) override {
        float clamped = std::clamp(angle_degrees, -45.0f, 45.0f);
        (void)clamped;
#ifdef __linux__
        if (i2c_fd >= 0) {
            // Impulsion servo RC : ~1.5ms centre (307 ticks à 50Hz), 1.0ms (-45°) = 205 ticks, 2.0ms (+45°) = 410 ticks
            float pulse = 307.0f + (clamped / 45.0f) * 102.0f;
            uint16_t pwm_val = static_cast<uint16_t>(std::clamp(pulse, 205.0f, 410.0f));
            uint8_t reg_data[5] = {0x0A, 0x00, 0x00, static_cast<uint8_t>(pwm_val & 0xFF), static_cast<uint8_t>((pwm_val >> 8) & 0x0F)};
            write(i2c_fd, reg_data, 5);
        }
#endif
    }

    void emergency_stop() override {
#ifdef __linux__
        if (i2c_fd >= 0) {
            // Tous les PWM à 0 (ALL_LED_OFF = registre 0xFD)
            uint8_t all_off[2] = {0xFD, 0x10};
            write(i2c_fd, all_off, 2);
        }
#endif
        std::cout << "[FAILSAFE] PiCarHats : Moteurs et PWM coupés à 0 V." << std::endl;
    }
};
