#pragma once

#include <iostream>
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>

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
    explicit PiCarHatsMotorDriver(uint8_t addr = picar::i2c::ADDR_PCA9685) 
        : m_i2c(addr) {
#ifdef __linux__
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
        std::cout << "[INFO] PiCarHatsMotorDriver initialisé en mode compatibilité (Non-Linux)." << std::endl;
#endif
    }

    ~PiCarHatsMotorDriver() override {
        emergency_stop();
    }

    void set_speed(float speed_percent) override {
        float clamped = std::clamp(speed_percent, -100.0f, 100.0f);
        uint16_t pwm_ticks = picar::i2c::pca9685::speed_percent_to_pwm(clamped);

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
