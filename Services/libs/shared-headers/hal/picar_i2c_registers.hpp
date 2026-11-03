#pragma once

#include <cstdint>
#include <algorithm>
#include <cmath>

namespace picar::i2c {

    // =========================================================================
    // 1. ADRESSES DU BUS I2C (/dev/i2c-1)
    // =========================================================================
    inline constexpr uint8_t ADDR_PCA9685           = 0x40; // Driver PWM & Servos
    inline constexpr uint8_t ADDR_PCA9685_ALLCALL   = 0x70; // All-Call général
    inline constexpr uint8_t ADDR_LINE_FOLLOWER     = 0x11; // Suiveur de ligne (défaut GitHub)
    inline constexpr uint8_t ADDR_LINE_FOLLOWER_ALT = 0x0B; // Suiveur de ligne (alternative doc/décimal 11)
    inline constexpr uint8_t ADDR_PCF8591_ADC       = 0x48; // ADC Robot HAT (capteurs lumière / batterie)

    // =========================================================================
    // 2. REGISTRES PCA9685 (0x40)
    // =========================================================================
    namespace pca9685 {
        inline constexpr uint8_t MODE1              = 0x00;
        inline constexpr uint8_t MODE2              = 0x01;
        inline constexpr uint8_t SUBADR1            = 0x02;
        inline constexpr uint8_t SUBADR2            = 0x03;
        inline constexpr uint8_t SUBADR3            = 0x04;
        inline constexpr uint8_t ALLCALLADR         = 0x05;
        inline constexpr uint8_t LED0_ON_L          = 0x06;
        inline constexpr uint8_t LED0_ON_H          = 0x07;
        inline constexpr uint8_t LED0_OFF_L         = 0x08;
        inline constexpr uint8_t LED0_OFF_H         = 0x09;
        inline constexpr uint8_t ALL_LED_ON_L       = 0xFA;
        inline constexpr uint8_t ALL_LED_ON_H       = 0xFB;
        inline constexpr uint8_t ALL_LED_OFF_L      = 0xFC;
        inline constexpr uint8_t ALL_LED_OFF_H      = 0xFD;
        inline constexpr uint8_t PRESCALE           = 0xFE;

        // Bits de configuration MODE1
        inline constexpr uint8_t RESTART            = 0x80;
        inline constexpr uint8_t EXTCLK             = 0x40;
        inline constexpr uint8_t AUTO_INC           = 0x20;
        inline constexpr uint8_t SLEEP              = 0x10;
        inline constexpr uint8_t SUB1               = 0x08;
        inline constexpr uint8_t SUB2               = 0x04;
        inline constexpr uint8_t SUB3               = 0x02;
        inline constexpr uint8_t ALLCALL            = 0x01;

        // Bits de configuration MODE2
        inline constexpr uint8_t INVRT              = 0x10;
        inline constexpr uint8_t OCH                = 0x08;
        inline constexpr uint8_t OUTDRV             = 0x04;
        inline constexpr uint8_t OUTNE1             = 0x02;
        inline constexpr uint8_t OUTNE0             = 0x01;

        // Affectation des canaux PiCar-S
        inline constexpr uint8_t CH_STEERING_SERVO  = 0; // Servo direction avant
        inline constexpr uint8_t CH_MOTOR_L_PWM     = 4; // PWM Moteur gauche (TB6612 PWM_A)
        inline constexpr uint8_t CH_MOTOR_R_PWM     = 5; // PWM Moteur droit (TB6612 PWM_B)

        /**
         * @brief Calcule le registre de base (LEDn_ON_L) pour un canal donné [0-15].
         */
        constexpr uint8_t channel_base_reg(uint8_t channel) {
            return static_cast<uint8_t>(LED0_ON_L + 4 * channel);
        }

        /**
         * @brief Calcule la valeur du prescaler pour une fréquence désirée (en Hz).
         * Horloge interne PCA9685 standard : 25 MHz, résolution 4096 pas.
         */
        inline uint8_t calculate_prescale(float freq_hz, float osc_clock_hz = 25000000.0f) {
            float val = std::round(osc_clock_hz / (4096.0f * freq_hz)) - 1.0f;
            return static_cast<uint8_t>(std::clamp(val, 3.0f, 255.0f));
        }

        /**
         * @brief Convertit un angle de braquage [-45.0°, +45.0°] en ticks PWM (0-4095) à 50 Hz.
         * Centre ~1.5 ms (~307 ticks), -45° ~1.0 ms (~205 ticks), +45° ~2.0 ms (~410 ticks).
         */
        inline uint16_t steering_angle_to_pwm(float angle_deg) {
            float clamped = std::clamp(angle_deg, -45.0f, 45.0f);
            float pulse = 307.0f + (clamped / 45.0f) * 102.0f;
            return static_cast<uint16_t>(std::clamp(pulse, 205.0f, 410.0f));
        }

        /**
         * @brief Convertit un pourcentage de vitesse [0.0%, 100.0%] en ticks PWM (0-4095).
         */
        inline uint16_t speed_percent_to_pwm(float speed_pct) {
            float clamped = std::clamp(std::abs(speed_pct), 0.0f, 100.0f);
            return static_cast<uint16_t>(clamped * 40.95f);
        }
    }

    // =========================================================================
    // 3. REGISTRES DU SUIVEUR DE LIGNE 5 VOIES (0x11 / 0x0B)
    // =========================================================================
    namespace line_follower {
        inline constexpr uint8_t REG_START              = 0x00; // Registre de lecture continue
        inline constexpr size_t  RAW_BUFFER_SIZE        = 10;   // 5 canaux x 2 octets (ADC 10 bits)
        inline constexpr uint16_t DEFAULT_THRESHOLD     = 60;   // Seuil noir/blanc calibré PiCar-S (noir < 50-60, blanc > 70-140)

        /**
         * @brief Parse les 10 octets bruts I2C en 5 valeurs ADC 10 bits (0-1023).
         */
        inline void parse_raw_adc(const uint8_t raw_data[10], uint16_t out_adc[5]) {
            for (size_t i = 0; i < 5; ++i) {
                uint16_t high = static_cast<uint16_t>(raw_data[i * 2]) << 8;
                uint16_t low  = static_cast<uint16_t>(raw_data[i * 2 + 1]);
                out_adc[i] = high | low;
            }
        }

        /**
         * @brief Convertit les 5 valeurs ADC en bits digitaux [0 = blanc, 1 = noir].
         * Pour les capteurs optiques TCRT5000 avec fond blanc réflectif, 
         * une surface sombre/noire réfléchit moins de lumière infrarouge (valeur ADC < seuil).
         */
        inline void binarize_sensors(const uint16_t adc_values[5], 
                                     const uint16_t thresholds[5], 
                                     uint8_t out_binary[5]) {
            for (size_t i = 0; i < 5; ++i) {
                out_binary[i] = (adc_values[i] < thresholds[i]) ? 1 : 0;
            }
        }
    }

    // =========================================================================
    // 4. COMMANDES PCF8591 ADC (0x48)
    // =========================================================================
    namespace pcf8591 {
        inline constexpr uint8_t CTRL_CH0_LIGHT_L       = 0x40; // Canal AIN0 : Photorésistance Gauche
        inline constexpr uint8_t CTRL_CH1_LIGHT_C       = 0x41; // Canal AIN1 : Photorésistance Centre
        inline constexpr uint8_t CTRL_CH2_LIGHT_R       = 0x42; // Canal AIN2 : Photorésistance Droite
        inline constexpr uint8_t CTRL_CH3_BATTERY       = 0x43; // Canal AIN3 : Surveillance tension batterie
        inline constexpr uint8_t CTRL_AUTO_INC          = 0x04; // Auto-incrément de canal
        inline constexpr uint8_t CTRL_OUTPUT_ENABLE     = 0x40; // Activation sortie DAC analogique
    }

} // namespace picar::i2c
