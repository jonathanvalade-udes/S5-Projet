#include <iostream>
#include <cassert>
#include <vector>
#include <cmath>

#include "hal/picar_i2c_registers.hpp"
#include "hal/i2c_device.hpp"
#include "hal/motor_driver.hpp"
#include "hal/sensor_driver.hpp"

void test_i2c_addresses_and_constants() {
    std::cout << "[TEST] 1. Vérification des adresses I2C & registres matériels..." << std::endl;

    using namespace picar::i2c;
    assert(ADDR_PCA9685 == 0x40);
    assert(ADDR_PCA9685_ALLCALL == 0x70);
    assert(ADDR_LINE_FOLLOWER == 0x11);
    assert(ADDR_LINE_FOLLOWER_ALT == 0x0B);
    assert(ADDR_PCF8591_ADC == 0x48);

    // Registres PCA9685
    assert(pca9685::MODE1 == 0x00);
    assert(pca9685::PRESCALE == 0xFE);
    assert(pca9685::ALL_LED_OFF_H == 0xFD);

    // Canaux PiCar-S
    assert(pca9685::CH_STEERING_SERVO == 0);
    assert(pca9685::CH_MOTOR_L_PWM == 4);
    assert(pca9685::CH_MOTOR_R_PWM == 5);

    std::cout << "  -> PASS: Adresses et constantes I2C validées." << std::endl;
}

void test_pca9685_math_and_registers() {
    std::cout << "[TEST] 2. Vérification des formules et registres PCA9685..." << std::endl;

    using namespace picar::i2c::pca9685;

    // Registres de base des canaux
    assert(channel_base_reg(0) == 0x06); // LED0_ON_L
    assert(channel_base_reg(4) == 0x16); // LED4_ON_L (0x06 + 4*4 = 0x16)
    assert(channel_base_reg(5) == 0x1A); // LED5_ON_L (0x06 + 4*5 = 0x1A)
    assert(channel_base_reg(15) == 0x42); // LED15_ON_L (0x06 + 4*15 = 0x42)

    // Calcul prescaler 50 Hz
    uint8_t prescale_50hz = calculate_prescale(50.0f);
    assert(prescale_50hz == 121); // round(25MHz / (4096 * 50)) - 1 = 121

    // Servo direction 50Hz (~1.5ms centre = ~307 ticks, +/- 45 deg)
    uint16_t centre = steering_angle_to_pwm(0.0f);
    uint16_t left_max = steering_angle_to_pwm(-45.0f);
    uint16_t right_max = steering_angle_to_pwm(45.0f);

    assert(centre >= 305 && centre <= 309);
    assert(left_max >= 204 && left_max <= 206);
    assert(right_max >= 408 && right_max <= 410);

    // Vitesse [0 - 100%]
    assert(speed_percent_to_pwm(0.0f) == 0);
    assert(speed_percent_to_pwm(100.0f) == 4095);
    assert(speed_percent_to_pwm(-100.0f) == 4095); // Valeur absolue

    std::cout << "  -> PASS: Calculs PCA9685 (prescale, servos, PWM moteur) validés." << std::endl;
}

void test_line_follower_parsing() {
    std::cout << "[TEST] 3. Vérification du parsing binaire suiveur de ligne 5 voies..." << std::endl;

    using namespace picar::i2c::line_follower;

    // Trame brute simulée de 10 octets reçue par I2C :
    // Sensor 0 : (0x01 << 8) | 0x90 = 400 (fond blanc)
    // Sensor 1 : (0x00 << 8) | 0xC8 = 200 (ligne noire < 300)
    // Sensor 2 : (0x00 << 8) | 0x64 = 100 (ligne noire < 300, centre)
    // Sensor 3 : (0x02 << 8) | 0xBC = 700 (fond blanc)
    // Sensor 4 : (0x03 << 8) | 0xE8 = 1000 (fond blanc)
    const uint8_t raw_buffer[10] = {
        0x01, 0x90, // S0: 400
        0x00, 0xC8, // S1: 200
        0x00, 0x64, // S2: 100
        0x02, 0xBC, // S3: 700
        0x03, 0xE8  // S4: 1000
    };

    uint16_t adc_values[5] = {0};
    parse_raw_adc(raw_buffer, adc_values);

    assert(adc_values[0] == 400);
    assert(adc_values[1] == 200);
    assert(adc_values[2] == 100);
    assert(adc_values[3] == 700);
    assert(adc_values[4] == 1000);

    const uint16_t thresholds[5] = {300, 300, 300, 300, 300};
    uint8_t binary_out[5] = {0};
    binarize_sensors(adc_values, thresholds, binary_out);

    // Attendu : S1 et S2 sont noirs (1), les autres blancs (0)
    assert(binary_out[0] == 0);
    assert(binary_out[1] == 1);
    assert(binary_out[2] == 1);
    assert(binary_out[3] == 0);
    assert(binary_out[4] == 0);

    std::cout << "  -> PASS: Décodage des 10 octets ADC et seuillage validés." << std::endl;
}

void test_hal_driver_instantiation() {
    std::cout << "[TEST] 4. Instanciation des pilotes matériels PiCarHats..." << std::endl;

    // PiCarHatsMotorDriver
    PiCarHatsMotorDriver motor_driver;
    motor_driver.set_steering(15.0f);
    motor_driver.set_speed(30.0f);
    motor_driver.emergency_stop();

    // PiCarHatsSensorDriver
    PiCarHatsSensorDriver sensor_driver;
    uint8_t line_sensors[5] = {0};
    sensor_driver.read_line_sensors(line_sensors);

    // En environnement de test ou sans capteur réel branché, doit retourner un état valide
    for (int i = 0; i < 5; ++i) {
        assert(line_sensors[i] == 0 || line_sensors[i] == 1);
    }

    std::cout << "  -> PASS: Instanciation et exécution sécurisée des pilotes HAL validées." << std::endl;
}

int main() {
    std::cout << "\n==========================================" << std::endl;
    std::cout << "  EXÉCUTION DE LA SUITE DE TESTS I2C (HAL) " << std::endl;
    std::cout << "==========================================" << std::endl;

    test_i2c_addresses_and_constants();
    test_pca9685_math_and_registers();
    test_line_follower_parsing();
    test_hal_driver_instantiation();

    std::cout << "\n>>> TOUS LES TESTS I2C ONT RÉUSSI AVEC SUCCÈS ! <<<\n" << std::endl;
    return 0;
}
