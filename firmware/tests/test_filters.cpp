#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

#include "../include/filters/median_filter.hpp"
#include "../include/filters/low_pass_ema.hpp"
#include "../include/filters/kalman_1d.hpp"
#include "../include/filters/ultrasonic_pipeline.hpp"

// Fonction utilitaire pour tester la proximité des flottants
bool approx_equal(float a, float b, float epsilon = 0.01f) {
    return std::abs(a - b) <= epsilon;
}

void test_median_filter() {
    std::cout << "[TEST] MedianFilter - Rejet des glitches..." << std::endl;
    MedianFilter filter(5);

    // Remplissage avec une valeur constante (30.0 cm)
    filter.update(30.0f);
    filter.update(30.0f);
    filter.update(30.0f);
    filter.update(30.0f);
    float val = filter.update(30.0f);
    assert(approx_equal(val, 30.0f));

    // Injection d'un pic aberrant (glitch HC-SR04 typique: 400 cm ou 0 cm)
    float spike_val1 = filter.update(400.0f);
    // Le médian doit ignorer complètement le 400 cm car 4 échantillons sur 5 sont à 30.0 cm
    assert(approx_equal(spike_val1, 30.0f));

    float spike_val2 = filter.update(0.0f);
    // Fenêtre: [30, 30, 400, 0, 30] -> triée: [0, 30, 30, 30, 400] -> Médian = 30.0 cm
    assert(approx_equal(spike_val2, 30.0f));

    std::cout << "  ✓ Médian glissant : Glitches (0cm et 400cm) parfaitement rejetés." << std::endl;
}

void test_low_pass_ema() {
    std::cout << "[TEST] LowPassEMA - Lissage sans dépassement..." << std::endl;
    LowPassEMA ema(0.25f);

    // Initialisation au premier échantillon
    float first = ema.filter(10.0f);
    assert(approx_equal(first, 10.0f));

    // Échelon vers 20.0f
    float step1 = ema.filter(20.0f); // 0.25 * 20 + 0.75 * 10 = 5 + 7.5 = 12.5
    assert(approx_equal(step1, 12.5f));

    float step2 = ema.filter(20.0f); // 0.25 * 20 + 0.75 * 12.5 = 5 + 9.375 = 14.375
    assert(approx_equal(step2, 14.375f));

    // Simulation de plusieurs itérations pour vérifier la convergence asymptotique
    for (int i = 0; i < 30; ++i) {
        ema.filter(20.0f);
    }
    assert(approx_equal(ema.get_value(), 20.0f, 0.05f));

    std::cout << "  ✓ Filtre EMA : Lissage exponentiel et convergence validés." << std::endl;
}

void test_kalman_1d() {
    std::cout << "[TEST] KalmanFilter1D - Fusion & Rejet d'accélération..." << std::endl;
    KalmanFilter1D kf(0.1f, 2.0f, 73.5f);

    // Initialisation
    float est = kf.update(50.0f, 0.02f);
    assert(approx_equal(est, 50.0f));

    // Évolution régulière : le robot s'approche d'un obstacle
    // (50cm, 49.5cm, 49.0cm avec un pas de temps dt = 0.02s => vitesse 25 cm/s)
    float current_dist = 50.0f;
    for (int i = 0; i < 20; ++i) {
        current_dist -= 0.5f;
        kf.predict(0.02f, -25.0f); // Modèle moteur prédisant l'approche
        est = kf.update(current_dist, 0.02f);
    }

    assert(approx_equal(est, current_dist, 1.0f));

    // Test de rejet d'une saut d'accélération impossible (mesure délirante subite)
    float crazy_measurement = 10.0f; // saut de 30 cm en 20 ms -> accélération irréaliste
    float filtered_crazy = kf.update(crazy_measurement, 0.02f);
    // L'estimation ne doit PAS sauter à 10.0f immédiatement
    assert(filtered_crazy > 30.0f);

    std::cout << "  ✓ Kalman 1D : Filtrage et amortissement des anomalies cinématiques validés." << std::endl;
}

void test_ultrasonic_pipeline() {
    std::cout << "[TEST] UltrasonicPipeline - Pipeline complet HC-SR04..." << std::endl;
    UltrasonicPipeline pipeline(5, 0.25f);
    pipeline.reset(100.0f);

    // Séquence de mesures avec bruit et glitches
    std::vector<float> simulated_readings = {
        100.0f, 99.0f, 101.0f, 0.0f /* Glitch */, 98.5f, 400.0f /* Glitch */,
        97.0f, 96.5f, 96.0f, 95.0f
    };

    float result = 100.0f;
    for (float r : simulated_readings) {
        result = pipeline.process(r, 0.02f, 10.0f); // robot avance à 10 cm/s
        // Le résultat doit toujours rester dans une plage raisonnable [90, 105] cm
        assert(result >= 90.0f && result <= 105.0f);
    }

    std::cout << "  ✓ Pipeline HC-SR04 : Résilience totale aux pannes et signaux parasites." << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  PiCar C++17 - Tests Unitaires Filtres " << std::endl;
    std::cout << "========================================" << std::endl;

    test_median_filter();
    test_low_pass_ema();
    test_kalman_1d();
    test_ultrasonic_pipeline();

    std::cout << "========================================" << std::endl;
    std::cout << "  TOUS LES TESTS SONT PASSÉS AVEC SUCCÈS" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
