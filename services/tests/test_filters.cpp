#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

#include "filters/median_filter.hpp"
#include "filters/low_pass_ema.hpp"
#include "filters/ultrasonic_pipeline.hpp"

// Fonction utilitaire pour tester la proximité des flottants
bool approx_equal(float a, float b, float epsilon = 0.05f) {
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

    // Simulation de plusieurs itérations pour vérifier la convergence asymptotique vers 20.0f
    for (int i = 0; i < 30; ++i) {
        ema.filter(20.0f);
    }
    assert(approx_equal(ema.get_value(), 20.0f, 0.05f));

    std::cout << "  ✓ Filtre EMA : Lissage exponentiel et convergence validés." << std::endl;
}

void test_ultrasonic_pipeline() {
    std::cout << "[TEST] UltrasonicPipeline - Duo Médian + Passe-Bas EMA..." << std::endl;
    UltrasonicPipeline pipeline(5, 0.25f);
    pipeline.reset(100.0f);

    // Vérification de la distance initiale
    assert(approx_equal(pipeline.get_distance(), 100.0f));

    // Séquence de mesures avec bruit et glitches (0cm et 400cm)
    std::vector<float> simulated_readings = {
        100.0f, 99.0f, 101.0f, 0.0f /* Glitch */, 98.5f, 400.0f /* Glitch */,
        97.0f, 96.5f, 96.0f, 95.0f
    };

    float result = 100.0f;
    for (float r : simulated_readings) {
        result = pipeline.process(r, 0.02f, 10.0f);
        // Le résultat ne doit jamais être corrompu par les glitches 0 cm ou 400 cm
        assert(result >= 90.0f && result <= 105.0f);
    }

    // Test de rejet immédiat d'un échelon violent avec glitch
    pipeline.reset(50.0f);
    pipeline.process(50.0f);
    pipeline.process(50.0f);
    pipeline.process(50.0f);
    float val_glitch = pipeline.process(400.0f); // Glitch direct
    // Le filtre médian rejette le 400.0f, donc le résultat reste proche de 50.0f
    assert(approx_equal(val_glitch, 50.0f, 1.0f));

    std::cout << "  ✓ Pipeline HC-SR04 (Médian + EMA) : Rejet des glitches et lissage validés." << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  PiCar C++17 - Tests Unitaires Filtres " << std::endl;
    std::cout << "========================================" << std::endl;

    test_median_filter();
    test_low_pass_ema();
    test_ultrasonic_pipeline();

    std::cout << "========================================" << std::endl;
    std::cout << "  TOUS LES TESTS SONT PASSÉS AVEC SUCCÈS" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
