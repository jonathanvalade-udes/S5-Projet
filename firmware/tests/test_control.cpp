#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

#include "../include/control/motion_profiler.hpp"
#include "../include/control/line_follower.hpp"

bool approx_equal(float a, float b, float epsilon = 0.01f) {
    return std::abs(a - b) <= epsilon;
}

void test_motion_profiler() {
    std::cout << "[TEST] MotionProfiler - Respect strict a <= 0.735 m/s²..." << std::endl;
    MotionProfiler profiler(0.735f, 0.25f);

    // 1. Test de limite en virage R = 120 mm (0.12 m)
    // v_max = sqrt(0.735 * 0.12) = sqrt(0.0882) = 0.29698 m/s
    float v_curve = profiler.calculate_curve_speed_limit(0.12f);
    assert(approx_equal(v_curve, 0.297f, 0.005f));
    std::cout << "  ✓ Vitesse limite virage R=120mm : " << v_curve << " m/s (~0.297 m/s)." << std::endl;

    // 2. Test d'accélération en rampe sur échelon brutal (0.0 -> 1.0 m/s)
    const float dt = 0.02f; // 50 Hz
    const float max_allowed_delta = 0.735f * dt + 1e-5f; // 0.0147 m/s max par pas

    float current = 0.0f;
    for (int i = 0; i < 50; ++i) {
        float next = profiler.update(1.0f, dt);
        float delta = next - current;
        assert(delta <= max_allowed_delta);
        current = next;
    }
    // La vitesse ne doit jamais dépasser max_speed (0.25 m/s)
    assert(profiler.get_current_speed() <= 0.25f);
    std::cout << "  ✓ Rampe d'accélération : Delta vitesse strictly borné à " << max_allowed_delta << " m/s par cycle." << std::endl;

    // 3. Test de freinage d'urgence progressif (décélération)
    for (int i = 0; i < 50; ++i) {
        float next = profiler.update(0.0f, dt);
        float delta = current - next;
        assert(delta <= max_allowed_delta);
        current = next;
    }
    assert(approx_equal(profiler.get_current_speed(), 0.0f, 0.001f));
    std::cout << "  ✓ Décélération maximale bornée : Bille protégée contre le renversement." << std::endl;
}

void test_line_follower() {
    std::cout << "[TEST] LineFollowerController - Barycentre et détection T-Stop..." << std::endl;
    LineFollowerController controller;

    const float dt = 0.02f;

    // 1. Centré sur la ligne [0, 0, 1, 0, 0]
    uint8_t centered[5] = {0, 0, 1, 0, 0};
    auto out_center = controller.update(centered, dt);
    assert(approx_equal(out_center.steer_normalized, 0.0f, 0.05f));
    assert(approx_equal(out_center.steer_degrees, 0.0f, 1.0f));
    assert(!out_center.is_finished);
    std::cout << "  ✓ Suivi centré : Direction neutre (0.0°)." << std::endl;

    // 2. Ligne à droite [0, 0, 0, 1, 0]
    uint8_t right[5] = {0, 0, 0, 1, 0};
    auto out_right = controller.update(right, dt);
    assert(out_right.steer_normalized > 0.0f);
    assert(out_right.steer_degrees > 0.0f);
    std::cout << "  ✓ Ligne déportée droite : Braquage positif (" << out_right.steer_degrees << "°)." << std::endl;

    // 3. Ligne à gauche [0, 1, 0, 0, 0]
    uint8_t left[5] = {0, 1, 0, 0, 0};
    auto out_left = controller.update(left, dt);
    assert(out_left.steer_normalized < 0.0f);
    assert(out_left.steer_degrees < 0.0f);
    std::cout << "  ✓ Ligne déportée gauche : Braquage négatif (" << out_left.steer_degrees << "°)." << std::endl;

    // 4. Barre d'arrivée en T [1, 1, 1, 1, 1]
    uint8_t t_stop[5] = {1, 1, 1, 1, 1};
    auto out_t = controller.update(t_stop, dt);
    assert(out_t.is_finished);
    assert(approx_equal(out_t.target_velocity, 0.0f));
    std::cout << "  ✓ Détection barre d'arrivée T-Stop : Arrêt franc confirmé." << std::endl;

    // 5. Perte de ligne temporaire vs prolongée
    uint8_t lost[5] = {0, 0, 0, 0, 0};
    controller.reset();
    controller.update(right, dt); // initialise un virage à droite
    auto out_lost1 = controller.update(lost, 0.1f);
    assert(!out_lost1.is_line_lost); // Perte temporaire (< 1.2s), maintien
    auto out_lost2 = controller.update(lost, 1.5f); // Dépassement timeout 1.2s
    assert(out_lost2.is_line_lost);
    std::cout << "  ✓ Gestion perte de ligne : Maintien temporaire puis sécurité Failsafe validés." << std::endl;
}

int main() {
    std::cout << "===============================================" << std::endl;
    std::cout << "   PiCar C++17 - Tests Unitaires Contrôle      " << std::endl;
    std::cout << "===============================================" << std::endl;

    test_motion_profiler();
    test_line_follower();

    std::cout << "===============================================" << std::endl;
    std::cout << "  TOUS LES TESTS DE CONTRÔLE ONT RÉUSSI !      " << std::endl;
    std::cout << "===============================================" << std::endl;
    return 0;
}
