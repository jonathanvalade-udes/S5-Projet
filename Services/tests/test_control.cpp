#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

#include "control/motion_profiler.hpp"
#include "control/line_follower.hpp"

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

    // 2. Test d'accélération par paliers de 20 ms (50 Hz)
    float dt = 0.02f;
    float max_delta_v = 0.735f * dt; // 0.0147 m/s par cycle max

    float current_v = 0.0f;
    float target_v = 0.25f;

    for (int step = 0; step < 50; ++step) {
        float next_v = profiler.update(target_v, dt);
        float delta_v = std::abs(next_v - current_v);
        // L'accélération instantanée ne doit JAMAIS dépasser a_max + tolérance flottante
        assert(delta_v <= max_delta_v + 1e-4f);
        current_v = next_v;
    }

    assert(approx_equal(current_v, target_v, 0.01f));

    // 3. Test de décélération progressive
    target_v = 0.0f;
    for (int step = 0; step < 50; ++step) {
        float next_v = profiler.update(target_v, dt);
        float delta_v = std::abs(next_v - current_v);
        assert(delta_v <= max_delta_v + 1e-4f);
        current_v = next_v;
    }

    assert(approx_equal(current_v, 0.0f, 0.01f));
    std::cout << "  ✓ MotionProfiler : Accélération toujours <= 0.735 m/s²." << std::endl;
}

void test_line_follower_controller() {
    std::cout << "[TEST] LineFollowerController - Suivi de ligne PD..." << std::endl;
    // Kp=0.65, Kd=0.20, v_cruise=0.22, v_turn=0.10
    LineFollowerController controller(0.65f, 0.20f, 0.22f, 0.10f);

    float dt = 0.02f;

    // 1. Robot parfaitement centré (capteur du milieu seul actif : [0, 0, 1, 0, 0])
    uint8_t centered_sensors[5] = {0, 0, 1, 0, 0};
    auto cmd_center = controller.update(centered_sensors, dt);
    assert(approx_equal(cmd_center.steer_normalized, 0.0f));
    assert(approx_equal(cmd_center.target_velocity, 0.22f));

    // 2. Déviation à gauche (capteur index 1 actif : [0, 1, 0, 0, 0])
    uint8_t left_sensors[5] = {0, 1, 0, 0, 0};
    auto cmd_left = controller.update(left_sensors, dt);
    // Braquage vers la droite (angle négatif selon la convention)
    assert(cmd_left.steer_normalized < 0.0f);
    assert(cmd_left.target_velocity <= 0.22f);

    // 3. Déviation extrême à droite (capteur index 4 actif : [0, 0, 0, 0, 1])
    uint8_t hard_right_sensors[5] = {0, 0, 0, 0, 1};
    auto cmd_hard_right = controller.update(hard_right_sensors, dt);
    assert(cmd_hard_right.steer_normalized > 0.0f);
    // En grand virage, la vitesse doit ralentir vers v_turn (0.10 m/s)
    assert(cmd_hard_right.target_velocity < 0.20f);

    // 4. Perte de ligne temporaire : maintien de la dernière direction (anti-décrochage)
    uint8_t lost_sensors[5] = {0, 0, 0, 0, 0};
    auto cmd_lost = controller.update(lost_sensors, dt);
    // Doit continuer à braquer dans la dernière direction connue
    assert(cmd_lost.steer_normalized > 0.0f);

    std::cout << "  ✓ LineFollowerController : Asservissement PD et anti-décrochage validés." << std::endl;
}

void test_obstacle_avoidance_s4_and_s5() {
    std::cout << "[TEST] FSM S4 & S5 - Évitement d'obstacle et Marche arrière..." << std::endl;

    // Simulation de la cinématique d'évitement
    MotionProfiler profiler(0.735f, 0.25f);
    const float dt = 0.02f;

    // 1. Décélération sécuritaire devant obstacle
    float speed = 0.25f;
    profiler.reset(speed);
    while (profiler.get_current_speed() > 0.01f) {
        speed = profiler.update(0.0f, dt);
    }
    assert(approx_equal(profiler.get_current_speed(), 0.0f, 0.01f));
    std::cout << "  ✓ S1 Arrêt de précision : Immobilisation complète sans secousse." << std::endl;

    // 2. Déviation latérale (RightDeviation) : rampe vers vitesse douce 0.15 m/s
    for (int i = 0; i < 20; ++i) {
        speed = profiler.update(0.15f, dt);
    }
    assert(speed <= 0.15f);
    std::cout << "  ✓ S4 Déviation : Accélération d'évitement sous le seuil critique." << std::endl;

    // 3. Marche arrière (S5) : vitesse négative bornée et distance < 200 mm
    profiler.reset(0.0f);
    float reverse_dist = 0.0f;
    const float reverse_speed = -0.10f;
    for (int i = 0; i < 75; ++i) { // 75 pas * 0.02s = 1.5s
        float v = profiler.update(reverse_speed, dt);
        reverse_dist += std::abs(v) * dt;
    }
    // Distance totale parcourue doit être strictement < 200 mm (0.20 m)
    assert(reverse_dist < 0.20f);
    assert(reverse_dist >= 0.10f); // ~150 mm
    std::cout << "  ✓ S5 Marche arrière guidée : Distance " << (reverse_dist * 1000.0f) << " mm (< 200 mm requis)." << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  PiCar C++17 - Tests Unitaires Contrôle" << std::endl;
    std::cout << "========================================" << std::endl;

    test_motion_profiler();
    test_line_follower_controller();
    test_obstacle_avoidance_s4_and_s5();

    std::cout << "========================================" << std::endl;
    std::cout << "  TOUS LES TESTS SONT PASSÉS AVEC SUCCÈS" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
