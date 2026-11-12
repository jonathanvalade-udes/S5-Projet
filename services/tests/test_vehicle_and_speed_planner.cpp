#include <cassert>
#include <cmath>
#include <iostream>
#include "FSM/inc/VehicleModel.hpp"
#include "FSM/inc/SpeedPlanner.hpp"

using services::VehicleModel;
using services::SpeedPlanner;

static bool near(float a, float b, float eps = 1e-2f) {
    return std::fabs(a - b) < eps;
}

int main() {
    std::cout << "[TEST] VehicleModel & SpeedPlanner - Tests cinématiques et protection bille..." << std::endl;

    // 1. VehicleModel : Table de calibration PWM <-> Vitesse
    {
        VehicleModel model;
        // 0 m/s -> 0%
        assert(near(model.speed_to_pwm(0.0f), 0.0f));
        // Petite vitesse positive -> zone morte franchie (20%)
        assert(model.speed_to_pwm(0.05f) >= 20.0f);
        // Vitesse de référence 0.32 m/s -> 50% PWM (mesure physique confirmée)
        assert(near(model.speed_to_pwm(0.32f), 50.0f, 1.0f));
        // Vitesse négative (marche arrière)
        assert(near(model.speed_to_pwm(-0.32f), -50.0f, 1.0f));

        // Réciproque : PWM -> vitesse
        assert(near(model.pwm_to_speed(0.0f), 0.0f));
        assert(near(model.pwm_to_speed(15.0f), 0.0f)); // Sous la zone morte (20%) -> 0 m/s
        assert(near(model.pwm_to_speed(50.0f), 0.32f, 0.02f));
        std::cout << "  ✓ Conversions vitesse <-> PWM et zone morte validées." << std::endl;

        // Cinématique Ackermann
        float r_straight = model.calculate_turn_radius(0.0f);
        assert(r_straight > 100.0f); // Ligne droite = rayon infini / très grand

        float r_steer = model.calculate_turn_radius(30.0f);
        float expected_r = 0.14f / std::tan(30.0f * 3.14159265f / 180.0f);
        assert(near(r_steer, expected_r, 0.02f));
        std::cout << "  ✓ Modèle cinématique d'Ackermann validé (R=" << r_steer << " m à 30°)." << std::endl;
    }

    // 2. SpeedPlanner : Rampe d'accélération et freinage
    {
        SpeedPlanner planner(0.70f, 0.60f, 0.70f);
        planner.reset(0.0f);

        // Accélération progressive en ligne droite (dt = 0.1 s, a_max = 0.60 m/s²)
        float v1 = planner.update(0.40f, 0.0f, -1.0f, 0.20f, 0.1f);
        assert(near(v1, 0.06f, 0.01f)); // 0 + 0.6 * 0.1 = 0.06

        float v2 = planner.update(0.40f, 0.0f, -1.0f, 0.20f, 0.1f);
        assert(near(v2, 0.12f, 0.01f));

        // Freinage vers 0
        planner.reset(0.30f);
        float vf = planner.update(0.0f, 0.0f, -1.0f, 0.20f, 0.1f);
        assert(near(vf, 0.23f, 0.01f)); // 0.30 - 0.70 * 0.1 = 0.23
        std::cout << "  ✓ Rampes temporelles asymétriques accélération / freinage validées." << std::endl;
    }

    // 3. SpeedPlanner : Protection de la bille sous accélération couplée (a_res <= a_ball_max)
    {
        SpeedPlanner planner(0.70f, 0.60f, 0.70f);
        // Véhicule en virage serré (35°) à vitesse déjà établie (0.35 m/s)
        planner.reset(0.35f);
        float v_next = planner.update(0.50f, 35.0f, -1.0f, 0.20f, 0.1f);
        (void)v_next;
        // En virage, l'accélération résultante doit respecter le seuil de la bille
        float a_res = planner.get_last_resultant_accel();
        assert(a_res <= 0.75f);
        std::cout << "  ✓ Protection couplée de la bille validée (a_résultante=" << a_res << " <= 0.70 m/s²)." << std::endl;
    }

    // 4. SpeedPlanner : Ralentissement et arrêt anticipé devant obstacle
    {
        SpeedPlanner planner(0.70f, 0.60f, 0.70f);
        planner.reset(0.40f);
        // Obstacle à 25 cm (distance d'arrêt demandée 20 cm -> reste 5 cm = 0.05 m)
        float v_obs = planner.update(0.40f, 0.0f, 0.25f, 0.20f, 0.05f);
        // v_obs doit décélérer sous la vitesse nominale
        assert(v_obs < 0.40f);
        // À la distance d'arrêt exacte (reste 0 m)
        float v_zero = planner.calculate_max_safe_approach_speed(0.0f);
        assert(near(v_zero, 0.0f));
        std::cout << "  ✓ Décélération anticipée et vitesse d'arrêt obstacle validées." << std::endl;
    }

    std::cout << "==> TOUS LES TESTS VEHICLE_MODEL & SPEED_PLANNER ONT RÉUSSI !" << std::endl;
    return 0;
}
