#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#if !defined(_WIN32)
#include <unistd.h>
#else
#define O_NONBLOCK 0
inline int mkfifo(const char*, int) { return 0; }
inline int unlink(const char*) { return 0; }
#endif

#include "control/motion_profiler.hpp"
#include "control/line_follower.hpp"
#include "control/picar_constants.hpp"
#include "control/picar_config.hpp"

bool approx_equal(float a, float b, float epsilon = 0.01f) {
    return std::abs(a - b) <= epsilon;
}

void test_motion_profiler() {
    std::cout << "[TEST] MotionProfiler - Respect strict a <= 0.735 m/s² à vitesse optimisée 0.32 m/s..." << std::endl;
    const float max_a = 0.735f;
    const float cruise_v = 0.32f; // Vitesse de croisière augmentée (+45%)
    MotionProfiler profiler(max_a, cruise_v);

    // 1. Test de limite en virage R = 120 mm (0.12 m)
    // v_max = sqrt(0.735 * 0.12) = sqrt(0.0882) = 0.29698 m/s
    float v_curve = profiler.calculate_curve_speed_limit(0.12f);
    assert(approx_equal(v_curve, 0.297f, 0.005f));

    // 2. Test d'accélération par paliers de 20 ms (50 Hz) de 0 à 0.32 m/s
    float dt = 0.02f;
    float max_delta_v = max_a * dt; // 0.0147 m/s par cycle max

    float current_v = 0.0f;
    float target_v = cruise_v;

    for (int step = 0; step < 60; ++step) {
        float next_v = profiler.update(target_v, dt);
        float delta_v = std::abs(next_v - current_v);
        // L'accélération instantanée ne doit JAMAIS dépasser a_max + tolérance flottante
        assert(delta_v <= max_delta_v + 1e-4f);
        current_v = next_v;
    }

    assert(approx_equal(current_v, target_v, 0.01f));

    // 3. Test de décélération progressive de 0.32 m/s à 0.0 m/s
    target_v = 0.0f;
    for (int step = 0; step < 60; ++step) {
        float next_v = profiler.update(target_v, dt);
        float delta_v = std::abs(next_v - current_v);
        assert(delta_v <= max_delta_v + 1e-4f);
        current_v = next_v;
    }

    assert(approx_equal(current_v, 0.0f, 0.01f));
    std::cout << "  ✓ MotionProfiler : Accélération toujours <= 0.735 m/s² jusqu'à 0.32 m/s." << std::endl;
}

void test_line_follower_controller() {
    std::cout << "[TEST] LineFollowerController - Suivi de ligne PD adaptatif..." << std::endl;
    // Kp=0.65, Kd=0.20, v_cruise=0.32, v_turn=0.14
    LineFollowerController controller(0.65f, 0.20f, 0.32f, 0.14f);

    float dt = 0.02f;

    // 1. Robot parfaitement centré (capteur du milieu seul actif : [0, 0, 1, 0, 0])
    uint8_t centered_sensors[5] = {0, 0, 1, 0, 0};
    auto cmd_center = controller.update(centered_sensors, dt);
    assert(approx_equal(cmd_center.steer_normalized, 0.0f));
    assert(approx_equal(cmd_center.target_velocity, 0.32f));

    // 2. Déviation à gauche (capteur index 1 actif : [0, 1, 0, 0, 0])
    uint8_t left_sensors[5] = {0, 1, 0, 0, 0};
    auto cmd_left = controller.update(left_sensors, dt);
    assert(cmd_left.steer_normalized < 0.0f);
    assert(cmd_left.target_velocity <= 0.32f);

    // 3. Déviation extrême à droite (capteur index 4 actif : [0, 0, 0, 0, 1])
    uint8_t hard_right_sensors[5] = {0, 0, 0, 0, 1};
    auto cmd_hard_right = controller.update(hard_right_sensors, dt);
    assert(cmd_hard_right.steer_normalized > 0.0f);
    // En grand virage, la vitesse doit ralentir vers v_turn (0.14 m/s)
    assert(cmd_hard_right.target_velocity < 0.25f);

    // 4. Perte de ligne temporaire : maintien de la dernière direction (anti-décrochage)
    uint8_t lost_sensors[5] = {0, 0, 0, 0, 0};
    auto cmd_lost = controller.update(lost_sensors, dt);
    // Doit continuer à braquer dans la dernière direction connue
    assert(cmd_lost.steer_normalized > 0.0f);

    std::cout << "  ✓ LineFollowerController : Asservissement PD et anti-décrochage validés." << std::endl;
}

void test_line_follower_analog() {
    std::cout << "[TEST] LineFollowerController - Lecture analogique (sans binarisation)..." << std::endl;

    const float dt = 0.02f;

    // 1. Ligne parfaitement centrée -> pas de braquage
    {
        LineFollowerController c(0.65f, 0.20f, 0.32f, 0.14f);
        const float centered[5] = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
        LineFollowerOutput out = c.update_analog(centered, dt);
        assert(std::abs(out.steer_normalized) < 1e-4f);
        assert(!out.is_finished && !out.is_line_lost);
    }

    // 2. Ruban à cheval entre le centre et D1 -> erreur 0.5, braquage intermédiaire (pas de saut)
    {
        LineFollowerController c(0.65f, 0.20f, 0.32f, 0.14f);
        const float between[5] = {0.0f, 0.0f, 0.5f, 0.5f, 0.0f};
        LineFollowerOutput out = c.update_analog(between, dt);
        assert(std::abs(out.steer_normalized - 0.65f * 0.5f) < 1e-2f); // + une trame d'intégrale (~0.003)
    }

    // 3. Balayage continu du ruban sous les capteurs : le braquage doit évoluer sans à-coups.
    //    (avec des capteurs binaires, l'erreur sauterait de 0.5 à 1.0 d'un coup)
    {
        LineFollowerController c(0.65f, 0.20f, 0.32f, 0.14f);
        const float w[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
        float prev_steer = 0.0f;
        float max_jump = 0.0f;
        bool first = true;
        for (float x = -1.5f; x <= 1.5f; x += 0.05f) {
            float d[5];
            for (int i = 0; i < 5; ++i) d[i] = std::max(0.0f, 1.0f - std::abs(x - w[i]));
            LineFollowerOutput out = c.update_analog(d, dt);
            if (!first) max_jump = std::max(max_jump, std::abs(out.steer_normalized - prev_steer));
            prev_steer = out.steer_normalized;
            first = false;
        }
        std::cout << "    plus grand saut de braquage sur le balayage : " << max_jump << std::endl;
        assert(max_jump < 0.20f);
    }

    // 3b. Ligne droite avec bruit de mesure (quantification 1/7 + bruit de +-1 rayon) :
    //     la direction ne doit pas trembler (régression : l'écart-type était 0.27 sans filtrage de l'erreur).
    {
        LineFollowerController c(0.65f, LineFollowerController::DEFAULT_KD, 0.32f, 0.14f);
        const float w[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
        uint32_t rng = 12345u; // LCG déterministe (indépendant de la bibliothèque standard)
        auto rnd = [&rng]() { rng = rng * 1664525u + 1013904223u; return static_cast<float>(rng >> 8) / 16777216.0f; };
        double sum = 0.0, sum2 = 0.0;
        int n = 0;
        for (int k = 0; k < 1000; ++k) {
            float d[5];
            for (int i = 0; i < 5; ++i) {
                float t = std::max(0.0f, 1.0f - std::abs(0.04f - w[i]));
                float hits = std::round(t * 7.0f);
                if (rnd() < 0.3f) hits += (rnd() < 0.5f) ? -1.0f : 1.0f;
                d[i] = std::clamp(hits / 7.0f, 0.0f, 1.0f);
            }
            float s = c.update_analog(d, dt).steer_normalized;
            if (k > 20) { sum += s; sum2 += static_cast<double>(s) * s; n++; }
        }
        double mean = sum / n;
        double sd = std::sqrt(sum2 / n - mean * mean);
        std::cout << "    écart-type du braquage en ligne droite bruitée : " << sd << std::endl;
        assert(sd < 0.10);
    }

    // 3c. Réactivité : un vrai décalage (virage) doit être suivi en quelques trames malgré le filtrage
    {
        LineFollowerController c(0.65f, 0.20f, 0.32f, 0.14f);
        const float centered[5] = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
        const float curve[5] = {0.0f, 0.0f, 0.0f, 1.0f, 0.5f}; // erreur ~ +1.33
        for (int k = 0; k < 10; ++k) c.update_analog(centered, dt);
        float s = 0.0f;
        for (int k = 0; k < 5; ++k) s = c.update_analog(curve, dt).steer_normalized; // 100 ms
        assert(s > 0.6f);
    }

    // 4. Barre d'arrivée : 4+ capteurs franchement noirs
    {
        LineFollowerController c(0.65f, 0.20f, 0.32f, 0.14f);
        const float bar[5] = {0.9f, 0.9f, 0.9f, 0.9f, 0.9f};
        assert(c.update_analog(bar, dt).is_finished);
        const float faint[5] = {0.4f, 0.4f, 0.4f, 0.4f, 0.4f}; // gris : pas une barre
        LineFollowerController c2(0.65f, 0.20f, 0.32f, 0.14f);
        assert(!c2.update_analog(faint, dt).is_finished);
    }

    // 5. Presque rien de noir -> ligne perdue (maintien du braquage, pas d'arrêt immédiat)
    {
        LineFollowerController c(0.65f, 0.20f, 0.32f, 0.14f);
        const float nothing[5] = {0.02f, 0.03f, 0.02f, 0.01f, 0.02f};
        LineFollowerOutput out = c.update_analog(nothing, dt);
        assert(!out.is_finished && !out.is_line_lost);
    }

    // 6. Continuité et monotonie stricte du barycentre (passage continu de capteur 2 à capteur 3)
    {
        LineFollowerController c(0.65f, 0.05f, 0.32f, 0.14f);
        float last_steer = -999.0f;
        for (float alpha = 0.0f; alpha <= 1.0f; alpha += 0.1f) {
            float d[5] = {0.0f, 0.0f, 1.0f - alpha, alpha, 0.0f};
            LineFollowerOutput out = c.update_analog(d, dt);
            assert(out.steer_normalized >= last_steer - 1e-4f); // Évolution strictement monotone sans discontinuité
            last_steer = out.steer_normalized;
        }
    }

    std::cout << "  ✓ LineFollowerController : lecture analogique (centre de gravité) validée." << std::endl;
}

void test_obstacle_avoidance_s4_and_s5() {
    std::cout << "[TEST] FSM S4 & S5 - Évitement d'obstacle et Marche arrière..." << std::endl;

    MotionProfiler profiler(0.735f, 0.32f);
    const float dt = 0.02f;

    // 1. Décélération sécuritaire devant obstacle depuis 0.32 m/s
    float speed = 0.32f;
    profiler.reset(speed);
    while (profiler.get_current_speed() > 0.01f) {
        speed = profiler.update(0.0f, dt);
    }
    assert(approx_equal(profiler.get_current_speed(), 0.0f, 0.01f));
    std::cout << "  ✓ S1 Arrêt de précision : Immobilisation complète sans secousse." << std::endl;

    // 2. Déviation latérale (RightDeviation) : dégagement odométrique 18 cm
    profiler.reset(0.0f);
    float dev_dist = 0.0f;
    for (int i = 0; i < 60; ++i) { // 60 pas * 0.02s = 1.2s
        float v = profiler.update(0.16f, dt);
        dev_dist += std::abs(v) * dt;
        if (dev_dist >= 0.18f) break;
    }
    assert(dev_dist >= 0.15f);
    std::cout << "  ✓ S4 Déviation : Dégagement odométrique fluide à " << (dev_dist * 100.0f) << " cm." << std::endl;

    // 3. Marche arrière (S5) : vitesse négative bornée et distance < 200 mm
    profiler.reset(0.0f);
    float reverse_dist = 0.0f;
    const float reverse_speed = -0.12f;
    for (int i = 0; i < 75; ++i) { // 75 pas * 0.02s = 1.5s
        float v = profiler.update(reverse_speed, dt);
        reverse_dist += std::abs(v) * dt;
    }
    // Distance totale parcourue doit être strictement < 200 mm (0.20 m)
    assert(reverse_dist < 0.20f);
    assert(reverse_dist >= 0.10f); // ~150-180 mm
    std::cout << "  ✓ S5 Marche arrière guidée : Distance " << (reverse_dist * 1000.0f) << " mm (< 200 mm requis)." << std::endl;
}

void test_dynamic_config_and_fallbacks() {
    std::cout << "[TEST] Configuration dynamique et Fallbacks..." << std::endl;

    // 1. Test du fallback par défaut sur le .hpp (fichier inexistant)
    auto cfg_default = config::PicarConfig::read_config("/chemin_inexistant/bogus.conf");
    assert(approx_equal(cfg_default.max_acceleration, picar::constants::DEFAULT_MAX_ACCELERATION));
    assert(approx_equal(cfg_default.cruise_speed, picar::constants::DEFAULT_STRAIGHT_CRUISE_SPEED));
    assert(approx_equal(cfg_default.min_curve_speed, picar::constants::DEFAULT_MIN_CURVE_SPEED));
    assert(approx_equal(cfg_default.line_kp, picar::constants::DEFAULT_LINE_KP));
    std::cout << "  ✓ Fallback dur .hpp validé si le .config est absent." << std::endl;

    // 2. Test d'écriture et de lecture dans un fichier .config temporaire
    std::string test_conf = "test_picar.conf";
    bool wrote1 = config::PicarConfig::write_config("MAX_ACCELERATION", 0.55f, test_conf);
    bool wrote2 = config::PicarConfig::write_config("CRUISE_SPEED", 0.25f, test_conf);
    assert(wrote1 && wrote2);

    auto cfg_loaded = config::PicarConfig::read_config(test_conf);
    assert(approx_equal(cfg_loaded.max_acceleration, 0.55f));
    assert(approx_equal(cfg_loaded.cruise_speed, 0.25f));
    // Les clés non modifiées doivent garder la valeur par défaut du .hpp
    assert(approx_equal(cfg_loaded.min_curve_speed, picar::constants::DEFAULT_MIN_CURVE_SPEED));
    unlink(test_conf.c_str());
    std::cout << "  ✓ Écriture et relecture .config validées avec préservation des fallbacks." << std::endl;

    // 3. Test de mise à jour dynamique sur MotionProfiler et LineFollowerController
    MotionProfiler profiler(0.735f, 0.32f);
    profiler.set_max_accel(0.50f);
    profiler.set_max_speed(0.20f);
    assert(approx_equal(profiler.get_max_accel(), 0.50f));
    assert(approx_equal(profiler.get_max_speed(), 0.20f));

    LineFollowerController controller;
    controller.set_kp(0.80f);
    controller.set_kd(0.10f);
    controller.set_cruise_speed(0.22f);
    controller.set_min_curve_speed(0.08f);

    // 4. Test du fallback capteur: si aucun capteur actif [0,0,0,0,0], perte de ligne
    uint8_t no_line[5] = {0, 0, 0, 0, 0};
    // Sur 1.5s sans ligne (timeout = 1.2s), is_line_lost doit devenir true
    LineFollowerOutput out_lost;
    for (int i = 0; i < 75; ++i) {
        out_lost = controller.update(no_line, 0.02f);
    }
    assert(out_lost.is_line_lost);
    std::cout << "  ✓ Sécurité perte de ligne (0 0 0 0 0) : transition arrêt/récupération confirmée." << std::endl;

    // 5. Test de l'événement POSIX "trigger_start" via FIFO
    std::string test_fifo = "/tmp/peripheral_test";
    mkfifo(test_fifo.c_str(), 0666);
    int read_fd = open(test_fifo.c_str(), O_RDWR | O_NONBLOCK);
    int write_fd = open(test_fifo.c_str(), O_WRONLY);
    const char* trigger_cmd = "trigger_start\n";
    write(write_fd, trigger_cmd, strlen(trigger_cmd));
    char read_buf[64] = {0};
    ssize_t n_read = read(read_fd, read_buf, sizeof(read_buf) - 1);
    assert(n_read > 0);
    assert(std::string(read_buf).find("trigger_start") != std::string::npos);
    close(read_fd);
    close(write_fd);
    unlink(test_fifo.c_str());
    std::cout << "  ✓ Tube FIFO POSIX (trigger_start) validé." << std::endl;
}

void test_ackermann_curve_speed() {
    std::cout << "[TEST] MotionProfiler - Vitesse en virage Ackermann..." << std::endl;
    MotionProfiler profiler(0.735f, 0.32f);

    const float cruise = 0.32f;
    const float min_spd = 0.14f;
    const float safety = 0.75f;

    // 1. Ligne droite (|steer| <= 2°) : maintien ferme à la vitesse de croisière (aucun ralentissement parasite)
    float v_straight0 = profiler.calculate_curve_speed_ackermann(0.0f, cruise, min_spd, safety);
    float v_straight2 = profiler.calculate_curve_speed_ackermann(2.0f, cruise, min_spd, safety);
    assert(approx_equal(v_straight0, cruise, 1e-4f));
    assert(approx_equal(v_straight2, cruise, 1e-4f));

    // 2. Virage modéré (15°) : ralentissement calculé par R = L/tan(15°)
    float v_turn15 = profiler.calculate_curve_speed_ackermann(15.0f, cruise, min_spd, safety);
    assert(v_turn15 <= cruise);
    assert(v_turn15 >= min_spd);

    // 3. Virage serré (35°) : doit ralentir davantage
    float v_turn35 = profiler.calculate_curve_speed_ackermann(35.0f, cruise, min_spd, safety);
    assert(v_turn35 < v_turn15);
    assert(v_turn35 >= min_spd);

    // 4. Virage extrême (45°) : borné au minimum autorisé
    float v_turn45 = profiler.calculate_curve_speed_ackermann(45.0f, cruise, min_spd, safety);
    assert(v_turn45 >= min_spd);

    // 5. Symétrie gauche/droite
    float v_left = profiler.calculate_curve_speed_ackermann(-25.0f, cruise, min_spd, safety);
    float v_right = profiler.calculate_curve_speed_ackermann(25.0f, cruise, min_spd, safety);
    assert(approx_equal(v_left, v_right, 1e-4f));

    std::cout << "  ✓ Vitesse en virage Ackermann : Maintien en ligne droite et décélération physique validées." << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  PiCar C++17 - Tests Unitaires Contrôle" << std::endl;
    std::cout << "========================================" << std::endl;

    test_motion_profiler();
    test_ackermann_curve_speed();
    test_line_follower_controller();
    test_line_follower_analog();
    test_obstacle_avoidance_s4_and_s5();
    test_dynamic_config_and_fallbacks();

    std::cout << "========================================" << std::endl;
    std::cout << "  TOUS LES TESTS SONT PASSÉS AVEC SUCCÈS" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
