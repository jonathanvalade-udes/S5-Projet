// Tests du ralentissement a l'approche d'un obstacle (FsmNavigator::limit_approach_speed).
#include <cassert>
#include <cmath>
#include <iostream>

#include "FsmNavigator.hpp"

using services::FsmNavigator;

static bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) < eps; }

int main() {
    FsmNavigator nav;
    // kp, kd, croisiere (m/s), vitesse mini en courbe, acceleration max (m/s^2)
    nav.init(0.75f, 0.2f, 0.45f, 0.22f, 0.735f);

    const float stop_cm = 20.0f;
    const float cruise = 0.45f;

    // Loin de l'obstacle (>= 45 cm) : la consigne n'est pas modifiee
    assert(near(nav.limit_approach_speed(cruise, 100.0f, stop_cm), cruise));
    assert(near(nav.limit_approach_speed(cruise, 45.0f, stop_cm), cruise));

    // Dans la zone de ralentissement : v <= sqrt(2 * a * d)
    // (consigne 1,0 m/s : superieure a la limite, c'est donc la limite qui s'applique)
    const float v35 = nav.limit_approach_speed(1.0f, 35.0f, stop_cm);
    assert(near(v35, std::sqrt(2.0f * 0.735f * 0.15f)));

    // Plus on est pres, plus la vitesse autorisee diminue
    const float v25 = nav.limit_approach_speed(1.0f, 25.0f, stop_cm);
    assert(v25 < v35);
    assert(near(v25, std::sqrt(2.0f * 0.735f * 0.05f)));

    // A 25 cm, la croisiere (0,45 m/s) depasse la limite (~0,27 m/s) : elle est bien reduite
    assert(nav.limit_approach_speed(cruise, 25.0f, stop_cm) < cruise);

    // A la distance d'arret (ou en dessous) : vitesse nulle
    assert(near(nav.limit_approach_speed(cruise, 20.0f, stop_cm), 0.0f));
    assert(near(nav.limit_approach_speed(cruise, 10.0f, stop_cm), 0.0f));

    // Une consigne deja plus lente que la limite n'est jamais augmentee
    assert(near(nav.limit_approach_speed(0.05f, 35.0f, stop_cm), 0.05f));

    // --- Evitement : diagonale de la phase 2 reglable (AVOID_DIAG_TIME_S / AVOID_DIAG_SPEED) ---
    {
        using services::ObstacleAvoidance;
        const float dark[5] = {0, 0, 0, 0, 0};
        // Premier pas (dt = 20 ms) ou la manoeuvre atteint la phase 3, et vitesse commandee a mi-diagonale
        auto step_phase3 = [&](float diag_time, float diag_speed, float& speed_mid_diag) {
            ObstacleAvoidance av;
            av.set_diagonal(diag_time, diag_speed);
            av.reset(0.0f, 0.2f);
            float steer = 0.0f, vel = 0.0f, v = 0.0f;
            speed_mid_diag = 0.0f;
            int phase2_start = -1;
            for (int k = 0; k < 2000; ++k) {
                v += (vel * 0.9f - v) * (0.02f / 0.1f);
                av.update(1.0f, 0.02f, dark, v, 0.2f, steer, vel);
                if (av.get_phase() == 2 && phase2_start < 0) phase2_start = k;
                if (av.get_phase() == 2 && k == phase2_start + 70) speed_mid_diag = vel;   // 1,4 s apres le debut de la phase 2 : en pleine diagonale
                if (av.get_phase() == 3) return k;
            }
            return -1;
        };
        float sp = 0.0f;
        const int k_default = step_phase3(ObstacleAvoidance::DEFAULT_DIAG_TIME_S, ObstacleAvoidance::DEFAULT_DIAG_SPEED, sp);
        assert(k_default > 0);
        assert(near(sp, ObstacleAvoidance::DEFAULT_DIAG_SPEED));

        // Une diagonale plus longue de 0,9 s retarde la fin de la phase 2 de 45 pas de 20 ms
        const int k_long = step_phase3(2.0f, 0.16f, sp);
        assert(k_long - k_default >= 44 && k_long - k_default <= 46);

        // La vitesse de la diagonale est bien celle demandee
        step_phase3(1.5f, 0.22f, sp);
        assert(near(sp, 0.22f));

        // Valeurs hors bornes : ramenees a 0,3..3 s et 0,12..0,25 m/s
        const int k_min = step_phase3(0.0f, 0.16f, sp);       // 0 s -> 0,3 s
        assert(k_default - k_min >= 39 && k_default - k_min <= 41);   // 0,8 s de moins = 40 pas
        step_phase3(1.5f, 9.0f, sp);
        assert(near(sp, 0.25f));
        step_phase3(1.5f, 0.01f, sp);
        assert(near(sp, 0.12f));
    }

    // --- Evitement : deuxieme diagonale (phase 5) reglable (AVOID_RETURN_TIME_S / AVOID_RETURN_SPEED) ---
    {
        using services::ObstacleAvoidance;
        const float dark[5] = {0, 0, 0, 0, 0};   // aucun ruban : la phase 5 va jusqu'a sa duree maximale
        // Pas ou la manoeuvre abandonne (SearchLine), et vitesse commandee a 1 s du debut de la phase 5
        auto step_giveup = [&](float ret_time, float ret_speed, float& speed_in_phase5) {
            ObstacleAvoidance av;
            av.set_return(ret_time, ret_speed);
            av.reset(0.0f, 0.2f);
            float steer = 0.0f, vel = 0.0f, v = 0.0f;
            speed_in_phase5 = 0.0f;
            int phase5_start = -1;
            for (int k = 0; k < 2000; ++k) {
                v += (vel * 0.9f - v) * (0.02f / 0.1f);
                const auto r = av.update(1.0f, 0.02f, dark, v, 0.2f, steer, vel);
                if (av.get_phase() == 5 && phase5_start < 0) phase5_start = k;
                if (phase5_start >= 0 && k == phase5_start + 50) speed_in_phase5 = vel;
                if (r == ObstacleAvoidance::DeviationResult::SearchLine) return k;
            }
            return -1;
        };
        float sp = 0.0f;
        const int k_default = step_giveup(ObstacleAvoidance::DEFAULT_RETURN_TIME_S, ObstacleAvoidance::DEFAULT_RETURN_SPEED, sp);
        assert(k_default > 0);
        assert(near(sp, ObstacleAvoidance::DEFAULT_RETURN_SPEED));

        // Une recherche plus courte de 1,5 s abandonne 75 pas de 20 ms plus tot
        const int k_short = step_giveup(2.0f, 0.16f, sp);
        assert(k_default - k_short >= 74 && k_default - k_short <= 76);

        // La vitesse de la deuxieme diagonale est bien celle demandee
        step_giveup(3.0f, 0.22f, sp);
        assert(near(sp, 0.22f));

        // Hors bornes : ramenees a 1..5 s et 0,12..0,25 m/s
        const int k_min = step_giveup(0.0f, 0.16f, sp);           // 0 s -> 1 s : 2,5 s de moins que 3,5 s = 125 pas
        assert(k_default - k_min >= 124 && k_default - k_min <= 126);
        step_giveup(3.0f, 9.0f, sp);
        assert(near(sp, 0.25f));
        step_giveup(3.0f, 0.01f, sp);
        assert(near(sp, 0.12f));
    }

    // --- Puissance moteur : vitesse / croisiere x puissance a la croisiere (MAX_POWER_PCT) ---
    {
        assert(near(nav.compute_motor_power(0.32f, 0.32f), 50.0f));            // defaut : 50 % a la croisiere
        assert(near(nav.compute_motor_power(0.16f, 0.32f), 25.0f));            // la manoeuvre d'evitement : 25 %
        assert(near(nav.compute_motor_power(0.32f, 0.32f, 60.0f), 60.0f));     // MAX_POWER_PCT = 60
        assert(near(nav.compute_motor_power(0.16f, 0.32f, 60.0f), 30.0f));
        assert(near(nav.compute_motor_power(-0.16f, 0.32f, 60.0f), -30.0f));   // marche arriere
        assert(near(nav.compute_motor_power(0.64f, 0.32f, 75.0f), 100.0f));    // jamais plus de 100 %
        assert(near(nav.compute_motor_power(0.30f, 0.0f), 0.0f));              // croisiere nulle : pas de mouvement
        assert(FsmNavigator::MIN_POWER_PCT < FsmNavigator::REFERENCE_POWER_PCT);
        assert(FsmNavigator::REFERENCE_POWER_PCT < FsmNavigator::MAX_POWER_PCT);
        assert(FsmNavigator::MIN_CRUISE_SPEED > 0.0f);
    }

    std::cout << "test_fsm_navigator: tous les tests passent" << std::endl;
    return 0;
}
