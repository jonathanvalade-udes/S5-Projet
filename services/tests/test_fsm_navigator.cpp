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

    std::cout << "test_fsm_navigator: tous les tests passent" << std::endl;
    return 0;
}
