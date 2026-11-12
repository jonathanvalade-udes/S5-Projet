#include <cassert>
#include <cmath>
#include <iostream>
#include "Motors/inc/MotorActuator.hpp"

using services::MotorActuator;

static bool near(float a, float b, float eps = 1e-2f) {
    return std::fabs(a - b) < eps;
}

int main() {
    std::cout << "[TEST] MotorActuator - Tests unitaires de sécurité et d'actionnement..." << std::endl;

    MotorActuator actuator(false, 0.0f);
    // Initialisation en mode non-requis (mock/fallback sans I2C physique)
    bool ok = actuator.init(false);
    assert(ok);
    std::cout << "  ✓ Initialisation mock/fallback validée." << std::endl;

    // 1. Limitation des consignes (saturation)
    {
        actuator.update_actuation(150.0f, 60.0f, 0.02f, false);
        assert(near(actuator.get_applied_speed(), 100.0f));
        // En mode sans hardware, applied_steering est borné à 45°
        assert(actuator.get_applied_steering() <= 45.0f);
        std::cout << "  ✓ Plafonnement des consignes (+150% -> 100%, +60° -> <= 45°) validé." << std::endl;

        actuator.update_actuation(-120.0f, -90.0f, 0.02f, false);
        assert(near(actuator.get_applied_speed(), -100.0f));
        assert(actuator.get_applied_steering() >= -45.0f);
        std::cout << "  ✓ Plafonnement des consignes négatives validé." << std::endl;
    }

    // 2. Protection contre les NaN et Infs
    {
        actuator.update_actuation(NAN, INFINITY, 0.02f, false);
        assert(near(actuator.get_applied_speed(), 0.0f));
        assert(std::isfinite(actuator.get_applied_steering()));
        std::cout << "  ✓ Immunité aux NaN et Infs validée (vitesse forcée à 0%)." << std::endl;
    }

    // 3. Arrêt d'urgence (coupure immédiate)
    {
        actuator.update_actuation(50.0f, 20.0f, 0.02f, false);
        actuator.emergency_stop();
        assert(near(actuator.get_applied_speed(), 0.0f));
        assert(near(actuator.get_applied_steering(), 0.0f));
        std::cout << "  ✓ Arrêt d'urgence validé (vitesse et braquage immédiatement nuls)." << std::endl;
    }

    // 4. Décalage de direction (trim/offset)
    {
        actuator.set_steering_offset(5.0f);
        assert(near(actuator.get_steering_offset(), 5.0f));
        // Saturation de l'offset à +/- 15°
        actuator.set_steering_offset(35.0f);
        assert(near(actuator.get_steering_offset(), 15.0f));
        actuator.set_steering_offset(-35.0f);
        assert(near(actuator.get_steering_offset(), -15.0f));
        std::cout << "  ✓ Offset de direction borné à [-15°, +15°] validé." << std::endl;
    }

    std::cout << "==> TOUS LES TESTS DE MOTOR_ACTUATOR ONT RÉUSSI !" << std::endl;
    return 0;
}
