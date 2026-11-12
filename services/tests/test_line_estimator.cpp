#include <cassert>
#include <cmath>
#include <iostream>
#include "Sensors/inc/LineEstimator.hpp"

using services::LineEstimator;
using services::LineEstimate;

static bool near(float a, float b, float eps = 1e-3f) {
    return std::fabs(a - b) < eps;
}

int main() {
    std::cout << "[TEST] LineEstimator - Tests unitaires de l'estimateur de ligne..." << std::endl;

    // 1. Ligne parfaitement centrée (capteur 2 = centre)
    {
        const float dark[5] = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
        LineEstimate est = LineEstimator::estimate_line(dark);
        assert(est.is_valid);
        assert(!est.is_line_lost);
        assert(!est.is_t_stop);
        assert(near(est.error, 0.0f));
        assert(near(est.normalized_error, 0.0f));
        assert(est.confidence > 0.8f);
        std::cout << "  ✓ Ligne centrée validée (err=0, conf=" << est.confidence << ")" << std::endl;
    }

    // 2. Ligne à gauche extrême (capteur 0)
    {
        const float dark[5] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        LineEstimate est = LineEstimator::estimate_line(dark);
        assert(est.is_valid);
        assert(!est.is_t_stop);
        assert(near(est.error, -2.0f));
        assert(near(est.normalized_error, -1.0f));
        assert(est.confidence > 0.8f);
        std::cout << "  ✓ Ligne gauche extrême validée (err=-2.0, conf=" << est.confidence << ")" << std::endl;
    }

    // 3. Ligne à droite extrême (capteur 4)
    {
        const float dark[5] = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
        LineEstimate est = LineEstimator::estimate_line(dark);
        assert(est.is_valid);
        assert(!est.is_t_stop);
        assert(near(est.error, 2.0f));
        assert(near(est.normalized_error, 1.0f));
        assert(est.confidence > 0.8f);
        std::cout << "  ✓ Ligne droite extrême validée (err=2.0, conf=" << est.confidence << ")" << std::endl;
    }

    // 4. Ligne entre deux capteurs (capteurs 1 et 2 à égale intensité)
    {
        const float dark[5] = {0.0f, 0.8f, 0.8f, 0.0f, 0.0f};
        LineEstimate est = LineEstimator::estimate_line(dark);
        assert(est.is_valid);
        // Barycentre: (-1.0*0.8 + 0.0*0.8) / (0.8 + 0.8) = -0.8 / 1.6 = -0.5
        assert(near(est.error, -0.5f));
        assert(near(est.normalized_error, -0.25f));
        std::cout << "  ✓ Ligne intermédiaire continue validée (err=-0.5)" << std::endl;
    }

    // 5. Perte de ligne (sol blanc uniforme, aucun contraste)
    {
        const float dark[5] = {0.02f, 0.03f, 0.02f, 0.01f, 0.02f};
        LineEstimate est = LineEstimator::estimate_line(dark);
        assert(!est.is_valid);
        assert(est.is_line_lost);
        assert(est.confidence == 0.0f);
        std::cout << "  ✓ Détection de perte de ligne validée (is_valid=false)" << std::endl;
    }

    // 6. Marqueur d'arrivée en T (au moins 4 capteurs très sombres)
    {
        const float dark[5] = {0.85f, 0.90f, 0.95f, 0.88f, 0.05f};
        LineEstimate est = LineEstimator::estimate_line(dark);
        assert(est.is_t_stop);
        std::cout << "  ✓ Détection marqueur d'arrivée T-Stop validée" << std::endl;
    }

    // 7. Robustesse aux valeurs non finies (NaN / Inf)
    {
        const float dark[5] = {NAN, 0.0f, 0.9f, INFINITY, 0.0f};
        LineEstimate est = LineEstimator::estimate_line(dark);
        assert(std::isfinite(est.error));
        assert(std::isfinite(est.confidence));
        std::cout << "  ✓ Robustesse face aux NaN / Inf validée" << std::endl;
    }

    std::cout << "==> TOUS LES TESTS DE LINE_ESTIMATOR ONT RÉUSSI !" << std::endl;
    return 0;
}
