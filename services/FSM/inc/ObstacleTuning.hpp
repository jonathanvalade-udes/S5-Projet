#pragma once

#include <algorithm>
#include <array>
#include <string>

namespace services {

/// Reglages de l'approche d'un obstacle, lus dans fsm.conf et modifiables a chaud. Ils completent OBSTACLE_STOP_DIST_CM.
///
/// Deroulement, du plus loin au plus pres :
///   OBSTACLE_SLOWDOWN_DIST_CM         le robot commence a ralentir (pour pouvoir s'arreter sans depasser l'acceleration max)
///   OBSTACLE_STOP_DIST_CM + marge     apres l'arret, si l'obstacle est encore a cette distance, la manoeuvre d'evitement demarre
///   OBSTACLE_STOP_DIST_CM             le robot s'arrete
/// Apres l'arret, le robot attend OBSTACLE_STOP_WAIT_S avant de decider (evitement ou reprise du suivi de ligne).
///
/// Une valeur hors bornes est ramenee dans l'intervalle. Les bornes ne servent qu'a eviter des valeurs absurdes.
struct ObstacleTuning {
    float slowdown_dist_cm{45.0f};   // OBSTACLE_SLOWDOWN_DIST_CM
    float avoid_margin_cm{5.0f};     // OBSTACLE_AVOID_MARGIN_CM : marge au-dessus de la distance d'arret
    float stop_wait_s{0.2f};         // OBSTACLE_STOP_WAIT_S : attente apres l'arret avant de decider

    struct Field {
        const char* key;
        float ObstacleTuning::*member;
        float min;
        float max;
    };

    static const std::array<Field, 3>& fields() {
        static const std::array<Field, 3> table{{
            {"OBSTACLE_SLOWDOWN_DIST_CM", &ObstacleTuning::slowdown_dist_cm, 5.0f, 400.0f},
            {"OBSTACLE_AVOID_MARGIN_CM",  &ObstacleTuning::avoid_margin_cm,  0.0f, 100.0f},
            {"OBSTACLE_STOP_WAIT_S",      &ObstacleTuning::stop_wait_s,      0.0f, 10.0f},
        }};
        return table;
    }

    /// Regle la cle demandee (bornee). Retourne false si la cle n'est pas un de ces reglages.
    bool set(const std::string& key, float value) {
        for (const auto& f : fields()) {
            if (key == f.key) {
                this->*(f.member) = std::clamp(value, f.min, f.max);
                return true;
            }
        }
        return false;
    }

    /// Valeur courante d'une cle (0 si inconnue).
    float get(const std::string& key) const {
        for (const auto& f : fields()) {
            if (key == f.key) return this->*(f.member);
        }
        return 0.0f;
    }
};

} // namespace services
