#pragma once

#include <algorithm>
#include <array>
#include <string>

namespace services {

/// Tous les reglages de la manoeuvre de contournement d'obstacle, lus dans fsm.conf (cles AVOID_*) et modifiables a chaud.
///
/// La manoeuvre suit ces 12 etapes (les numeros sont ceux de la documentation) :
///   1      Reculer                                              AVOID_REVERSE_DIST_M
///   2 + 3  Roues braquees de X deg (cote choisi), avancer X s   AVOID_ANGLE_1_DEG, AVOID_TIME_1_S
///   4 + 5  Roues droites, avancer X s (diagonale)               AVOID_DIAG_TIME_S, AVOID_DIAG_SPEED
///   6 + 7  Roues braquees de X deg de l'autre cote, avancer X s AVOID_ANGLE_2_DEG, AVOID_TIME_2_S
///   8      Roues droites, avancer plus (parallele a la ligne)   AVOID_PARALLEL_TIME_S
///   9 + 10 Roues braquees de X deg vers la ligne, avancer X s   AVOID_ANGLE_3_DEG, AVOID_TIME_3_S
///   11 + 12 Roues droites, avancer jusqu'a trouver la ligne     AVOID_RETURN_TIME_S (duree max), AVOID_RETURN_SPEED
///
/// Les valeurs par defaut sont celles validees en simulation. Une valeur hors bornes est ramenee dans l'intervalle.
/// Vitesses en m/s du FSM : avec STRAIGHT_CRUISE_SPEED = 0.32, 0.16 = 25 % de puissance moteur (environ 0,16 m/s).
struct AvoidanceSettings {
    float reverse_dist_m{0.25f};      // etape 1
    float angle1_deg{45.0f};          // etape 2
    float time1_s{1.10f};             // etape 3
    float diag_time_s{1.10f};         // etape 5
    float diag_speed{0.16f};          // etape 5
    float angle2_deg{45.0f};          // etape 6
    float time2_s{0.70f};             // etape 7
    float parallel_time_s{1.00f};     // etape 8 (le robot est parallele a la ligne : il avance plus)
    float angle3_deg{45.0f};          // etape 9
    float time3_s{0.70f};             // etape 10
    float return_time_s{3.5f};        // etape 12 : duree maximale de la recherche de la ligne
    float return_speed{0.16f};        // etape 12

    struct Field {
        const char* key;
        float AvoidanceSettings::*member;
        float min;
        float max;
    };

    static const std::array<Field, 12>& fields() {
        static const std::array<Field, 12> table{{
            {"AVOID_REVERSE_DIST_M",   &AvoidanceSettings::reverse_dist_m,  0.10f, 0.50f},
            {"AVOID_ANGLE_1_DEG",      &AvoidanceSettings::angle1_deg,      15.0f, 45.0f},
            {"AVOID_TIME_1_S",         &AvoidanceSettings::time1_s,         0.3f,  3.0f},
            {"AVOID_DIAG_TIME_S",      &AvoidanceSettings::diag_time_s,     0.3f,  3.0f},
            {"AVOID_DIAG_SPEED",       &AvoidanceSettings::diag_speed,      0.12f, 0.25f},
            {"AVOID_ANGLE_2_DEG",      &AvoidanceSettings::angle2_deg,      15.0f, 45.0f},
            {"AVOID_TIME_2_S",         &AvoidanceSettings::time2_s,         0.2f,  2.0f},
            {"AVOID_PARALLEL_TIME_S",  &AvoidanceSettings::parallel_time_s, 0.2f,  4.0f},
            {"AVOID_ANGLE_3_DEG",      &AvoidanceSettings::angle3_deg,      15.0f, 45.0f},
            {"AVOID_TIME_3_S",         &AvoidanceSettings::time3_s,         0.2f,  2.0f},
            {"AVOID_RETURN_TIME_S",    &AvoidanceSettings::return_time_s,   1.0f,  5.0f},
            {"AVOID_RETURN_SPEED",     &AvoidanceSettings::return_speed,    0.12f, 0.25f},
        }};
        return table;
    }

    /// Ramene tous les reglages dans leurs bornes.
    void clamp_all() {
        for (const auto& f : fields()) this->*(f.member) = std::clamp(this->*(f.member), f.min, f.max);
    }

    /// Regle la cle demandee (bornee). Retourne false si la cle n'est pas un reglage d'evitement.
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
