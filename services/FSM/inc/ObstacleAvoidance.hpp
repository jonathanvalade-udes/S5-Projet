#pragma once

#include <algorithm>

namespace services {

/// Manoeuvre de contournement d'un obstacle : 5 phases simples, sans capteur de position.
/// Toutes les valeurs (distances, angles, durees) sont ecrites en dur en tete de ObstacleAvoidance.cpp.
///
///   phase 1  RECULER        recule en ligne droite (distance reglable)
///   phase 2  S'ECARTER      vire a 45 deg du cote choisi, puis avance en diagonale
///   phase 3  SE REDRESSER   vire a 45 deg de l'autre cote (cap parallele a la piste), puis avance droit
///                           pour depasser le cube
///   phase 4  REVENIR        vire encore a 45 deg de l'autre cote (cap vers la piste)
///   phase 5  RETROUVER      avance en diagonale jusqu'a ce qu'un capteur voie le ruban
///
/// Appelee a chaque cycle par FsmNavigator (etats RightDeviation / LeftDeviation).
class ObstacleAvoidance {
public:
    enum class DeviationResult { Running, LineFound, SearchLine };

    /// Valeurs par defaut de la diagonale de la phase 2 (reglables dans fsm.conf : AVOID_DIAG_TIME_S / AVOID_DIAG_SPEED).
    /// Distance parcourue en diagonale = vitesse x duree (0,16 x 1,10 = environ 18 cm).
    static constexpr float DEFAULT_DIAG_TIME_S = 1.10f;
    static constexpr float DEFAULT_DIAG_SPEED = 0.16f;
    /// Bornes acceptees (une valeur hors bornes est ramenee dans l'intervalle).
    static constexpr float MIN_DIAG_TIME_S = 0.3f;
    static constexpr float MAX_DIAG_TIME_S = 3.0f;
    static constexpr float MIN_DIAG_SPEED = 0.12f;
    static constexpr float MAX_DIAG_SPEED = 0.25f;

    /// Deuxieme diagonale (phase 5, retour vers la piste) : AVOID_RETURN_TIME_S = duree maximale de la recherche du ruban,
    /// AVOID_RETURN_SPEED = vitesse. La phase s'arrete des que le ruban est vu.
    static constexpr float DEFAULT_RETURN_TIME_S = 3.5f;
    static constexpr float DEFAULT_RETURN_SPEED = 0.16f;
    static constexpr float MIN_RETURN_TIME_S = 1.0f;
    static constexpr float MAX_RETURN_TIME_S = 5.0f;
    static constexpr float MIN_RETURN_SPEED = 0.12f;
    static constexpr float MAX_RETURN_SPEED = 0.25f;

    /// Demarre une manoeuvre. trim_deg = biais de braquage appris (roues mal alignees).
    /// obstacle_dist (m) n'est plus utilise : la manoeuvre est reglee par des durees fixes.
    void reset(float trim_deg, float obstacle_dist);

    /// Un pas de manoeuvre. side = +1 (droite) / -1 (gauche), darkness = 5 capteurs de ligne (0..1),
    /// current_speed = vitesse reelle (m/s). Ecrit le braquage (deg) et la vitesse (m/s) a commander.
    DeviationResult update(float side, float dt, const float darkness[5],
                           float current_speed, float min_curve_speed,
                           float& steering_deg, float& velocity);

    /// Regle la diagonale de la phase 2 : duree (s) et vitesse (m/s). Valeurs bornees (0,3 a 3 s ; 0,12 a 0,25 m/s).
    void set_diagonal(float time_s, float speed);

    /// Regle la deuxieme diagonale (phase 5) : duree maximale (s) et vitesse (m/s), bornees comme ci-dessus.
    void set_return(float time_s, float speed);

    void set_ignore_finish(float duration_s) { m_ignore_finish_s = duration_s; }
    void update_ignore_finish(float dt) { m_ignore_finish_s = std::max(0.0f, m_ignore_finish_s - dt); }
    bool is_finish_ignored() const { return m_ignore_finish_s > 0.0f; }

    int get_phase() const { return m_phase; }

private:
    // Chaque phase fixe le braquage et la vitesse, et retourne true quand elle est terminee.
    bool phase_reverse(float reverse_speed, float& steering_deg, float& velocity) const;
    bool phase_move_away(float side, float forward_speed, float& steering_deg, float& velocity) const;
    bool phase_straighten(float side, float forward_speed, float& steering_deg, float& velocity) const;
    bool phase_turn_back(float side, float forward_speed, float& steering_deg, float& velocity) const;
    bool phase_find_line(float& steering_deg, float& velocity) const;

    /// Vire a fond (turn_direction = +1 / -1) pendant turn_time_s, puis roule droit : braquage selon le temps ecoule dans la phase.
    float turn_then_straight(float turn_direction, float turn_time_s) const;

    static bool line_seen(const float darkness[5]);
    void next_phase();

    float m_diag_time_s{DEFAULT_DIAG_TIME_S};   // phase 2 : duree de la diagonale
    float m_diag_speed{DEFAULT_DIAG_SPEED};     // phase 2 : vitesse pendant la diagonale
    float m_return_time_s{DEFAULT_RETURN_TIME_S};   // phase 5 : duree maximale de la diagonale de retour
    float m_return_speed{DEFAULT_RETURN_SPEED};     // phase 5 : vitesse pendant la diagonale de retour

    int m_phase{1};
    float m_phase_time{0.0f};        // temps ecoule dans la phase courante (s)
    float m_phase_distance{0.0f};    // distance parcourue dans la phase courante (m)
    float m_total_time{0.0f};
    float m_trim_deg{0.0f};
    float m_ignore_finish_s{0.0f};
};

} // namespace services
