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

    /// Demarre une manoeuvre. trim_deg = biais de braquage appris (roues mal alignees).
    /// obstacle_dist (m) n'est plus utilise : la manoeuvre est reglee par des durees fixes.
    void reset(float trim_deg, float obstacle_dist);

    /// Un pas de manoeuvre. side = +1 (droite) / -1 (gauche), darkness = 5 capteurs de ligne (0..1),
    /// current_speed = vitesse reelle (m/s). Ecrit le braquage (deg) et la vitesse (m/s) a commander.
    DeviationResult update(float side, float dt, const float darkness[5],
                           float current_speed, float min_curve_speed,
                           float& steering_deg, float& velocity);

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
    bool phase_find_line(float forward_speed, float& steering_deg, float& velocity) const;

    /// Vire a fond (turn_direction = +1 / -1) pendant turn_time_s, puis roule droit : braquage selon le temps ecoule dans la phase.
    float turn_then_straight(float turn_direction, float turn_time_s) const;

    static bool line_seen(const float darkness[5]);
    void next_phase();

    int m_phase{1};
    float m_phase_time{0.0f};        // temps ecoule dans la phase courante (s)
    float m_phase_distance{0.0f};    // distance parcourue dans la phase courante (m)
    float m_total_time{0.0f};
    float m_trim_deg{0.0f};
    float m_ignore_finish_s{0.0f};
};

} // namespace services
