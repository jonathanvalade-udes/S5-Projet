#pragma once

#include <algorithm>
#include "AvoidanceSettings.hpp"

namespace services {

/// Manoeuvre de contournement d'un obstacle, sans capteur de position : une suite de 7 segments, qui couvre les 12 etapes
/// decrites dans AvoidanceSettings.hpp. Chaque segment fixe l'angle des roues et la vitesse, et dure un temps reglable
/// (le premier dure une distance, le dernier jusqu'a ce que la ligne soit retrouvee).
///
///   segment 1  etape 1        reculer
///   segment 2  etapes 2-3     roues braquees (cote choisi), avancer
///   segment 3  etapes 4-5     roues droites, avancer (diagonale)
///   segment 4  etapes 6-7     roues braquees de l'autre cote, avancer
///   segment 5  etape 8        roues droites, avancer plus (parallele a la ligne)
///   segment 6  etapes 9-10    roues braquees vers la ligne, avancer
///   segment 7  etapes 11-12   roues droites, avancer jusqu'a trouver la ligne
///
/// Appelee a chaque cycle par FsmNavigator (etats RightDeviation / LeftDeviation).
class ObstacleAvoidance {
public:
    enum class DeviationResult { Running, LineFound, SearchLine };

    /// Reglages (angles, durees, vitesses). Valeurs bornees. A appeler avant reset() ou a chaud.
    void set_settings(const AvoidanceSettings& settings) {
        m_cfg = settings;
        m_cfg.clamp_all();
    }
    const AvoidanceSettings& settings() const { return m_cfg; }

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

    /// Numero de l'etape (1 a 12) pendant laquelle le robot est en mouvement : 1, 3, 5, 7, 8, 10 ou 12.
    int get_step() const;

private:
    enum Segment { Reverse = 0, TurnAway, Diagonal, TurnOpposite, Parallel, TurnToLine, FindLine, SegmentCount };

    struct Command { float steering_deg; float velocity; };

    Command command_for(Segment seg, float side) const;   // angle des roues et vitesse du segment
    float duration_of(Segment seg) const;                 // duree (s) du segment ; ignoree pour Reverse (distance) et FindLine (maximum)
    bool segment_done() const;
    float total_timeout_s() const;                        // securite : au moins la somme des durees reglees
    static bool line_seen(const float darkness[5]);
    void next_segment();

    AvoidanceSettings m_cfg;

    Segment m_segment{Reverse};
    float m_segment_time{0.0f};       // temps ecoule dans le segment courant (s)
    float m_segment_distance{0.0f};   // distance parcourue dans le segment courant (m)
    float m_total_time{0.0f};
    float m_trim_deg{0.0f};
    float m_ignore_finish_s{0.0f};
};

} // namespace services
