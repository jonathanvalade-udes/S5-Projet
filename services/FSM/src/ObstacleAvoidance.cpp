#include "ObstacleAvoidance.hpp"
#include <iostream>
#include <cmath>

namespace services {

namespace {

// ================================ CONSTANTES FIXES DE LA MANOEUVRE ================================
// Les angles, durees et vitesses se reglent dans fsm.conf (voir AvoidanceSettings.hpp). Restent ici :
//
// Vitesse d'avance des segments qui n'ont pas de vitesse propre (m/s du FSM). Avec STRAIGHT_CRUISE_SPEED = 0.32, elle vaut
// des m/s reels et 0.16 = 25 % de puissance moteur (la puissance la plus basse qui fait avancer le vrai robot).
constexpr float FORWARD_SPEED = 0.16f;
constexpr float REVERSE_SPEED = 0.16f;

// Securite : duree maximale de toute la manoeuvre (augmentee automatiquement si les durees reglees la depassent)
constexpr float TOTAL_TIMEOUT_S = 14.0f;

// Detection du ruban (etape 12) : un capteur assez sombre, ou plusieurs moyennement sombres
constexpr float LINE_MAX_DARKNESS = 0.50f;
constexpr float LINE_TOTAL_DARKNESS = 0.35f;
constexpr float LINE_TOTAL_MIN_SENSOR = 0.35f;
// ==================================================================================================

struct SegmentInfo {
    int step;               // numero de l'etape pendant laquelle le robot avance
    const char* label;      // texte du journal
};

constexpr SegmentInfo SEGMENTS[] = {
    {1,  "etape 1 : reculer"},
    {3,  "etapes 2-3 : roues braquees (cote choisi), avancer"},
    {5,  "etapes 4-5 : roues droites, avancer en diagonale"},
    {7,  "etapes 6-7 : roues braquees de l'autre cote, avancer"},
    {8,  "etape 8 : roues droites, avancer (parallele a la ligne)"},
    {10, "etapes 9-10 : roues braquees vers la ligne, avancer"},
    {12, "etapes 11-12 : roues droites, avancer jusqu'a trouver la ligne"},
};

} // namespace

void ObstacleAvoidance::reset(float trim_deg, float /*obstacle_dist*/) {
    m_segment = Reverse;
    m_segment_time = 0.0f;
    m_segment_distance = 0.0f;
    m_total_time = 0.0f;
    m_trim_deg = trim_deg;
}

int ObstacleAvoidance::get_step() const {
    return SEGMENTS[m_segment].step;
}

float ObstacleAvoidance::total_timeout_s() const {
    // Recul + toutes les durees reglees + marge : la securite ne doit jamais couper une manoeuvre reglee
    const float reverse_s = m_cfg.reverse_dist_m / REVERSE_SPEED + 0.5f;
    const float budget = reverse_s + m_cfg.time1_s + m_cfg.diag_time_s + m_cfg.time2_s + m_cfg.parallel_time_s +
                         m_cfg.time3_s + m_cfg.return_time_s + 3.0f;
    return std::max(TOTAL_TIMEOUT_S, budget);
}

bool ObstacleAvoidance::line_seen(const float darkness[5]) {
    const float max_darkness = std::max({darkness[0], darkness[1], darkness[2], darkness[3], darkness[4]});
    const float total_darkness = darkness[0] + darkness[1] + darkness[2] + darkness[3] + darkness[4];
    return max_darkness >= LINE_MAX_DARKNESS ||
           (total_darkness >= LINE_TOTAL_DARKNESS && max_darkness >= LINE_TOTAL_MIN_SENSOR);
}

float ObstacleAvoidance::duration_of(Segment seg) const {
    switch (seg) {
        case TurnAway:     return m_cfg.time1_s;
        case Diagonal:     return m_cfg.diag_time_s;
        case TurnOpposite: return m_cfg.time2_s;
        case Parallel:     return m_cfg.parallel_time_s;
        case TurnToLine:   return m_cfg.time3_s;
        case FindLine:     return m_cfg.return_time_s;
        default:           return 0.0f;
    }
}

ObstacleAvoidance::Command ObstacleAvoidance::command_for(Segment seg, float side) const {
    switch (seg) {
        case Reverse:      return {0.0f, -REVERSE_SPEED};
        case TurnAway:     return {side * m_cfg.angle1_deg, FORWARD_SPEED};      // etapes 2-3 : vers le cote choisi
        case Diagonal:     return {0.0f, m_cfg.diag_speed};                      // etapes 4-5 : roues droites
        case TurnOpposite: return {-side * m_cfg.angle2_deg, FORWARD_SPEED};     // etapes 6-7 : de l'autre cote
        case Parallel:     return {0.0f, FORWARD_SPEED};                         // etape 8 : roues droites
        case TurnToLine:   return {-side * m_cfg.angle3_deg, FORWARD_SPEED};     // etapes 9-10 : vers la ligne
        case FindLine:     return {0.0f, m_cfg.return_speed};                    // etapes 11-12 : roues droites
        default:           return {0.0f, 0.0f};
    }
}

bool ObstacleAvoidance::segment_done() const {
    if (m_segment == Reverse) return m_segment_distance >= m_cfg.reverse_dist_m;   // le recul se mesure en distance
    return m_segment_time >= duration_of(m_segment);
}

void ObstacleAvoidance::next_segment() {
    const int next = static_cast<int>(m_segment) + 1;
    if (next >= SegmentCount) return;
    m_segment = static_cast<Segment>(next);
    m_segment_time = 0.0f;
    m_segment_distance = 0.0f;
    std::cout << "[ObstacleAvoidance] " << SEGMENTS[m_segment].label << " (t=" << m_total_time << " s)" << std::endl;
}

ObstacleAvoidance::DeviationResult ObstacleAvoidance::update(
    float side, float dt, const float darkness[5],
    float current_speed, float min_curve_speed,
    float& steering_deg, float& velocity) {

    (void)min_curve_speed;   // les vitesses de la manoeuvre sont fixes ou reglees (pas de plancher sur MIN_CURVE_SPEED)

    m_total_time += dt;
    m_segment_time += dt;
    m_segment_distance += std::abs(current_speed) * dt;

    const Command cmd = command_for(m_segment, side);
    steering_deg = cmd.steering_deg;
    velocity = cmd.velocity;

    if (m_segment == FindLine) {
        // Dernier segment : on s'arrete des que le ruban est vu, ou apres la duree maximale
        if (line_seen(darkness)) return DeviationResult::LineFound;
        if (segment_done()) return DeviationResult::SearchLine;
    } else if (segment_done()) {
        next_segment();
    }

    steering_deg = std::clamp(steering_deg + m_trim_deg, -45.0f, 45.0f);
    return m_total_time >= total_timeout_s() ? DeviationResult::SearchLine : DeviationResult::Running;
}

} // namespace services
