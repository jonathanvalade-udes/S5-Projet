#include "ObstacleAvoidance.hpp"
#include <iostream>
#include <cmath>

namespace services {

namespace {

// ================================ REGLAGES DE LA MANOEUVRE ================================
// Tout se regle ici, par essais. Les durees supposent la vitesse d'evitement ci-dessous.
//
// Vitesses en m/s du FSM. Avec STRAIGHT_CRUISE_SPEED = 0.32, elles valent des m/s reels et 0.16 = 25 % de puissance
// moteur (la puissance la plus basse qui fait avancer le vrai robot). Si la croisiere change, ces vitesses
// changent de puissance : il faut alors recalibrer les durees.
constexpr float FORWARD_SPEED = 0.16f;
constexpr float REVERSE_SPEED = 0.16f;

// --- Phase 1 : reculer (assez pour avoir la place d'ecarter la voiture avant d'arriver au cube) ---
constexpr float REVERSE_DISTANCE_M = 0.25f;

// --- Virages a fond (braquage 45 deg) qui changent le cap d'environ 45 deg ---
// Le 1er virage est plus long : le robot repart de l'arret apres la marche arriere (acceleration + retard du servo).
constexpr float TURN_STEER_DEG = 45.0f;
constexpr float TURN_OUT_TIME_S = 1.10f;   // phase 2 (depart de l'arret)
constexpr float TURN_TIME_S = 0.70f;       // phases 3 et 4 (robot deja en mouvement)

// --- Phase 2 : s'ecarter (virage de 45 deg puis diagonale) ---
constexpr float DIAGONAL_TIME_S = 1.10f;

// --- Phase 3 : se redresser (virage retour puis ligne droite pour depasser le cube) ---
constexpr float PASS_TIME_S = 0.80f;

// --- Phase 4 : revenir (virage de 45 deg vers la piste) : dure TURN_TIME_S ---

// --- Phase 5 : retrouver la ligne (diagonale), abandon au bout de ce temps ---
constexpr float FIND_LINE_MAX_TIME_S = 3.50f;

// --- Securite : duree maximale de toute la manoeuvre ---
constexpr float TOTAL_TIMEOUT_S = 14.0f;

// --- Detection du ruban : un capteur assez sombre, ou plusieurs moyennement sombres ---
constexpr float LINE_MAX_DARKNESS = 0.50f;
constexpr float LINE_TOTAL_DARKNESS = 0.35f;
constexpr float LINE_TOTAL_MIN_SENSOR = 0.35f;
// ==========================================================================================

} // namespace

void ObstacleAvoidance::reset(float trim_deg, float /*obstacle_dist*/) {
    m_phase = 1;
    m_phase_time = 0.0f;
    m_phase_distance = 0.0f;
    m_total_time = 0.0f;
    m_trim_deg = trim_deg;
}

void ObstacleAvoidance::next_phase() {
    std::cout << "[ObstacleAvoidance] Phase " << m_phase << " -> " << (m_phase + 1)
              << " (t=" << m_total_time << " s)" << std::endl;
    m_phase++;
    m_phase_time = 0.0f;
    m_phase_distance = 0.0f;
}

bool ObstacleAvoidance::line_seen(const float darkness[5]) {
    const float max_darkness = std::max({darkness[0], darkness[1], darkness[2], darkness[3], darkness[4]});
    const float total_darkness = darkness[0] + darkness[1] + darkness[2] + darkness[3] + darkness[4];
    return max_darkness >= LINE_MAX_DARKNESS ||
           (total_darkness >= LINE_TOTAL_DARKNESS && max_darkness >= LINE_TOTAL_MIN_SENSOR);
}

float ObstacleAvoidance::turn_then_straight(float turn_direction, float turn_time_s) const {
    return m_phase_time < turn_time_s ? turn_direction * TURN_STEER_DEG : 0.0f;
}

// Phase 1 : reculer de 15 cm en ligne droite
bool ObstacleAvoidance::phase_reverse(float reverse_speed, float& steering_deg, float& velocity) const {
    steering_deg = 0.0f;
    velocity = -reverse_speed;
    return m_phase_distance >= REVERSE_DISTANCE_M;
}

// Phase 2 : virer a 45 deg du cote choisi, puis avancer en diagonale
bool ObstacleAvoidance::phase_move_away(float side, float forward_speed, float& steering_deg, float& velocity) const {
    steering_deg = turn_then_straight(side, TURN_OUT_TIME_S);
    velocity = forward_speed;
    return m_phase_time >= TURN_OUT_TIME_S + DIAGONAL_TIME_S;
}

// Phase 3 : virer de 45 deg de l'autre cote (cap parallele a la piste), puis avancer droit pour depasser le cube
bool ObstacleAvoidance::phase_straighten(float side, float forward_speed, float& steering_deg, float& velocity) const {
    steering_deg = turn_then_straight(-side, TURN_TIME_S);
    velocity = forward_speed;
    return m_phase_time >= TURN_TIME_S + PASS_TIME_S;
}

// Phase 4 : virer encore de 45 deg de l'autre cote (cap dirige vers la piste)
bool ObstacleAvoidance::phase_turn_back(float side, float forward_speed, float& steering_deg, float& velocity) const {
    steering_deg = -side * TURN_STEER_DEG;
    velocity = forward_speed;
    return m_phase_time >= TURN_TIME_S;
}

// Phase 5 : avancer en diagonale (braquage nul) ; la fin est decidee par la detection du ruban ou par le temps max
bool ObstacleAvoidance::phase_find_line(float forward_speed, float& steering_deg, float& velocity) const {
    steering_deg = 0.0f;
    velocity = forward_speed;
    return m_phase_time >= FIND_LINE_MAX_TIME_S;
}

ObstacleAvoidance::DeviationResult ObstacleAvoidance::update(
    float side, float dt, const float darkness[5],
    float current_speed, float min_curve_speed,
    float& steering_deg, float& velocity) {

    (void)min_curve_speed;   // les vitesses de la manoeuvre sont fixes (voir REGLAGES), sans plancher
    const float forward_speed = FORWARD_SPEED;
    const float reverse_speed = REVERSE_SPEED;

    m_total_time += dt;
    m_phase_time += dt;
    m_phase_distance += std::abs(current_speed) * dt;

    bool phase_done = false;
    switch (m_phase) {
        case 1: phase_done = phase_reverse(reverse_speed, steering_deg, velocity); break;
        case 2: phase_done = phase_move_away(side, forward_speed, steering_deg, velocity); break;
        case 3: phase_done = phase_straighten(side, forward_speed, steering_deg, velocity); break;
        case 4: phase_done = phase_turn_back(side, forward_speed, steering_deg, velocity); break;
        default:
            phase_done = phase_find_line(forward_speed, steering_deg, velocity);
            if (line_seen(darkness)) return DeviationResult::LineFound;
            if (phase_done) return DeviationResult::SearchLine;
            break;
    }
    if (phase_done) next_phase();

    steering_deg = std::clamp(steering_deg + m_trim_deg, -45.0f, 45.0f);
    return m_total_time >= TOTAL_TIMEOUT_S ? DeviationResult::SearchLine : DeviationResult::Running;
}

} // namespace services
