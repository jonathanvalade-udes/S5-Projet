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
// L'angle et les durees des virages se reglent dans fsm.conf (AVOID_TURN_ANGLE_DEG, AVOID_TURN_OUT_TIME_S, AVOID_TURN_TIME_S) ;
// valeurs par defaut (45 deg, 1,10 s pour le premier virage, 0,70 s pour les suivants) dans ObstacleAvoidance.hpp.

// --- Phase 2 : s'ecarter (virage de 45 deg puis diagonale) ---
// La duree et la vitesse de la diagonale se reglent dans fsm.conf (AVOID_DIAG_TIME_S, AVOID_DIAG_SPEED) ;
// leurs valeurs par defaut (1,10 s et 0,16 m/s) sont dans ObstacleAvoidance.hpp.

// --- Phase 3 : se redresser (virage retour puis ligne droite pour depasser le cube) ---
constexpr float PASS_TIME_S = 0.80f;

// --- Phase 4 : revenir (virage de 45 deg vers la piste) : dure TURN_TIME_S ---

// --- Phase 5 : retrouver la ligne (deuxieme diagonale), abandon au bout d'un temps maximal ---
// Duree maximale et vitesse se reglent dans fsm.conf (AVOID_RETURN_TIME_S, AVOID_RETURN_SPEED) ;
// valeurs par defaut (3,5 s et 0,16 m/s) dans ObstacleAvoidance.hpp.

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

void ObstacleAvoidance::set_diagonal(float time_s, float speed) {
    m_diag_time_s = std::clamp(time_s, MIN_DIAG_TIME_S, MAX_DIAG_TIME_S);
    m_diag_speed = std::clamp(speed, MIN_DIAG_SPEED, MAX_DIAG_SPEED);
}

void ObstacleAvoidance::set_return(float time_s, float speed) {
    m_return_time_s = std::clamp(time_s, MIN_RETURN_TIME_S, MAX_RETURN_TIME_S);
    m_return_speed = std::clamp(speed, MIN_RETURN_SPEED, MAX_RETURN_SPEED);
}

void ObstacleAvoidance::set_turn(float angle_deg, float out_time_s, float time_s) {
    m_turn_angle_deg = std::clamp(angle_deg, MIN_TURN_ANGLE_DEG, MAX_TURN_ANGLE_DEG);
    m_turn_out_time_s = std::clamp(out_time_s, MIN_TURN_OUT_TIME_S, MAX_TURN_OUT_TIME_S);
    m_turn_time_s = std::clamp(time_s, MIN_TURN_TIME_S, MAX_TURN_TIME_S);
}

float ObstacleAvoidance::total_timeout_s() const {
    // Recul (~2 s) + virages + diagonales + passage + recherche + marge : la securite ne doit jamais couper une manoeuvre reglee
    const float budget = 2.0f + m_turn_out_time_s + m_diag_time_s + 2.0f * m_turn_time_s + PASS_TIME_S + m_return_time_s + 3.0f;
    return std::max(TOTAL_TIMEOUT_S, budget);
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
    return m_phase_time < turn_time_s ? turn_direction * m_turn_angle_deg : 0.0f;
}

// Phase 1 : reculer de 15 cm en ligne droite
bool ObstacleAvoidance::phase_reverse(float reverse_speed, float& steering_deg, float& velocity) const {
    steering_deg = 0.0f;
    velocity = -reverse_speed;
    return m_phase_distance >= REVERSE_DISTANCE_M;
}

// Phase 2 : virer a 45 deg du cote choisi, puis avancer en diagonale
bool ObstacleAvoidance::phase_move_away(float side, float forward_speed, float& steering_deg, float& velocity) const {
    steering_deg = turn_then_straight(side, m_turn_out_time_s);
    velocity = (m_phase_time < m_turn_out_time_s) ? forward_speed : m_diag_speed;   // virage, puis diagonale
    return m_phase_time >= m_turn_out_time_s + m_diag_time_s;
}

// Phase 3 : virer de 45 deg de l'autre cote (cap parallele a la piste), puis avancer droit pour depasser le cube
bool ObstacleAvoidance::phase_straighten(float side, float forward_speed, float& steering_deg, float& velocity) const {
    steering_deg = turn_then_straight(-side, m_turn_time_s);
    velocity = forward_speed;
    return m_phase_time >= m_turn_time_s + PASS_TIME_S;
}

// Phase 4 : virer encore de 45 deg de l'autre cote (cap dirige vers la piste)
bool ObstacleAvoidance::phase_turn_back(float side, float forward_speed, float& steering_deg, float& velocity) const {
    steering_deg = -side * m_turn_angle_deg;
    velocity = forward_speed;
    return m_phase_time >= m_turn_time_s;
}

// Phase 5 : avancer en diagonale (braquage nul) ; la fin est decidee par la detection du ruban ou par le temps max
bool ObstacleAvoidance::phase_find_line(float& steering_deg, float& velocity) const {
    steering_deg = 0.0f;
    velocity = m_return_speed;
    return m_phase_time >= m_return_time_s;
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
            phase_done = phase_find_line(steering_deg, velocity);
            if (line_seen(darkness)) return DeviationResult::LineFound;
            if (phase_done) return DeviationResult::SearchLine;
            break;
    }
    if (phase_done) next_phase();

    steering_deg = std::clamp(steering_deg + m_trim_deg, -45.0f, 45.0f);
    return m_total_time >= total_timeout_s() ? DeviationResult::SearchLine : DeviationResult::Running;
}

} // namespace services
