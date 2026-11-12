#include "SpeedPlanner.hpp"
#include <algorithm>
#include <cmath>

namespace services {

SpeedPlanner::SpeedPlanner(float ball_max_accel, float max_accel, float max_brake)
    : m_ball_max_accel(ball_max_accel),
      m_max_accel(max_accel),
      m_max_brake(max_brake) {}

void SpeedPlanner::reset(float initial_speed) {
    m_current_speed = initial_speed;
    m_last_ax = 0.0f;
    m_last_ay = 0.0f;
}

float SpeedPlanner::calculate_stopping_distance(float speed_m_s) const {
    const float v = std::max(0.0f, speed_m_s);
    const float safe_brake = std::max(0.1f, m_max_brake);
    return (v * DEFAULT_REACTION_TIME) + ((v * v) / (2.0f * safe_brake)) + DEFAULT_STOP_MARGIN_M;
}

float SpeedPlanner::calculate_max_safe_approach_speed(float remaining_distance_to_stop_m) const {
    if (remaining_distance_to_stop_m <= 0.0f) {
        return 0.0f;
    }
    return std::sqrt(2.0f * m_max_brake * remaining_distance_to_stop_m);
}

float SpeedPlanner::update(float target_speed_m_s,
                           float steer_deg,
                           float dist_to_obstacle_m,
                           float obstacle_stop_dist_m,
                           float dt,
                           float curve_safety_factor) {
    // Protection du pas temporel
    if (dt <= 0.0f) {
        dt = 0.02f;
    } else if (dt > 0.2f) {
        dt = 0.2f;
    }

    if (!std::isfinite(target_speed_m_s)) {
        target_speed_m_s = 0.0f;
    }

    // 1. Limitation de vitesse en virage selon Ackermann & accélération centrifuge max
    float R = m_vehicle_model.calculate_turn_radius(steer_deg);
    float v_curve = m_vehicle_model.calculate_curve_speed_limit(steer_deg, m_max_lat_accel, curve_safety_factor);
    float v_target = std::min(target_speed_m_s, v_curve);

    // 2. Décélération anticipée devant obstacle
    if (dist_to_obstacle_m > 0.0f && dist_to_obstacle_m < 0.60f) {
        float remaining_m = dist_to_obstacle_m - obstacle_stop_dist_m;
        float v_obs = calculate_max_safe_approach_speed(remaining_m);
        v_target = std::min(v_target, v_obs);
    }

    // 3. Modélisation de l'accélération latérale instantanée
    float safe_R = std::max(0.10f, R);
    m_last_ay = (m_current_speed * m_current_speed) / safe_R;

    // 4. Protection couplée de la bille (a_résultante <= a_bille_max)
    // a_x_max = sqrt(max(0, a_bille_max² - a_y²))
    float ay_sq = m_last_ay * m_last_ay;
    float ball_sq = m_ball_max_accel * m_ball_max_accel;
    float max_ax_allowed = (ball_sq > ay_sq) ? std::sqrt(ball_sq - ay_sq) : 0.05f;

    // 5. Rampe temporelle asymétrique avec respect strict de l'accélération résultante bille
    if (v_target > m_current_speed) {
        float effective_accel = std::min(m_max_accel, max_ax_allowed);
        float step = effective_accel * dt;
        float next_v = std::min(v_target, m_current_speed + step);
        m_last_ax = (next_v - m_current_speed) / dt;
        m_current_speed = next_v;
    } else {
        float effective_brake = std::min(m_max_brake, max_ax_allowed);
        float step = effective_brake * dt;
        float next_v = std::max(v_target, m_current_speed - step);
        m_last_ax = (next_v - m_current_speed) / dt;
        m_current_speed = next_v;
    }

    return m_current_speed;
}

} // namespace services
