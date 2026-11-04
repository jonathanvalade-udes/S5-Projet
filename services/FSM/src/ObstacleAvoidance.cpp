#include "ObstacleAvoidance.hpp"
#include <iostream>
#include <cmath>

namespace services {

void ObstacleAvoidance::reset(float trim_deg, float obstacle_dist) {
    m_dev_phase = 0;
    m_dev_phase_distance = 0.0f;
    m_dev_x = 0.0f;
    m_dev_y = 0.0f;
    m_dev_obstacle_far_x = 0.0f;
    m_dev_trim_deg = trim_deg;
    m_dev_obstacle_dist = obstacle_dist;
    m_dev_steer_real = 0.0f;
    m_dev_heading = 0.0f;
    m_state_timer = 0.0f;
}

ObstacleAvoidance::DeviationResult ObstacleAvoidance::update(
    float side, float dt, const float darkness[5],
    float current_speed, float min_curve_speed,
    float& steering_deg, float& velocity) {

    constexpr float REVERSE_DISTANCE_M = 0.10f;
    constexpr float FORWARD_SPEED_NOMINAL = 0.16f;
    constexpr float REVERSE_SPEED_NOMINAL = 0.12f;
    constexpr float RETURN_SPEED_NOMINAL = 0.15f;
    constexpr float RETURN_STEER_DEG = 30.0f;
    constexpr float RETURN_HEADING_RAD = 35.0f * 3.14159265f / 180.0f;
    constexpr float RETURN_STRAIGHT_MAX_M = 0.60f;
    constexpr float ARC_STEER_DEG = 45.0f;
    constexpr float ARC_HEADING_RAD = 82.0f * 3.14159265f / 180.0f;
    constexpr float WHEELBASE_M = 0.14f;
    constexpr float STEER_DEG_TO_REAL = 48.0f / 45.0f;
    constexpr float STEER_LAG_S = 0.06f;
    constexpr float OBSTACLE_DEPTH_M = 0.15f;
    constexpr float CAR_REAR_M = 0.15f;
    constexpr float CLEARANCE_M = 0.03f;
    constexpr float TIMEOUT_S = 14.0f;

    const float FORWARD_SPEED = avoidance_speed(FORWARD_SPEED_NOMINAL, min_curve_speed);
    const float REVERSE_SPEED = avoidance_speed(REVERSE_SPEED_NOMINAL, min_curve_speed);
    const float RETURN_SPEED = avoidance_speed(RETURN_SPEED_NOMINAL, min_curve_speed);

    m_state_timer += dt;
    const float v = current_speed;
    m_dev_phase_distance += std::abs(v) * dt;

    auto integrate = [&](float steer_deg) {
        const float applied = std::clamp(steer_deg + m_dev_trim_deg, -45.0f, 45.0f) - m_dev_trim_deg;
        m_dev_steer_real += (applied - m_dev_steer_real) * std::min(1.0f, dt / STEER_LAG_S);
        m_dev_heading += v / WHEELBASE_M * std::tan(m_dev_steer_real * STEER_DEG_TO_REAL * 3.14159265f / 180.0f) * dt * side;
        m_dev_x += v * std::cos(m_dev_heading) * dt;
        m_dev_y += v * std::sin(m_dev_heading) * dt;
    };

    auto next_phase = [&]() {
        std::cout << "[ObstacleAvoidance] Phase " << m_dev_phase << " -> " << (m_dev_phase + 1)
                  << " (x=" << m_dev_x << " y=" << m_dev_y << " cap=" << (m_dev_heading * 57.2958f)
                  << " deg, t=" << m_state_timer << " s)" << std::endl;
        m_dev_phase++;
        m_dev_phase_distance = 0.0f;
    };

    switch (m_dev_phase) {
        case 0:
            steering_deg = 0.0f;
            velocity = -REVERSE_SPEED;
            integrate(steering_deg);
            if (m_dev_phase_distance >= REVERSE_DISTANCE_M) {
                m_dev_obstacle_far_x = m_dev_obstacle_dist + OBSTACLE_DEPTH_M;
                next_phase();
            }
            break;
        case 1:
            steering_deg = side * ARC_STEER_DEG;
            velocity = FORWARD_SPEED;
            integrate(steering_deg);
            if (m_dev_heading >= ARC_HEADING_RAD) next_phase();
            break;
        case 2:
            steering_deg = -side * ARC_STEER_DEG;
            velocity = FORWARD_SPEED;
            integrate(steering_deg);
            if (m_dev_heading <= 0.0f) next_phase();
            break;
        case 3:
            steering_deg = 0.0f;
            velocity = FORWARD_SPEED;
            integrate(steering_deg);
            if (m_dev_x - CAR_REAR_M >= m_dev_obstacle_far_x + CLEARANCE_M) next_phase();
            break;
        case 4:
        default: {
            const bool turning = (m_dev_phase == 4);
            steering_deg = turning ? -side * RETURN_STEER_DEG : 0.0f;
            velocity = RETURN_SPEED;
            integrate(steering_deg);
            const float max_darkness = std::max({darkness[0], darkness[1], darkness[2], darkness[3], darkness[4]});
            const float total_darkness = darkness[0] + darkness[1] + darkness[2] + darkness[3] + darkness[4];
            if (max_darkness >= 0.50f || (total_darkness >= 0.35f && max_darkness >= 0.35f)) {
                return DeviationResult::LineFound;
            }
            if (turning && m_dev_heading <= -RETURN_HEADING_RAD) next_phase();
            else if (!turning && m_dev_phase_distance >= RETURN_STRAIGHT_MAX_M) return DeviationResult::SearchLine;
            break;
        }
    }

    steering_deg = std::clamp(steering_deg + m_dev_trim_deg, -45.0f, 45.0f);
    return m_state_timer >= TIMEOUT_S ? DeviationResult::SearchLine : DeviationResult::Running;
}

} // namespace services
