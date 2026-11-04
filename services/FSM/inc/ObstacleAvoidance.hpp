#pragma once

#include <cstdint>
#include <algorithm>

namespace services {

class ObstacleAvoidance {
public:
    enum class DeviationResult { Running, LineFound, SearchLine };

    ObstacleAvoidance() = default;
    ~ObstacleAvoidance() = default;

    void reset(float trim_deg, float obstacle_dist);
    DeviationResult update(float side, float dt, const float darkness[5],
                           float current_speed, float min_curve_speed,
                           float& steering_deg, float& velocity);

    void set_ignore_finish(float duration_s) { m_ignore_finish_s = duration_s; }
    void update_ignore_finish(float dt) { m_ignore_finish_s = std::max(0.0f, m_ignore_finish_s - dt); }
    bool is_finish_ignored() const { return m_ignore_finish_s > 0.0f; }

    int get_phase() const { return m_dev_phase; }
    float get_x() const { return m_dev_x; }
    float get_y() const { return m_dev_y; }
    float get_heading_deg() const { return m_dev_heading * 57.2958f; }

private:
    float avoidance_speed(float nominal, float min_curve_speed) const {
        return std::max(nominal, min_curve_speed);
    }

    int m_dev_phase{0};
    float m_dev_phase_distance{0.0f};
    float m_dev_x{0.0f};
    float m_dev_y{0.0f};
    float m_dev_obstacle_far_x{0.0f};
    float m_dev_trim_deg{0.0f};
    float m_dev_obstacle_dist{0.0f};
    float m_dev_steer_real{0.0f};
    float m_dev_heading{0.0f};
    float m_state_timer{0.0f};
    float m_ignore_finish_s{0.0f};
};

} // namespace services
