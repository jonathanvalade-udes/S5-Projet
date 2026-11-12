#include "VehicleModel.hpp"
#include <algorithm>
#include <cmath>

namespace services {

VehicleModel::VehicleModel() {
    init_default_table();
}

void VehicleModel::init_default_table() {
    // Points d'étalonnage provisoires basés sur les mesures réelles (0.32 m/s à 50% PWM)
    // et la zone morte typique du pont en H TB6612FNG
    m_calibration_table = {
        {0.0f,   0.00f},
        {20.0f,  0.05f},
        {30.0f,  0.16f},
        {40.0f,  0.25f},
        {50.0f,  0.32f},
        {60.0f,  0.40f},
        {70.0f,  0.48f},
        {80.0f,  0.55f},
        {100.0f, 0.65f}
    };
}

void VehicleModel::set_calibration_table(const std::vector<CalibrationPoint>& table) {
    if (table.size() >= 2) {
        m_calibration_table = table;
        std::sort(m_calibration_table.begin(), m_calibration_table.end(),
                  [](const CalibrationPoint& a, const CalibrationPoint& b) {
                      return a.speed_m_s < b.speed_m_s;
                  });
    }
}

float VehicleModel::speed_to_pwm(float target_speed_m_s) const {
    if (!std::isfinite(target_speed_m_s) || std::abs(target_speed_m_s) < 0.005f) {
        return 0.0f;
    }

    const float sign = (target_speed_m_s < 0.0f) ? -1.0f : 1.0f;
    const float abs_v = std::abs(target_speed_m_s);

    if (m_calibration_table.empty()) {
        return sign * (abs_v / 0.32f) * 50.0f;
    }

    if (abs_v <= m_calibration_table.front().speed_m_s) {
        return sign * m_deadband_pwm;
    }

    if (abs_v >= m_calibration_table.back().speed_m_s) {
        return sign * m_calibration_table.back().pwm_pct;
    }

    // Interpolation linéaire entre les deux points encadrants
    for (size_t i = 1; i < m_calibration_table.size(); ++i) {
        if (abs_v <= m_calibration_table[i].speed_m_s) {
            float v0 = m_calibration_table[i - 1].speed_m_s;
            float v1 = m_calibration_table[i].speed_m_s;
            float p0 = m_calibration_table[i - 1].pwm_pct;
            float p1 = m_calibration_table[i].pwm_pct;

            float ratio = (abs_v - v0) / std::max(1e-4f, v1 - v0);
            float pwm = p0 + ratio * (p1 - p0);
            return sign * std::clamp(pwm, 0.0f, 100.0f);
        }
    }

    return sign * 100.0f;
}

float VehicleModel::pwm_to_speed(float pwm_pct) const {
    if (!std::isfinite(pwm_pct) || std::abs(pwm_pct) < m_deadband_pwm) {
        return 0.0f;
    }

    const float sign = (pwm_pct < 0.0f) ? -1.0f : 1.0f;
    const float abs_p = std::abs(pwm_pct);

    if (m_calibration_table.empty()) {
        return sign * (abs_p / 50.0f) * 0.32f;
    }

    if (abs_p >= m_calibration_table.back().pwm_pct) {
        return sign * m_calibration_table.back().speed_m_s;
    }

    for (size_t i = 1; i < m_calibration_table.size(); ++i) {
        if (abs_p <= m_calibration_table[i].pwm_pct) {
            float p0 = m_calibration_table[i - 1].pwm_pct;
            float p1 = m_calibration_table[i].pwm_pct;
            float v0 = m_calibration_table[i - 1].speed_m_s;
            float v1 = m_calibration_table[i].speed_m_s;

            float ratio = (abs_p - p0) / std::max(1e-4f, p1 - p0);
            float spd = v0 + ratio * (v1 - v0);
            return sign * std::max(0.0f, spd);
        }
    }

    return sign * m_calibration_table.back().speed_m_s;
}

float VehicleModel::calculate_turn_radius(float steer_deg) const {
    constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
    float abs_steer_rad = std::abs(steer_deg) * DEG_TO_RAD;

    // Angle inférieur à ~1.5 degré : trajectoire assimilée à une ligne droite
    if (abs_steer_rad < 0.026f) {
        return 1000.0f;
    }

    float tan_val = std::tan(abs_steer_rad);
    if (tan_val <= 1e-4f) {
        return 1000.0f;
    }

    float R = m_wheelbase_m / tan_val;
    return std::max(R, MIN_TURN_RADIUS_M);
}

float VehicleModel::calculate_curve_speed_limit(float steer_deg, float max_lat_accel, float safety_factor) const {
    float R = calculate_turn_radius(steer_deg);
    if (R >= 10.0f) {
        return 2.0f; // Pas de limitation de virage en ligne droite
    }
    float v_lim = std::sqrt(std::max(0.0f, max_lat_accel * R));
    return safety_factor * v_lim;
}

} // namespace services
