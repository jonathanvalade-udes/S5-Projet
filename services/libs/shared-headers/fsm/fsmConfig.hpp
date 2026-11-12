#pragma once

namespace shared::fsm {

struct FSMAvoidanceConfig {
    float reverse_dist_m;
    float angle_1_deg;
    float time_1_s;
    float diag_time_s;
    float diag_speed;
    float angle_2_deg;
    float time_2_s;
    float parallel_time_s;
    float angle_3_deg;
    float time_3_s;
    float return_time_s;
    float return_speed;
};

struct FSMConfig {
    float hz;
    float max_acceleration;
    float straight_cruise_speed;
    float max_power_pct;
    float min_curve_speed;
    float curve_safety_factor;
    float line_kp;
    float line_kd;
    float obstacle_stop_dist_cm;
    float reverse_speed;
    float avoid_side;

    FSMAvoidanceConfig avoidance;
};

} // namespace shared::fsm
