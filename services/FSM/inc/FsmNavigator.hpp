#pragma once

#include <memory>
#include "fsm/machine_component.hpp"
#include "control/line_follower.hpp"
#include "control/motion_profiler.hpp"
#include "control/picar_constants.hpp"
#include "ObstacleAvoidance.hpp"

namespace services {

struct FsmNavOutput {
    float desired_velocity{0.0f};
    float desired_steering{0.0f};
    bool req_emergency{false};
    bool state_changed{false};
    shared::fsm::MachineStates next_state{shared::fsm::MachineStates::Init};
};

class FsmNavigator {
public:
    FsmNavigator();
    ~FsmNavigator() = default;

    void init(float kp, float kd, float cruise_speed, float min_curve_speed, float max_accel);
    void update_controllers(float kp, float kd, float cruise_speed, float min_curve_speed, float max_accel);

    FsmNavOutput step(shared::fsm::MachineStates current_state, float dt,
                      float current_dist_cm, const float line_darkness[5],
                      bool start_triggered, float avoid_side, float obstacle_stop_dist_cm,
                      float reverse_speed, float curve_safety_factor);

    /// Diagonale de la phase 2 de l'evitement : duree (s) et vitesse (m/s), lues dans fsm.conf.
    void set_avoidance_diagonal(float time_s, float speed) { m_obstacle_avoidance.set_diagonal(time_s, speed); }

    /// Deuxieme diagonale de l'evitement (phase 5) : duree maximale (s) et vitesse (m/s), lues dans fsm.conf.
    void set_avoidance_return(float time_s, float speed) { m_obstacle_avoidance.set_return(time_s, speed); }

    /// Virages de l'evitement : angle de braquage (deg), duree du 1er virage et des virages suivants (s), lus dans fsm.conf.
    void set_avoidance_turn(float angle_deg, float out_time_s, float time_s) {
        m_obstacle_avoidance.set_turn(angle_deg, out_time_s, time_s);
    }

    void reset_controllers();
    float get_current_speed() const;
    float compute_bounded_velocity(float desired_v, float dt);
    /// Puissance moteur (%) pour une vitesse donnee : vitesse / croisiere x power_at_cruise_pct.
    float compute_motor_power(float bounded_v, float cruise_speed, float power_at_cruise_pct = REFERENCE_POWER_PCT);

    /// Puissance (%) a la vitesse de croisiere. 50 = reference validee (environ 0,32 m/s) sur laquelle l'evitement est calibre.
    static constexpr float REFERENCE_POWER_PCT = 50.0f;
    /// Bornes de MAX_POWER_PCT (puissance a la croisiere) et de STRAIGHT_CRUISE_SPEED.
    static constexpr float MIN_POWER_PCT = 20.0f;
    static constexpr float MAX_POWER_PCT = 75.0f;
    static constexpr float MIN_CRUISE_SPEED = 0.10f;
    static constexpr float MAX_CRUISE_SPEED = 0.60f;

    /// Distance (cm) sous laquelle la voiture commence a ralentir avant l'obstacle.
    static constexpr float OBSTACLE_SLOWDOWN_THRESHOLD_CM = 45.0f;

    /// Plafonne la vitesse pour pouvoir s'arreter a obstacle_stop_dist_cm sans depasser a_max (v = sqrt(2*a*d)).
    float limit_approach_speed(float desired_v, float current_dist_cm, float obstacle_stop_dist_cm) const;

private:
    std::unique_ptr<LineFollowerController> m_line_controller;
    std::unique_ptr<MotionProfiler> m_motion_profiler;
    ObstacleAvoidance m_obstacle_avoidance;

    float m_cruise_speed{0.35f};
    float m_min_curve_speed{0.17f};
    float m_last_steer_degrees{0.0f};
    float m_state_timer{0.0f};
    float m_state_distance{0.0f};
    bool m_course_finished{false};
};

} // namespace services
