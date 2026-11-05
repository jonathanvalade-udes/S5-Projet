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

    void reset_controllers();
    float get_current_speed() const;
    float compute_bounded_velocity(float desired_v, float dt);
    float compute_motor_power(float bounded_v, float cruise_speed);

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
