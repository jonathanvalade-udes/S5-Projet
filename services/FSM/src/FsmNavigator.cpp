#include "FsmNavigator.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace services {

FsmNavigator::FsmNavigator() = default;

void FsmNavigator::init(float kp, float kd, float cruise_speed, float min_curve_speed, float max_accel) {
    m_cruise_speed = cruise_speed;
    m_min_curve_speed = min_curve_speed;
    m_line_controller = std::make_unique<LineFollowerController>(kp, kd, cruise_speed, min_curve_speed);
    m_motion_profiler = std::make_unique<MotionProfiler>(max_accel, cruise_speed);
}

void FsmNavigator::update_controllers(float kp, float kd, float cruise_speed, float min_curve_speed, float max_accel) {
    m_cruise_speed = cruise_speed;
    m_min_curve_speed = min_curve_speed;
    if (m_motion_profiler) {
        m_motion_profiler->set_max_accel(max_accel);
        m_motion_profiler->set_max_speed(cruise_speed);
    }
    if (m_line_controller) {
        m_line_controller->set_kp(kp);
        m_line_controller->set_kd(kd);
        m_line_controller->set_cruise_speed(cruise_speed);
        m_line_controller->set_min_curve_speed(min_curve_speed);
    }
}

void FsmNavigator::reset_controllers() {
    if (m_line_controller) m_line_controller->reset();
    if (m_motion_profiler) m_motion_profiler->reset(0.0f);
    m_state_timer = 0.0f;
    m_state_distance = 0.0f;
    m_course_finished = false;
}

float FsmNavigator::get_current_speed() const {
    return m_motion_profiler ? m_motion_profiler->get_current_speed() : 0.0f;
}

float FsmNavigator::compute_bounded_velocity(float desired_v, float dt) {
    return m_motion_profiler ? m_motion_profiler->update(desired_v, dt) : desired_v;
}

float FsmNavigator::compute_motor_power(float bounded_v, float cruise_speed) {
    if (cruise_speed <= 0.01f) return 0.0f;
    float pct = (bounded_v / cruise_speed) * 50.0f;
    return std::clamp(pct, -100.0f, 100.0f);
}

FsmNavOutput FsmNavigator::step(shared::fsm::MachineStates current_state, float dt,
                               float current_dist_cm, const float line_darkness[5],
                               bool start_triggered, float avoid_side, float obstacle_stop_dist_cm,
                               float reverse_speed, float curve_safety_factor, float predictive_brake_gain) {
    FsmNavOutput out{};
    float curr_v = get_current_speed();
    float cruise_spd = m_cruise_speed;
    float min_curve_spd = m_min_curve_speed;

    auto set_transition = [&](shared::fsm::MachineStates next) {
        out.state_changed = true;
        out.next_state = next;
        m_state_timer = 0.0f;
        m_state_distance = 0.0f;
    };

    switch (current_state) {
        case shared::fsm::MachineStates::Init:
            m_state_timer += dt;
            if (m_state_timer >= 0.1f) set_transition(shared::fsm::MachineStates::Unarmed);
            break;

        case shared::fsm::MachineStates::Unarmed:
            m_state_timer += dt;
            if (m_state_timer >= 0.4f) set_transition(shared::fsm::MachineStates::Armed);
            break;

        case shared::fsm::MachineStates::Armed:
            m_state_timer += dt;
            if (start_triggered) {
                reset_controllers();
                set_transition(shared::fsm::MachineStates::LineFollowingForward);
            }
            break;

        case shared::fsm::MachineStates::LineFollowingForward: {
            m_obstacle_avoidance.update_ignore_finish(dt);
            if (m_line_controller) {
                m_line_controller->set_finish_detection(!m_obstacle_avoidance.is_finish_ignored());
            }

            LineFollowerOutput line_out = m_line_controller ? m_line_controller->update_analog(line_darkness, dt) : LineFollowerOutput{};
            out.desired_steering = line_out.steer_degrees;

            if (line_out.is_finished) {
                m_course_finished = true;
                set_transition(shared::fsm::MachineStates::Stopped);
                break;
            }
            if (line_out.is_line_lost) {
                set_transition(shared::fsm::MachineStates::Reverse);
                break;
            }
            if (current_dist_cm <= obstacle_stop_dist_cm) {
                set_transition(shared::fsm::MachineStates::CollisionRisk);
                break;
            }

            float adapt_v = m_motion_profiler
                ? m_motion_profiler->calculate_curve_speed_ackermann(out.desired_steering, cruise_spd, min_curve_spd, curve_safety_factor)
                : cruise_spd;

            if (m_motion_profiler && predictive_brake_gain > 0.0f) {
                adapt_v = m_motion_profiler->apply_predictive_brake(line_out.d_error_dt, adapt_v, predictive_brake_gain);
            }
            out.desired_velocity = adapt_v;
            break;
        }

        case shared::fsm::MachineStates::CollisionRisk:
            if (m_motion_profiler && m_motion_profiler->get_current_speed() < 0.03f) {
                set_transition(shared::fsm::MachineStates::Stopped);
            }
            break;

        case shared::fsm::MachineStates::Stopped:
            m_state_timer += dt;
            if (m_course_finished) {
                if (m_state_timer >= 0.5f) {
                    m_course_finished = false;
                    set_transition(shared::fsm::MachineStates::Armed);
                }
                break;
            }
            if (m_state_timer >= 0.2f) {
                if (current_dist_cm <= 25.0f) {
                    set_transition(avoid_side < 0.0f ? shared::fsm::MachineStates::LeftDeviation : shared::fsm::MachineStates::RightDeviation);
                    float trim = m_line_controller ? m_line_controller->steering_trim() * 45.0f : 0.0f;
                    m_obstacle_avoidance.reset(trim, current_dist_cm / 100.0f);
                    m_obstacle_avoidance.set_ignore_finish(1.0f);
                } else {
                    set_transition(shared::fsm::MachineStates::LineFollowingForward);
                }
            }
            break;

        case shared::fsm::MachineStates::RightDeviation:
        case shared::fsm::MachineStates::LeftDeviation: {
            float side = (current_state == shared::fsm::MachineStates::RightDeviation) ? 1.0f : -1.0f;
            // Vitesse RÉELLE : la vitesse du FSM est convertie en puissance moteur avec la croisière = 50 %, donc elle ne vaut pas
            // des m/s réels (sur le robot, 50 % de puissance = ~0,32 m/s, que la croisière configurée soit 0,35 ou 0,45).
            const float odo_scale = picar::constants::PHYSICAL_SPEED_AT_HALF_THROTTLE / std::max(m_cruise_speed, 0.05f);
            auto res = m_obstacle_avoidance.update(side, dt, line_darkness, curr_v * odo_scale, min_curve_spd, out.desired_steering, out.desired_velocity);
            if (res == ObstacleAvoidance::DeviationResult::LineFound) {
                set_transition(shared::fsm::MachineStates::LineDetected);
            } else if (res == ObstacleAvoidance::DeviationResult::SearchLine) {
                set_transition(shared::fsm::MachineStates::LineSearching);
            }
            break;
        }

        case shared::fsm::MachineStates::LineSearching: {
            m_state_timer += dt;
            out.desired_steering = -24.0f;
            out.desired_velocity = 0.15f;
            float max_d = std::max({line_darkness[0], line_darkness[1], line_darkness[2], line_darkness[3], line_darkness[4]});
            if (max_d >= 0.55f && m_state_timer >= 0.25f) {
                set_transition(shared::fsm::MachineStates::LineDetected);
            } else if (m_state_timer >= 4.0f) {
                set_transition(shared::fsm::MachineStates::LineSearchingTimedOut);
                out.req_emergency = true;
            }
            break;
        }

        case shared::fsm::MachineStates::LineDetected:
            m_state_timer += dt;
            out.desired_velocity = min_curve_spd;
            if (m_state_timer >= 0.2f) set_transition(shared::fsm::MachineStates::LineFollowingForward);
            break;

        case shared::fsm::MachineStates::Reverse: {
            m_state_timer += dt;
            out.desired_velocity = -reverse_speed;
            m_state_distance += std::abs(curr_v) * dt;
            bool found = (line_darkness[0] > 0.4f || line_darkness[1] > 0.4f || line_darkness[2] > 0.4f ||
                          line_darkness[3] > 0.4f || line_darkness[4] > 0.4f);
            if (found && m_state_timer >= 0.2f) {
                set_transition(shared::fsm::MachineStates::LineFollowingForward);
            } else if (m_state_distance >= 0.15f || m_state_timer >= 3.0f) {
                set_transition(shared::fsm::MachineStates::Failure);
            }
            break;
        }

        case shared::fsm::MachineStates::CollisionOccured:
        case shared::fsm::MachineStates::Abort:
        case shared::fsm::MachineStates::LineSearchingTimedOut:
        case shared::fsm::MachineStates::Failure:
        default:
            out.desired_velocity = 0.0f;
            out.desired_steering = 0.0f;
            out.req_emergency = true;
            break;
    }

    return out;
}

} // namespace services
