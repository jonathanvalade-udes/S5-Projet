#pragma once

#include <atomic>
#include <string>
#include "fsm/machine_component.hpp"
#include "control/picar_constants.hpp"
#include "FsmNavigator.hpp"
#include "FsmEventPipe.hpp"
#include "FsmIpcHandler.hpp"
#include "FsmShmBridge.hpp"

namespace services {

class FSM {
public:
    explicit FSM(int hz = 100);
    ~FSM();

    bool init();
    void run();
    void stop();

    bool is_running() const { return m_running.load(); }
    shared::fsm::MachineStates get_current_state() const { return m_current_state; }
    void set_state(shared::fsm::MachineStates state) { transition_to(state); }

private:
    void transition_to(shared::fsm::MachineStates new_state);

    std::atomic<bool> m_running{false};
    int m_hz{100};
    shared::fsm::MachineStates m_current_state{shared::fsm::MachineStates::Init};

    // Kinematic & control parameters
    float m_straight_cruise_speed{picar::constants::DEFAULT_STRAIGHT_CRUISE_SPEED};
    float m_min_curve_speed{picar::constants::DEFAULT_MIN_CURVE_SPEED};
    float m_curve_safety_factor{picar::constants::DEFAULT_CURVE_SAFETY_FACTOR};
    float m_max_acceleration{picar::constants::DEFAULT_MAX_ACCELERATION};
    float m_line_kp{picar::constants::DEFAULT_LINE_KP};
    float m_line_kd{picar::constants::DEFAULT_LINE_KD};
    float m_obstacle_stop_dist_cm{picar::constants::DEFAULT_OBSTACLE_STOP_DIST_CM};
    float m_reverse_speed{picar::constants::DEFAULT_REVERSE_SPEED};
    float m_avoid_side{1.0f};
    float m_max_power_pct{FsmNavigator::REFERENCE_POWER_PCT};   // puissance moteur (%) quand la vitesse atteint la croisiere
    float m_avoid_diag_time_s{ObstacleAvoidance::DEFAULT_DIAG_TIME_S};   // evitement, phase 2 : duree de la diagonale (s)
    float m_avoid_diag_speed{ObstacleAvoidance::DEFAULT_DIAG_SPEED};     // evitement, phase 2 : vitesse de la diagonale (m/s)
    float m_avoid_return_time_s{ObstacleAvoidance::DEFAULT_RETURN_TIME_S};   // evitement, phase 5 : duree max de la diagonale de retour (s)
    float m_avoid_return_speed{ObstacleAvoidance::DEFAULT_RETURN_SPEED};     // evitement, phase 5 : vitesse de la diagonale de retour (m/s)
    float m_avoid_turn_angle_deg{ObstacleAvoidance::DEFAULT_TURN_ANGLE_DEG};     // evitement : braquage des virages (deg)
    float m_avoid_turn_out_time_s{ObstacleAvoidance::DEFAULT_TURN_OUT_TIME_S};   // evitement, phase 2 : duree du 1er virage (s)
    float m_avoid_turn_time_s{ObstacleAvoidance::DEFAULT_TURN_TIME_S};           // evitement, phases 3 et 4 : duree des virages (s)

    FsmNavigator m_navigator;
    FsmEventPipe m_event_pipe;
    FsmIpcHandler m_ipc_handler;
    FsmShmBridge m_shm_bridge;

    bool apply_config_key(const std::string& key, float value);
    void update_controllers();
};

} // namespace services
