#include "FSM.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cstdlib>
#include <string>

namespace services {

namespace {
// Variable d'environnement AVOID_SIDE (gauche|droite|left|right) : prioritaire sur AVOID_SIDE du fichier de config
// (utilisee par run_sim.sh --direction). -1 = gauche, +1 = droite.
void apply_avoid_side_env(float& avoid_side) {
    const char* side_env = std::getenv("AVOID_SIDE");
    if (!side_env) return;
    const std::string side(side_env);
    if (side == "gauche" || side == "left" || side == "g" || side == "l") avoid_side = -1.0f;
    else if (side == "droite" || side == "right" || side == "d" || side == "r") avoid_side = 1.0f;
    else std::cerr << "[FSM] AVOID_SIDE='" << side << "' inconnu (gauche|droite) -> valeur du fichier de config conservee" << std::endl;
}
} // namespace

FSM::FSM(int hz)
    : m_running(false),
      m_hz(hz > 0 ? hz : 100),
      m_event_pipe("/tmp/peripherals.sock"),
      m_ipc_handler("fsm.conf", "/tmp/fsm.sock") {}

FSM::~FSM() {
    stop();
}

void FSM::transition_to(shared::fsm::MachineStates new_state) {
    if (m_current_state == new_state) return;
    std::cout << "[FSM] Transition: " << static_cast<int>(m_current_state)
              << " -> " << static_cast<int>(new_state) << std::endl;
    m_current_state = new_state;
}

void FSM::update_controllers() {
    m_navigator.update_controllers(m_line_kp, m_line_kd, m_straight_cruise_speed, m_min_curve_speed, m_max_acceleration);
}

bool FSM::init() {
    std::cout << "[FSM] Initializing Decision & Navigation engine at " << m_hz << " Hz..." << std::endl;

    if (!m_ipc_handler.init(
        [this](const std::string& key, float val) { return apply_config_key(key, val); },
        [this](bool active) {
            std::cout << "[FSM] SetActive command received: " << (active ? "ACTIVE" : "INACTIVE") << std::endl;
            if (active) {
                // Réarmement universel au clic depuis n'importe quel état d'arrêt ou d'erreur
                if (m_current_state == shared::fsm::MachineStates::Stopped ||
                    m_current_state == shared::fsm::MachineStates::Init ||
                    m_current_state == shared::fsm::MachineStates::Failure ||
                    m_current_state == shared::fsm::MachineStates::Abort) {
                    transition_to(shared::fsm::MachineStates::Armed);
                }
            } else {
                transition_to(shared::fsm::MachineStates::Stopped);
            }
            return true;
        },
        [this](uint32_t reason) {
            std::cerr << "[FSM] Emergency Stop command received! Reason: " << reason << std::endl;
            transition_to(shared::fsm::MachineStates::Abort);
            return true;
        }
    )) {
        std::cerr << "[FSM] Failed to initialize IPC handler!" << std::endl;
        return false;
    }

    m_straight_cruise_speed = std::clamp(m_ipc_handler.get_float("STRAIGHT_CRUISE_SPEED", m_straight_cruise_speed),
                                         FsmNavigator::MIN_CRUISE_SPEED, FsmNavigator::MAX_CRUISE_SPEED);
    m_max_power_pct         = std::clamp(m_ipc_handler.get_float("MAX_POWER_PCT", m_max_power_pct),
                                         FsmNavigator::MIN_POWER_PCT, FsmNavigator::MAX_POWER_PCT);
    std::cout << "[FSM] Puissance moteur a la croisiere : " << m_max_power_pct << " % (croisiere " << m_straight_cruise_speed
              << ")" << std::endl;
    m_min_curve_speed       = m_ipc_handler.get_float("MIN_CURVE_SPEED", m_min_curve_speed);
    m_curve_safety_factor   = m_ipc_handler.get_float("CURVE_SAFETY_FACTOR", m_curve_safety_factor);
    m_max_acceleration      = m_ipc_handler.get_float("MAX_ACCELERATION", m_max_acceleration);
    m_line_kp               = m_ipc_handler.get_float("LINE_KP", m_line_kp);
    m_line_kd               = m_ipc_handler.get_float("LINE_KD", m_line_kd);
    m_obstacle_stop_dist_cm = m_ipc_handler.get_float("OBSTACLE_STOP_DIST_CM", m_obstacle_stop_dist_cm);
    m_reverse_speed         = m_ipc_handler.get_float("REVERSE_SPEED", m_reverse_speed);
    m_avoid_side            = m_ipc_handler.get_float("AVOID_SIDE", m_avoid_side);
    apply_avoid_side_env(m_avoid_side);
    m_avoid_diag_time_s = std::clamp(m_ipc_handler.get_float("AVOID_DIAG_TIME_S", m_avoid_diag_time_s),
                                     ObstacleAvoidance::MIN_DIAG_TIME_S, ObstacleAvoidance::MAX_DIAG_TIME_S);
    m_avoid_diag_speed  = std::clamp(m_ipc_handler.get_float("AVOID_DIAG_SPEED", m_avoid_diag_speed),
                                     ObstacleAvoidance::MIN_DIAG_SPEED, ObstacleAvoidance::MAX_DIAG_SPEED);
    m_navigator.set_avoidance_diagonal(m_avoid_diag_time_s, m_avoid_diag_speed);
    std::cout << "[FSM] Evitement, diagonale de la phase 2 : " << m_avoid_diag_speed << " m/s pendant "
              << m_avoid_diag_time_s << " s (~" << m_avoid_diag_speed * m_avoid_diag_time_s * 100.0f << " cm)" << std::endl;
    m_avoid_return_time_s = std::clamp(m_ipc_handler.get_float("AVOID_RETURN_TIME_S", m_avoid_return_time_s),
                                       ObstacleAvoidance::MIN_RETURN_TIME_S, ObstacleAvoidance::MAX_RETURN_TIME_S);
    m_avoid_return_speed  = std::clamp(m_ipc_handler.get_float("AVOID_RETURN_SPEED", m_avoid_return_speed),
                                       ObstacleAvoidance::MIN_RETURN_SPEED, ObstacleAvoidance::MAX_RETURN_SPEED);
    m_navigator.set_avoidance_return(m_avoid_return_time_s, m_avoid_return_speed);
    m_avoid_turn_angle_deg  = std::clamp(m_ipc_handler.get_float("AVOID_TURN_ANGLE_DEG", m_avoid_turn_angle_deg),
                                         ObstacleAvoidance::MIN_TURN_ANGLE_DEG, ObstacleAvoidance::MAX_TURN_ANGLE_DEG);
    m_avoid_turn_out_time_s = std::clamp(m_ipc_handler.get_float("AVOID_TURN_OUT_TIME_S", m_avoid_turn_out_time_s),
                                         ObstacleAvoidance::MIN_TURN_OUT_TIME_S, ObstacleAvoidance::MAX_TURN_OUT_TIME_S);
    m_avoid_turn_time_s     = std::clamp(m_ipc_handler.get_float("AVOID_TURN_TIME_S", m_avoid_turn_time_s),
                                         ObstacleAvoidance::MIN_TURN_TIME_S, ObstacleAvoidance::MAX_TURN_TIME_S);
    m_navigator.set_avoidance_turn(m_avoid_turn_angle_deg, m_avoid_turn_out_time_s, m_avoid_turn_time_s);
    std::cout << "[FSM] Evitement, virages : roues a " << m_avoid_turn_angle_deg << " deg pendant " << m_avoid_turn_out_time_s
              << " s (1er virage) puis " << m_avoid_turn_time_s << " s" << std::endl;
    std::cout << "[FSM] Evitement, diagonale de retour (phase 5) : " << m_avoid_return_speed << " m/s, "
              << m_avoid_return_time_s << " s maximum (~" << m_avoid_return_speed * m_avoid_return_time_s * 100.0f
              << " cm au plus)" << std::endl;
    std::cout << "[FSM] Contournement des obstacles par la " << (m_avoid_side < 0.0f ? "GAUCHE" : "DROITE") << std::endl;
    m_hz = std::clamp(static_cast<int>(m_ipc_handler.get_float("HZ", static_cast<float>(m_hz))), 10, 10000);

    m_navigator.init(m_line_kp, m_line_kd, m_straight_cruise_speed, m_min_curve_speed, m_max_acceleration);

    if (!m_shm_bridge.init(shared::fsm::MachineStates::Init)) {
        std::cerr << "[FSM] Failed to initialize SHM bridge!" << std::endl;
        return false;
    }

    transition_to(shared::fsm::MachineStates::Init);
    m_running.store(true);
    return true;
}

void FSM::run() {
    const auto period = std::chrono::nanoseconds(1000000000LL / m_hz);
    auto next_deadline = std::chrono::steady_clock::now();
    const float dt = 1.0f / static_cast<float>(m_hz);
    const int max_watchdog_cycles = (m_hz * 250) / 1000;
    uint64_t loop_count = 0;

    std::cout << "[FSM] Navigation & Avoidance loop started at " << m_hz << " Hz." << std::endl;

    while (m_running.load(std::memory_order_relaxed)) {
        next_deadline += period;

        // 1. Process non-blocking IPC commands (/tmp/fsm.sock)
        m_ipc_handler.process_pending_commands();

        // 2. Read sensors from SHM
        float current_dist_cm = 100.0f;
        uint8_t line_sensors[5] = {0, 0, 0, 0, 0};
        float line_darkness[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        bool shm_emergency = false;
        bool sensors_alive = true;
        m_shm_bridge.read_sensors(current_dist_cm, line_sensors, line_darkness, shm_emergency, sensors_alive);

        // 4. Watchdog: lost sensors heartbeat
        const bool is_active_nav_state = (
            m_current_state != shared::fsm::MachineStates::Init &&
            m_current_state != shared::fsm::MachineStates::Unarmed &&
            m_current_state != shared::fsm::MachineStates::Armed &&
            m_current_state != shared::fsm::MachineStates::Stopped &&
            m_current_state != shared::fsm::MachineStates::Abort &&
            m_current_state != shared::fsm::MachineStates::Failure
        );

        if (is_active_nav_state) {
            if (m_shm_bridge.check_watchdog(max_watchdog_cycles)) {
                if (m_current_state != shared::fsm::MachineStates::Failure) {
                    std::cerr << "[FSM] Watchdog: Sensor heartbeat timeout (250ms) -> Failure" << std::endl;
                    transition_to(shared::fsm::MachineStates::Failure);
                }
            }
        } else {
            m_shm_bridge.reset_watchdog();
        }

        if (shm_emergency && m_current_state != shared::fsm::MachineStates::Failure) {
            transition_to(shared::fsm::MachineStates::Failure);
        }

        // 5. Navigation & State Machine Step
        bool start_triggered = m_event_pipe.check_start_trigger();

        FsmNavOutput nav_out = m_navigator.step(m_current_state, dt, current_dist_cm, line_darkness,
                                                start_triggered, m_avoid_side, m_obstacle_stop_dist_cm,
                                                m_reverse_speed, m_curve_safety_factor);

        if (nav_out.state_changed) {
            transition_to(nav_out.next_state);
        }

        // 6. Smooth velocity profiling & power computation
        float bounded_velocity = m_navigator.compute_bounded_velocity(nav_out.desired_velocity, dt);
        // L'evitement d'obstacle garde la puissance de reference (ses durees sont calibrees pour elle) ; le reste suit MAX_POWER_PCT
        const bool avoiding = (m_current_state == shared::fsm::MachineStates::RightDeviation ||
                               m_current_state == shared::fsm::MachineStates::LeftDeviation);
        float motor_power_pct = m_navigator.compute_motor_power(bounded_velocity, m_straight_cruise_speed,
                                                                avoiding ? FsmNavigator::REFERENCE_POWER_PCT : m_max_power_pct);
        if (nav_out.req_emergency ||
            m_current_state == shared::fsm::MachineStates::Stopped ||
            m_current_state == shared::fsm::MachineStates::Failure ||
            m_current_state == shared::fsm::MachineStates::Abort) {
            motor_power_pct = 0.0f;
        }

        // 7. Publish targets to SHM
        m_shm_bridge.publish_control_commands(motor_power_pct, nav_out.desired_steering,
                                              nav_out.req_emergency, m_current_state);

        // 8. 1 Hz periodic log
        loop_count++;
        if (loop_count % m_hz == 0) {
            std::cout << "[FSM " << m_hz << "Hz] State: " << static_cast<int>(m_current_state)
                      << " | Vel: " << bounded_velocity << " m/s (" << motor_power_pct << "%)"
                      << " | Steer: " << nav_out.desired_steering << " deg | Dist: " << current_dist_cm << " cm" << std::endl;
        }

        // 9. Deterministic anti-drift deadline pacing
        auto now = std::chrono::steady_clock::now();
        if (now < next_deadline) {
            std::this_thread::sleep_until(next_deadline);
        } else {
            next_deadline = now;
        }
    }

    m_shm_bridge.emergency_stop_shm();
    std::cout << "[FSM] Service loop terminated." << std::endl;
}

void FSM::stop() {
    if (m_running.load()) {
        std::cout << "[FSM] Stopping service..." << std::endl;
        m_running.store(false);
        m_shm_bridge.emergency_stop_shm();
    }
}

bool FSM::apply_config_key(const std::string& key, float value) {
    if (key == "LINE_KP") {
        m_line_kp = value;
        m_ipc_handler.set_float("LINE_KP", value);
        update_controllers();
        return true;
    }
    if (key == "LINE_KD") {
        m_line_kd = value;
        m_ipc_handler.set_float("LINE_KD", value);
        update_controllers();
        return true;
    }
    if (key == "MAX_POWER_PCT") {
        m_max_power_pct = std::clamp(value, FsmNavigator::MIN_POWER_PCT, FsmNavigator::MAX_POWER_PCT);
        m_ipc_handler.set_float("MAX_POWER_PCT", m_max_power_pct);
        return true;
    }
    if (key == "STRAIGHT_CRUISE_SPEED") {
        m_straight_cruise_speed = std::clamp(value, FsmNavigator::MIN_CRUISE_SPEED, FsmNavigator::MAX_CRUISE_SPEED);
        m_ipc_handler.set_float("STRAIGHT_CRUISE_SPEED", m_straight_cruise_speed);
        update_controllers();
        return true;
    }
    if (key == "MIN_CURVE_SPEED") {
        m_min_curve_speed = value;
        m_ipc_handler.set_float("MIN_CURVE_SPEED", value);
        update_controllers();
        return true;
    }
    if (key == "MAX_ACCELERATION") {
        m_max_acceleration = value;
        m_ipc_handler.set_float("MAX_ACCELERATION", value);
        update_controllers();
        return true;
    }
    if (key == "CURVE_SAFETY_FACTOR") {
        m_curve_safety_factor = std::clamp(value, 0.1f, 2.0f);
        m_ipc_handler.set_float("CURVE_SAFETY_FACTOR", m_curve_safety_factor);
        return true;
    }
    if (key == "OBSTACLE_STOP_DIST_CM") {
        m_obstacle_stop_dist_cm = std::max(value, 5.0f);
        m_ipc_handler.set_float("OBSTACLE_STOP_DIST_CM", m_obstacle_stop_dist_cm);
        return true;
    }
    if (key == "REVERSE_SPEED") {
        m_reverse_speed = std::clamp(value, 0.01f, 1.0f);
        m_ipc_handler.set_float("REVERSE_SPEED", m_reverse_speed);
        return true;
    }
    if (key == "AVOID_SIDE") {
        m_avoid_side = (value < 0.0f) ? -1.0f : 1.0f;
        m_ipc_handler.set_float("AVOID_SIDE", m_avoid_side);
        return true;
    }
    if (key == "AVOID_DIAG_TIME_S") {
        m_avoid_diag_time_s = std::clamp(value, ObstacleAvoidance::MIN_DIAG_TIME_S, ObstacleAvoidance::MAX_DIAG_TIME_S);
        m_ipc_handler.set_float("AVOID_DIAG_TIME_S", m_avoid_diag_time_s);
        m_navigator.set_avoidance_diagonal(m_avoid_diag_time_s, m_avoid_diag_speed);
        return true;
    }
    if (key == "AVOID_DIAG_SPEED") {
        m_avoid_diag_speed = std::clamp(value, ObstacleAvoidance::MIN_DIAG_SPEED, ObstacleAvoidance::MAX_DIAG_SPEED);
        m_ipc_handler.set_float("AVOID_DIAG_SPEED", m_avoid_diag_speed);
        m_navigator.set_avoidance_diagonal(m_avoid_diag_time_s, m_avoid_diag_speed);
        return true;
    }
    if (key == "AVOID_TURN_ANGLE_DEG") {
        m_avoid_turn_angle_deg = std::clamp(value, ObstacleAvoidance::MIN_TURN_ANGLE_DEG, ObstacleAvoidance::MAX_TURN_ANGLE_DEG);
        m_ipc_handler.set_float("AVOID_TURN_ANGLE_DEG", m_avoid_turn_angle_deg);
        m_navigator.set_avoidance_turn(m_avoid_turn_angle_deg, m_avoid_turn_out_time_s, m_avoid_turn_time_s);
        return true;
    }
    if (key == "AVOID_TURN_OUT_TIME_S") {
        m_avoid_turn_out_time_s = std::clamp(value, ObstacleAvoidance::MIN_TURN_OUT_TIME_S, ObstacleAvoidance::MAX_TURN_OUT_TIME_S);
        m_ipc_handler.set_float("AVOID_TURN_OUT_TIME_S", m_avoid_turn_out_time_s);
        m_navigator.set_avoidance_turn(m_avoid_turn_angle_deg, m_avoid_turn_out_time_s, m_avoid_turn_time_s);
        return true;
    }
    if (key == "AVOID_TURN_TIME_S") {
        m_avoid_turn_time_s = std::clamp(value, ObstacleAvoidance::MIN_TURN_TIME_S, ObstacleAvoidance::MAX_TURN_TIME_S);
        m_ipc_handler.set_float("AVOID_TURN_TIME_S", m_avoid_turn_time_s);
        m_navigator.set_avoidance_turn(m_avoid_turn_angle_deg, m_avoid_turn_out_time_s, m_avoid_turn_time_s);
        return true;
    }
    if (key == "AVOID_RETURN_TIME_S") {
        m_avoid_return_time_s = std::clamp(value, ObstacleAvoidance::MIN_RETURN_TIME_S, ObstacleAvoidance::MAX_RETURN_TIME_S);
        m_ipc_handler.set_float("AVOID_RETURN_TIME_S", m_avoid_return_time_s);
        m_navigator.set_avoidance_return(m_avoid_return_time_s, m_avoid_return_speed);
        return true;
    }
    if (key == "AVOID_RETURN_SPEED") {
        m_avoid_return_speed = std::clamp(value, ObstacleAvoidance::MIN_RETURN_SPEED, ObstacleAvoidance::MAX_RETURN_SPEED);
        m_ipc_handler.set_float("AVOID_RETURN_SPEED", m_avoid_return_speed);
        m_navigator.set_avoidance_return(m_avoid_return_time_s, m_avoid_return_speed);
        return true;
    }
    if (key == "HZ") {
        m_hz = std::clamp(static_cast<int>(value), 10, 10000);
        m_ipc_handler.set_float("HZ", static_cast<float>(m_hz));
        return true;
    }
    return false;
}

} // namespace services
