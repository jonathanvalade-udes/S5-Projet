#include "FSM.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

namespace services {

FSM::FSM(int hz)
    : m_running(false),
      m_hz(hz > 0 ? hz : 100000),
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

    if (!m_ipc_handler.init([this](const std::string& key, float val) {
        return apply_config_key(key, val);
    })) {
        std::cerr << "[FSM] Failed to initialize IPC handler!" << std::endl;
        return false;
    }

    m_straight_cruise_speed = m_ipc_handler.get_float("STRAIGHT_CRUISE_SPEED", m_straight_cruise_speed);
    m_min_curve_speed       = m_ipc_handler.get_float("MIN_CURVE_SPEED", m_min_curve_speed);
    m_curve_safety_factor   = m_ipc_handler.get_float("CURVE_SAFETY_FACTOR", m_curve_safety_factor);
    m_max_acceleration      = m_ipc_handler.get_float("MAX_ACCELERATION", m_max_acceleration);
    m_line_kp               = m_ipc_handler.get_float("LINE_KP", m_line_kp);
    m_line_kd               = m_ipc_handler.get_float("LINE_KD", m_line_kd);
    m_obstacle_stop_dist_cm = m_ipc_handler.get_float("OBSTACLE_STOP_DIST_CM", m_obstacle_stop_dist_cm);
    m_reverse_speed         = m_ipc_handler.get_float("REVERSE_SPEED", m_reverse_speed);
    m_auto_start_delay      = m_ipc_handler.get_float("AUTO_START_DELAY", m_auto_start_delay);
    m_avoid_side            = m_ipc_handler.get_float("AVOID_SIDE", m_avoid_side);
    m_predictive_brake_gain = m_ipc_handler.get_float("PREDICTIVE_BRAKE_GAIN", m_predictive_brake_gain);
    m_hz = std::clamp(static_cast<int>(m_ipc_handler.get_float("HZ", static_cast<float>(m_hz))), 10, 100000);

    m_navigator.init(m_line_kp, m_line_kd, m_straight_cruise_speed, m_min_curve_speed, m_max_acceleration);

    if (!m_shm_bridge.init(shared::fsm::MachineStates::Init)) {
        std::cerr << "[FSM] Failed to initialize SHM bridge!" << std::endl;
        return false;
    }

    m_shm_bridge.sync_config_to_shm(m_max_acceleration, m_straight_cruise_speed, m_min_curve_speed,
                                    m_curve_safety_factor, m_predictive_brake_gain, m_line_kp, m_line_kd);

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

        // 2. Dynamic config sync from SHM
        if (m_shm_bridge.check_config_from_shm(m_max_acceleration, m_straight_cruise_speed, m_min_curve_speed,
                                               m_curve_safety_factor, m_predictive_brake_gain, m_line_kp, m_line_kd)) {
            update_controllers();
        }

        // 3. Read sensors from SHM
        float current_dist_cm = 100.0f;
        uint8_t line_sensors[5] = {0, 0, 0, 0, 0};
        float line_darkness[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        bool shm_emergency = false;
        bool sensors_alive = true;
        m_shm_bridge.read_sensors(current_dist_cm, line_sensors, line_darkness, shm_emergency, sensors_alive);

        // 4. Watchdog: lost sensors heartbeat
        if (m_current_state != shared::fsm::MachineStates::Init && m_current_state != shared::fsm::MachineStates::Stopped) {
            if (m_shm_bridge.check_watchdog(max_watchdog_cycles)) {
                if (m_current_state != shared::fsm::MachineStates::Failure) {
                    std::cerr << "[FSM] Watchdog: Sensor heartbeat timeout (250ms) -> Failure" << std::endl;
                    transition_to(shared::fsm::MachineStates::Failure);
                }
            }
        }

        if (shm_emergency && m_current_state != shared::fsm::MachineStates::Failure) {
            transition_to(shared::fsm::MachineStates::Failure);
        }

        // 5. Navigation & State Machine Step
        bool start_triggered = m_event_pipe.check_start_trigger() ||
            (m_auto_start_delay > 0.0f && loop_count >= static_cast<uint64_t>(m_auto_start_delay * m_hz));

        FsmNavOutput nav_out = m_navigator.step(m_current_state, dt, current_dist_cm, line_darkness,
                                                start_triggered, m_avoid_side, m_obstacle_stop_dist_cm,
                                                m_reverse_speed, m_curve_safety_factor, m_predictive_brake_gain);

        if (nav_out.state_changed) {
            transition_to(nav_out.next_state);
        }

        // 6. Smooth velocity profiling & power computation
        float bounded_velocity = m_navigator.compute_bounded_velocity(nav_out.desired_velocity, dt);
        float motor_power_pct = m_navigator.compute_motor_power(bounded_velocity, m_straight_cruise_speed);
        if (nav_out.req_emergency || m_current_state == shared::fsm::MachineStates::Stopped || m_current_state == shared::fsm::MachineStates::Failure) {
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
    if (key == "STRAIGHT_CRUISE_SPEED") {
        m_straight_cruise_speed = value;
        m_ipc_handler.set_float("STRAIGHT_CRUISE_SPEED", value);
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
    if (key == "HZ") {
        m_hz = std::clamp(static_cast<int>(value), 10, 100000);
        m_ipc_handler.set_float("HZ", static_cast<float>(m_hz));
        return true;
    }
    return false;
}

} // namespace services
