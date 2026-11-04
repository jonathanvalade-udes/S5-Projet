#include "FsmShmBridge.hpp"
#include <iostream>

namespace services {

bool FsmShmBridge::init(shared::fsm::MachineStates initial_state) {
    try {
        m_shm_mgr = std::make_unique<SharedMemoryManager>();
        m_telemetry = m_shm_mgr->open_shm(true);
        if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            m_telemetry->emergency_stop = false;
            m_telemetry->state = static_cast<int>(initial_state);
            m_current_cfg_version = m_telemetry->cfg_version;
            pthread_mutex_unlock(&(m_telemetry->mtx));
        }
    } catch (const std::exception& e) {
        std::cerr << "[FsmShmBridge] Warning: Failed to open POSIX SHM: " << e.what() << std::endl;
        m_telemetry = nullptr;
    }

    try {
        m_sensor_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("Sensors");
        m_fsm_pub = std::make_unique<shared::services::memory::MemoryShareComponentServer>("FSM");
    } catch (const std::exception& e) {
        std::cerr << "[FsmShmBridge] Warning: MemoryShare subscriber/publisher: " << e.what() << std::endl;
    }

    return true;
}

void FsmShmBridge::sync_config_to_shm(float max_accel, float cruise_spd, float min_curve_spd,
                                      float safety_factor, float brake_gain, float kp, float kd) {
    if (!m_telemetry) return;

    pthread_mutex_lock(&(m_telemetry->mtx));
    m_telemetry->cfg_max_acceleration = max_accel;
    m_telemetry->cfg_cruise_speed     = cruise_spd;
    m_telemetry->cfg_min_curve_speed  = min_curve_spd;
    m_telemetry->cfg_curve_safety_factor = safety_factor;
    m_telemetry->cfg_predictive_brake_gain = brake_gain;
    m_telemetry->cfg_line_kp          = kp;
    m_telemetry->cfg_line_kd          = kd;
    m_telemetry->cfg_version          = m_telemetry->cfg_version + 1;
    m_current_cfg_version = m_telemetry->cfg_version;
    pthread_mutex_unlock(&(m_telemetry->mtx));
}

bool FsmShmBridge::check_config_from_shm(float& max_accel, float& cruise_spd, float& min_curve_spd,
                                         float& safety_factor, float& brake_gain, float& kp, float& kd) {
    if (!m_telemetry) return false;

    bool updated = false;
    pthread_mutex_lock(&(m_telemetry->mtx));
    if (m_telemetry->cfg_version > m_current_cfg_version && m_telemetry->cfg_version > 0) {
        m_current_cfg_version = m_telemetry->cfg_version;
        max_accel      = m_telemetry->cfg_max_acceleration;
        cruise_spd     = m_telemetry->cfg_cruise_speed;
        min_curve_spd  = m_telemetry->cfg_min_curve_speed;
        if (m_telemetry->cfg_curve_safety_factor > 0.1f) safety_factor = m_telemetry->cfg_curve_safety_factor;
        if (m_telemetry->cfg_predictive_brake_gain >= 0.0f) brake_gain = m_telemetry->cfg_predictive_brake_gain;
        kp             = m_telemetry->cfg_line_kp;
        kd             = m_telemetry->cfg_line_kd;
        updated = true;
    }
    pthread_mutex_unlock(&(m_telemetry->mtx));
    return updated;
}

bool FsmShmBridge::read_sensors(float& dist, uint8_t line_sensors[5], float line_darkness[5],
                                bool& emergency, bool& sensors_alive) {
    if (!m_telemetry) {
        sensors_alive = false;
        return false;
    }

    uint64_t current_heartbeat = 0;
    pthread_mutex_lock(&(m_telemetry->mtx));
    dist = m_telemetry->filtered_dist;
    for (int i = 0; i < 5; ++i) {
        line_sensors[i] = m_telemetry->line_sensors[i];
        line_darkness[i] = m_telemetry->line_darkness[i];
    }
    emergency = m_telemetry->emergency_stop;
    current_heartbeat = m_telemetry->heartbeat;
    pthread_mutex_unlock(&(m_telemetry->mtx));

    sensors_alive = (current_heartbeat > 0);

    if (current_heartbeat == m_last_heartbeat) {
        m_heartbeat_starvation_cycles++;
    } else {
        m_heartbeat_starvation_cycles = 0;
        m_last_heartbeat = current_heartbeat;
    }

    return true;
}

bool FsmShmBridge::check_watchdog(int max_cycles) {
    return (m_heartbeat_starvation_cycles > max_cycles);
}

void FsmShmBridge::publish_control_commands(float speed_pct, float steering_deg, bool emergency,
                                           shared::fsm::MachineStates state) {
    if (m_telemetry) {
        pthread_mutex_lock(&(m_telemetry->mtx));
        m_telemetry->target_speed = speed_pct;
        m_telemetry->target_steering = steering_deg;
        m_telemetry->state = static_cast<int>(state);
        if (emergency) {
            m_telemetry->emergency_stop = true;
        }
        m_telemetry->heartbeat++;
        pthread_mutex_unlock(&(m_telemetry->mtx));
    }

    publish_fsm_telemetry(state);
}

void FsmShmBridge::publish_fsm_telemetry(shared::fsm::MachineStates state) {
    if (m_fsm_pub && m_fsm_pub->isValid()) {
        shared::fsm::FSMTelemetry telem{};
        telem.currentState = state;
        telem.requestedState = state;
        telem.flags = shared::fsm::flags::DISCOVERABLE;
        m_fsm_pub->publish(telem);
    }
}

void FsmShmBridge::emergency_stop_shm() {
    if (m_telemetry) {
        pthread_mutex_lock(&(m_telemetry->mtx));
        m_telemetry->target_speed = 0.0f;
        m_telemetry->target_steering = 0.0f;
        m_telemetry->emergency_stop = true;
        pthread_mutex_unlock(&(m_telemetry->mtx));
    }
}

} // namespace services
