#include "SimShmBridge.hpp"
#include "control/picar_config.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <cstring>
#include <iostream>

namespace services {

SimShmBridge::SimShmBridge() = default;

bool SimShmBridge::init() {
    try {
        m_shm_mgr = std::make_unique<SharedMemoryManager>();
        m_telemetry = m_shm_mgr->open_shm(true);
        if (m_telemetry) {
            auto initial_cfg = config::PicarConfig::read_config();
            config::PicarConfig::apply_to_shm(initial_cfg, m_telemetry);
        }
    } catch (const std::exception& e) {
        std::cerr << "[SimShmBridge] Warning: Failed to open POSIX SHM: " << e.what() << std::endl;
        m_telemetry = nullptr;
        return false;
    }
    reset_pipeline(100.0f);
    return true;
}

void SimShmBridge::reset_pipeline(float dist) {
    m_us_pipeline.reset(dist);
}

void SimShmBridge::inject_sim_telemetry(const GodotTelemetryIn& telem) {
    if (!m_telemetry) return;

    float current_speed = 0.0f;
    pthread_mutex_lock(&(m_telemetry->mtx));
    current_speed = m_telemetry->speed;
    pthread_mutex_unlock(&(m_telemetry->mtx));

    float filtered_dist = m_us_pipeline.process(telem.distance, 0.02f, current_speed);

    pthread_mutex_lock(&(m_telemetry->mtx));
    m_telemetry->filtered_dist = filtered_dist;
    // Godot est en miroir de la convention du robot (HAL : braquage + = droite, capteur D = droite) : un braquage
    // positif y tourne vers la gauche. SimBridge inverse donc l'ordre des capteurs ici et le signe du braquage
    // (SimBridge.cpp), pour que "gauche" et "droite" aient le meme sens en simulation et sur le robot.
    for (int i = 0; i < 5; ++i) {
        const int j = 4 - i;
        m_telemetry->line_sensors[i] = telem.line_sensors[j];
        m_telemetry->line_darkness[i] = std::clamp(telem.line_darkness[j], 0.0f, 1.0f);
    }
    m_telemetry->heartbeat++;
    pthread_mutex_unlock(&(m_telemetry->mtx));
}

void SimShmBridge::read_control_targets(float& target_spd, float& target_steer, bool& emergency, float& cruise_cfg) {
    target_spd = 0.0f;
    target_steer = 0.0f;
    emergency = false;
    cruise_cfg = 0.40f;

    if (!m_telemetry) return;

    pthread_mutex_lock(&(m_telemetry->mtx));
    target_spd = m_telemetry->target_speed;
    target_steer = m_telemetry->target_steering;
    emergency = m_telemetry->emergency_stop;
    cruise_cfg = (m_telemetry->cfg_cruise_speed > 0.05f) ? m_telemetry->cfg_cruise_speed : 0.40f;
    pthread_mutex_unlock(&(m_telemetry->mtx));
}

void SimShmBridge::read_vehicle_sensors(float& dist, uint8_t line_sensors[5]) {
    dist = 100.0f;
    for (int i = 0; i < 5; ++i) line_sensors[i] = 0;

    if (!m_telemetry) return;

    pthread_mutex_lock(&(m_telemetry->mtx));
    dist = m_telemetry->filtered_dist;
    for (int i = 0; i < 5; ++i) {
        line_sensors[i] = m_telemetry->line_sensors[i];
    }
    pthread_mutex_unlock(&(m_telemetry->mtx));
}

void SimShmBridge::set_emergency_stop(bool emergency, float target_spd) {
    if (!m_telemetry) return;

    pthread_mutex_lock(&(m_telemetry->mtx));
    m_telemetry->emergency_stop = emergency;
    m_telemetry->target_speed = target_spd;
    pthread_mutex_unlock(&(m_telemetry->mtx));
}

void SimShmBridge::apply_config_update(const std::string& key, float val) {
    config::PicarConfig::write_config(key, val);
    if (m_telemetry) {
        pthread_mutex_lock(&(m_telemetry->mtx));
        if (key == "MAX_ACCELERATION") m_telemetry->cfg_max_acceleration = val;
        else if (key == "STRAIGHT_CRUISE_SPEED" || key == "CRUISE_SPEED") m_telemetry->cfg_cruise_speed = val;
        else if (key == "MIN_CURVE_SPEED") m_telemetry->cfg_min_curve_speed = val;
        else if (key == "CURVE_SAFETY_FACTOR") m_telemetry->cfg_curve_safety_factor = val;
        else if (key == "PREDICTIVE_BRAKE_GAIN") m_telemetry->cfg_predictive_brake_gain = val;
        else if (key == "LINE_KP" || key == "KP") m_telemetry->cfg_line_kp = val;
        else if (key == "LINE_KD" || key == "KD") m_telemetry->cfg_line_kd = val;
        m_telemetry->cfg_version++;
        pthread_mutex_unlock(&(m_telemetry->mtx));
    }
}

void SimShmBridge::forward_start_to_fifo() {
    int fifo_fd = open("/tmp/peripherals.sock", O_WRONLY | O_NONBLOCK);
    if (fifo_fd >= 0) {
        const char* cmd = "trigger_start\n";
        ssize_t w = write(fifo_fd, cmd, strlen(cmd));
        (void)w;
        close(fifo_fd);
    }
}

} // namespace services
