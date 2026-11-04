#pragma once

#include <atomic>
#include <memory>
#include <string>

// Shared headers from libs
#include "header/header.hpp"
#include "services/services_telemetry.hpp"
#include "fsm/machine_component.hpp"
#include "sensors/sensors_telemetry.hpp"
#include "control/line_follower.hpp"
#include "control/motion_profiler.hpp"
#include "control/picar_constants.hpp"
#include "control/picar_config.hpp"
#include "ipc/shared_memory.hpp"
#include "shared-services/memory/subscriber.hpp"
#include "shared-services/memory/server.hpp"

namespace services {

class FSM {
public:
    explicit FSM(int hz = 50);
    ~FSM();

    bool init();
    void run();
    void stop();

    bool is_running() const { return m_running.load(); }
    shared::fsm::MachineStates get_current_state() const { return m_current_state; }
    void set_state(shared::fsm::MachineStates state) { transition_to(state); }

private:
    void transition_to(shared::fsm::MachineStates new_state);
    enum class DeviationResult { Running, LineFound, SearchLine };
    float avoidance_speed(float nominal) const;
    DeviationResult run_obstacle_deviation(float side, float dt, const float darkness[5], float& steering_deg, float& velocity);

    std::atomic<bool> m_running{false};
    int m_hz{50};
    shared::fsm::MachineStates m_current_state{shared::fsm::MachineStates::Init};
    float m_state_timer{0.0f};
    float m_state_distance{0.0f};
    int m_dev_phase{0};              // phase de la manoeuvre d'évitement (voir run_obstacle_deviation)
    float m_dev_phase_distance{0.0f};
    float m_dev_x{0.0f};             // position estimée pendant l'évitement : x vers l'avant (m)
    float m_dev_y{0.0f};             // y du côté de la déviation, 0 = ligne d'origine (m)
    float m_dev_obstacle_far_x{0.0f};// x estimé de la face arrière de l'obstacle (m)
    float m_ignore_finish_s{0.0f};   // durée restante pendant laquelle la barre d'arrivée est ignorée (après un évitement)
    float m_dev_trim_deg{0.0f};      // correction de braquage (biais des roues) mémorisée à l'entrée de l'évitement
    float m_dev_obstacle_dist{0.0f}; // distance à l'obstacle à l'entrée de la manoeuvre (m)
    float m_dev_steer_real{0.0f};    // braquage réel estimé (°), après retard du servo
    float m_dev_heading{0.0f};       // cap estimé par odométrie pendant l'évitement (rad, + = côté de la déviation)
    bool m_course_finished{false}; // Barre d'arrivée vue : l'état Stopped devient terminal

    // Paramètres cinématiques et de suivi (modifiables à chaud via WiFi / .config / POSIX)
    float m_straight_cruise_speed{picar::constants::DEFAULT_STRAIGHT_CRUISE_SPEED};
    float m_min_curve_speed{picar::constants::DEFAULT_MIN_CURVE_SPEED};
    float m_curve_safety_factor{picar::constants::DEFAULT_CURVE_SAFETY_FACTOR};
    float m_max_acceleration{picar::constants::DEFAULT_MAX_ACCELERATION};
    float m_line_kp{picar::constants::DEFAULT_LINE_KP};
    float m_line_kd{picar::constants::DEFAULT_LINE_KD};
    float m_obstacle_stop_dist_cm{picar::constants::DEFAULT_OBSTACLE_STOP_DIST_CM};
    float m_reverse_speed{picar::constants::DEFAULT_REVERSE_SPEED};
    float m_auto_start_delay{0.0f}; // 0.0f = uniquement au bouton / TRIGGER_START
    float m_avoid_side{1.0f};        // côté de contournement des obstacles : +1 droite, -1 gauche (env AVOID_SIDE)
    float m_predictive_brake_gain{picar::constants::DEFAULT_PREDICTIVE_BRAKE_GAIN};

    uint32_t m_current_cfg_version{0};
    uint64_t m_last_heartbeat{0};
    int m_heartbeat_starvation_cycles{0};

    std::unique_ptr<LineFollowerController> m_line_controller;
    std::unique_ptr<MotionProfiler> m_motion_profiler;
    std::unique_ptr<SharedMemoryManager> m_shm_mgr;
    RobotTelemetry* m_telemetry{nullptr};

    int m_event_pipe_fd{-1};
    std::string m_event_pipe_path{"/tmp/peripherals.sock"};
    bool check_start_trigger();

    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_sensor_sub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_fsm_pub;
};

} // namespace services
