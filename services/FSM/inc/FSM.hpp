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

    std::atomic<bool> m_running{false};
    int m_hz{50};
    shared::fsm::MachineStates m_current_state{shared::fsm::MachineStates::Init};
    float m_state_timer{0.0f};
    float m_state_distance{0.0f};
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

    uint32_t m_current_cfg_version{0};
    uint64_t m_last_heartbeat{0};
    int m_heartbeat_starvation_cycles{0};

    std::unique_ptr<LineFollowerController> m_line_controller;
    std::unique_ptr<MotionProfiler> m_motion_profiler;
    std::unique_ptr<SharedMemoryManager> m_shm_mgr;
    RobotTelemetry* m_telemetry{nullptr};

    int m_event_pipe_fd{-1};
    std::string m_event_pipe_path{"/tmp/peripheral"};
    bool check_start_trigger();

    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_sensor_sub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_fsm_pub;
};

} // namespace services
