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

    static constexpr float STRAIGHT_CRUISE_SPEED = 0.32f; // m/s en ligne droite
    static constexpr float MIN_CURVE_SPEED = 0.14f;       // m/s en virage serré
    static constexpr float MAX_ACCELERATION = 0.735f;     // m/s² (seuil critique FastDrink)

    std::unique_ptr<LineFollowerController> m_line_controller;
    std::unique_ptr<MotionProfiler> m_motion_profiler;
    std::unique_ptr<SharedMemoryManager> m_shm_mgr;
    RobotTelemetry* m_telemetry{nullptr};

    std::unique_ptr<shared::services::memory::MemoryShareComponentSubscriber> m_sensor_sub;
    std::unique_ptr<shared::services::memory::MemoryShareComponentServer> m_fsm_pub;
};

} // namespace services
