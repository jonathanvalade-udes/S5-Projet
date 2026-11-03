#include "FSM.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

namespace services {

FSM::FSM(int hz) : m_running(false), m_hz(hz) {}

FSM::~FSM() {
    stop();
}

bool FSM::init() {
    std::cout << "[FSM] Initializing Decision & Navigation engine at " << m_hz << " Hz..." << std::endl;

    m_line_controller = std::make_unique<LineFollowerController>(0.65f, 0.20f, 0.22f, 0.10f);
    m_motion_profiler = std::make_unique<MotionProfiler>(0.735f, 0.25f); // a_max = 0.735 m/s² (FastDrink)

    try {
        m_shm_mgr = std::make_unique<SharedMemoryManager>();
        m_telemetry = m_shm_mgr->open_shm(true);
    } catch (const std::exception& e) {
        std::cerr << "[FSM] Warning: Failed to open POSIX SHM: " << e.what() << std::endl;
        m_telemetry = nullptr;
    }

    try {
        m_sensor_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("Sensors");
        m_fsm_pub = std::make_unique<shared::services::memory::MemoryShareComponentServer>("FSM");
    } catch (const std::exception& e) {
        std::cerr << "[FSM] Warning: MemoryShare subscriber/publisher: " << e.what() << std::endl;
    }

    m_running.store(true);
    return true;
}

void FSM::run() {
    const auto period_ms = std::chrono::milliseconds(1000 / m_hz);
    const float dt = 1.0f / static_cast<float>(m_hz);

    const float TARGET_OBSTACLE_STOP_DISTANCE_CM = 20.0f;
    const float OBSTACLE_SLOWDOWN_THRESHOLD_CM = 45.0f;

    uint64_t loop_count = 0;
    std::cout << "[FSM] Navigation loop started (a_max <= 0.735 m/s²)." << std::endl;

    while (m_running.load()) {
        auto start_time = std::chrono::steady_clock::now();

        float current_dist_cm = 100.0f;
        uint8_t line_sensors[5] = {0, 0, 1, 0, 0};

        if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            current_dist_cm = m_telemetry->filtered_dist;
            for (int i = 0; i < 5; ++i) {
                line_sensors[i] = m_telemetry->line_sensors[i];
            }
            pthread_mutex_unlock(&(m_telemetry->mtx));
        } else if (m_sensor_sub && m_sensor_sub->isConnected()) {
            shared::sensors::SensorsTelemetry s_telem{};
            if (m_sensor_sub->read(s_telem)) {
                current_dist_cm = s_telem.ultrasonicSensor.calculatedDistance;
                for (int i = 0; i < 5; ++i) {
                    line_sensors[i] = (s_telem.lineFollower.sensorsData[i].normalizedValue > 0.5f) ? 1 : 0;
                }
            }
        }

        // Étape A : Suivi de ligne optique (PD)
        LineFollowerOutput line_out = m_line_controller ? m_line_controller->update(line_sensors, dt) 
                                                        : LineFollowerOutput{0.0f, 0.0f, 0.22f, false, false};

        // Étape B : Gestion d'obstacle
        float desired_velocity = line_out.target_velocity;
        shared::fsm::MachineStates fsm_state = shared::fsm::MachineStates::LineFollowingForward;

        if (line_out.is_finished) {
            desired_velocity = 0.0f;
            fsm_state = shared::fsm::MachineStates::Stopped;
        } else if (current_dist_cm <= TARGET_OBSTACLE_STOP_DISTANCE_CM) {
            desired_velocity = 0.0f;
            fsm_state = shared::fsm::MachineStates::CollisionRisk;
        } else if (current_dist_cm < OBSTACLE_SLOWDOWN_THRESHOLD_CM) {
            float distance_to_target_m = (current_dist_cm - TARGET_OBSTACLE_STOP_DISTANCE_CM) / 100.0f;
            if (m_motion_profiler) {
                float safe_approach_speed = m_motion_profiler->calculate_max_safe_approach_speed(distance_to_target_m);
                desired_velocity = std::min(desired_velocity, safe_approach_speed);
            }
            fsm_state = shared::fsm::MachineStates::CollisionRisk;
        }

        // Étape C : Profil trapézoïdal
        float bounded_velocity = m_motion_profiler ? m_motion_profiler->update(desired_velocity, dt) : desired_velocity;
        float motor_power_percent = (bounded_velocity / 0.25f) * 50.0f;
        motor_power_percent = std::clamp(motor_power_percent, 0.0f, 100.0f);

        bool req_emergency = line_out.is_line_lost;
        if (req_emergency) {
            fsm_state = shared::fsm::MachineStates::Failure;
        }

        // Mise à jour SHM
        if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            m_telemetry->target_speed = motor_power_percent;
            m_telemetry->target_steering = line_out.steer_degrees;
            m_telemetry->emergency_stop = req_emergency;
            m_telemetry->state = static_cast<uint32_t>(fsm_state);
            pthread_mutex_unlock(&(m_telemetry->mtx));
        }

        // Publication FSM telemetry
        if (m_fsm_pub && m_fsm_pub->isValid()) {
            shared::fsm::FSMTelemetry fsm_telem{};
            fsm_telem.currentState = fsm_state;
            fsm_telem.requestedState = fsm_state;
            fsm_telem.flags = shared::fsm::flags::RDY;
            m_fsm_pub->publish(fsm_telem);
        }

        loop_count++;
        if (loop_count % m_hz == 0) {
            std::cout << "[FSM] Speed: " << bounded_velocity 
                      << " m/s (" << motor_power_percent << "%)"
                      << " | Steering: " << line_out.steer_degrees << " deg"
                      << " | State: " << static_cast<int>(fsm_state) << std::endl;
        }

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < period_ms) {
            std::this_thread::sleep_for(period_ms - elapsed);
        }
    }

    if (m_telemetry) {
        pthread_mutex_lock(&(m_telemetry->mtx));
        m_telemetry->target_speed = 0.0f;
        m_telemetry->target_steering = 0.0f;
        pthread_mutex_unlock(&(m_telemetry->mtx));
    }

    std::cout << "[FSM] Service loop terminated." << std::endl;
}

void FSM::stop() {
    if (m_running.load()) {
        std::cout << "[FSM] Stopping service..." << std::endl;
        m_running.store(false);
    }
}

} // namespace services
