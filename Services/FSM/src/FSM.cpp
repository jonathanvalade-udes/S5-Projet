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

    m_current_state = shared::fsm::MachineStates::LineFollowingForward;
    m_state_timer = 0.0f;
    m_running.store(true);
    return true;
}

void FSM::run() {
    const auto period_ms = std::chrono::milliseconds(1000 / m_hz);
    const float dt = 1.0f / static_cast<float>(m_hz);

    const float TARGET_OBSTACLE_STOP_DISTANCE_CM = 20.0f;
    const float OBSTACLE_SLOWDOWN_THRESHOLD_CM = 45.0f;

    uint64_t loop_count = 0;
    std::cout << "[FSM] Navigation & Avoidance loop started (a_max <= 0.735 m/s²)." << std::endl;

    while (m_running.load()) {
        auto start_time = std::chrono::steady_clock::now();

        float current_dist_cm = 100.0f;
        uint8_t line_sensors[5] = {0, 0, 1, 0, 0};
        bool shm_emergency = false;

        if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            current_dist_cm = m_telemetry->filtered_dist;
            for (int i = 0; i < 5; ++i) {
                line_sensors[i] = m_telemetry->line_sensors[i];
            }
            shm_emergency = m_telemetry->emergency_stop;
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

        // 1. Détection arrêt d'urgence externe
        if (shm_emergency) {
            m_current_state = shared::fsm::MachineStates::Failure;
        }

        float desired_velocity = 0.0f;
        float desired_steering = 0.0f;
        bool req_emergency = false;

        // 2. Machine à états finis (FSM)
        switch (m_current_state) {
            case shared::fsm::MachineStates::LineFollowingForward: {
                LineFollowerOutput line_out = m_line_controller ? m_line_controller->update(line_sensors, dt) 
                                                                : LineFollowerOutput{0.0f, 0.0f, 0.22f, false, false};
                desired_steering = line_out.steer_degrees;
                desired_velocity = line_out.target_velocity;

                if (line_out.is_finished) {
                    desired_velocity = 0.0f;
                    m_current_state = shared::fsm::MachineStates::Stopped;
                    m_state_timer = 0.0f;
                } else if (line_out.is_line_lost) {
                    m_current_state = shared::fsm::MachineStates::Failure;
                    req_emergency = true;
                } else if (current_dist_cm <= TARGET_OBSTACLE_STOP_DISTANCE_CM) {
                    desired_velocity = 0.0f;
                    m_current_state = shared::fsm::MachineStates::CollisionRisk;
                    m_state_timer = 0.0f;
                } else if (current_dist_cm < OBSTACLE_SLOWDOWN_THRESHOLD_CM) {
                    float distance_to_target_m = (current_dist_cm - TARGET_OBSTACLE_STOP_DISTANCE_CM) / 100.0f;
                    if (m_motion_profiler) {
                        float safe_approach_speed = m_motion_profiler->calculate_max_safe_approach_speed(distance_to_target_m);
                        desired_velocity = std::min(desired_velocity, safe_approach_speed);
                    }
                }
                break;
            }

            case shared::fsm::MachineStates::CollisionRisk: {
                desired_velocity = 0.0f;
                desired_steering = 0.0f;
                // Dès que le véhicule est stabilisé à l'arrêt, transition vers Stopped
                if (m_motion_profiler && m_motion_profiler->get_current_speed() < 0.03f) {
                    m_current_state = shared::fsm::MachineStates::Stopped;
                    m_state_timer = 0.0f;
                }
                break;
            }

            case shared::fsm::MachineStates::Stopped: {
                desired_velocity = 0.0f;
                desired_steering = 0.0f;
                m_state_timer += dt;

                // Après 0.5s de repos complet :
                if (m_state_timer >= 0.5f) {
                    if (current_dist_cm <= 25.0f) {
                        // S4 : Engager la manœuvre de contournement autonome
                        m_current_state = shared::fsm::MachineStates::RightDeviation;
                        m_state_timer = 0.0f;
                        std::cout << "[FSM] Obstacle confirmé à " << current_dist_cm << " cm -> Démarrage évitement (S4 RightDeviation)!" << std::endl;
                    } else {
                        // L'obstacle a été retiré, reprise nominale
                        m_current_state = shared::fsm::MachineStates::LineFollowingForward;
                        m_state_timer = 0.0f;
                    }
                }
                break;
            }

            case shared::fsm::MachineStates::RightDeviation: {
                m_state_timer += dt;
                desired_steering = 25.0f;  // Angle de braquage à droite
                desired_velocity = 0.15f;  // Vitesse douce respectant a <= 0.735 m/s²

                // Braquage pendant 1.2s pour dégager la largeur de l'obstacle
                if (m_state_timer >= 1.2f) {
                    m_current_state = shared::fsm::MachineStates::LineSearching;
                    m_state_timer = 0.0f;
                    std::cout << "[FSM] Dépassement latéral achevé -> Recherche du ruban (LineSearching)..." << std::endl;
                }
                break;
            }

            case shared::fsm::MachineStates::LineSearching: {
                m_state_timer += dt;
                desired_steering = -24.0f; // Braquage opposé pour recouper la piste
                desired_velocity = 0.14f;

                // Détection de tout capteur optique sur le ruban noir
                bool line_seen = (line_sensors[0] || line_sensors[1] || line_sensors[2] || line_sensors[3] || line_sensors[4]);
                if (line_seen && m_state_timer >= 0.5f) {
                    m_current_state = shared::fsm::MachineStates::LineDetected;
                    m_state_timer = 0.0f;
                    std::cout << "[FSM] Ruban noir détecté -> Transition vers LineDetected!" << std::endl;
                } else if (m_state_timer >= 4.0f) {
                    std::cerr << "[FSM] Erreur : Timeout de recherche de ligne ! Failsafe." << std::endl;
                    m_current_state = shared::fsm::MachineStates::LineSearchingTimedOut;
                    req_emergency = true;
                }
                break;
            }

            case shared::fsm::MachineStates::LineDetected: {
                m_state_timer += dt;
                desired_steering = 0.0f;
                desired_velocity = 0.12f;

                if (m_line_controller) {
                    m_line_controller->reset();
                }

                if (line_sensors[2] || m_state_timer >= 0.3f) {
                    m_current_state = shared::fsm::MachineStates::LineFollowingForward;
                    m_state_timer = 0.0f;
                    std::cout << "[FSM] Verrouillage sur ruban réussi -> Reprise suivi nominal!" << std::endl;
                }
                break;
            }

            case shared::fsm::MachineStates::Reverse: {
                // Exigence S5 : Marche arrière guidée autonome (< 200 mm)
                m_state_timer += dt;
                desired_velocity = -0.10f; // Vitesse de recul
                float err = 0.0f;
                if (m_line_controller) {
                    err = m_line_controller->update(line_sensors, dt).steer_degrees;
                }
                desired_steering = -err; // Inversion de direction en marche arrière

                // Distance cible = 150 mm (< 200 mm) à 0.10 m/s => 1.5 s
                if (m_state_timer >= 1.5f) {
                    desired_velocity = 0.0f;
                    m_current_state = shared::fsm::MachineStates::Stopped;
                    m_state_timer = 0.0f;
                    std::cout << "[FSM] Marche arrière terminée (< 200 mm) -> Arrêt." << std::endl;
                }
                break;
            }

            case shared::fsm::MachineStates::Failure:
            case shared::fsm::MachineStates::LineSearchingTimedOut:
            default: {
                desired_velocity = 0.0f;
                desired_steering = 0.0f;
                req_emergency = true;
                break;
            }
        }

        // 3. Profil trapézoïdal et limitation cinématique
        float bounded_velocity = m_motion_profiler ? m_motion_profiler->update(desired_velocity, dt) : desired_velocity;
        float motor_power_percent = (bounded_velocity / 0.25f) * 50.0f;
        motor_power_percent = std::clamp(motor_power_percent, -100.0f, 100.0f);

        // 4. Mise à jour Mémoire Partagée (POSIX SHM)
        if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            m_telemetry->target_speed = motor_power_percent;
            m_telemetry->target_steering = desired_steering;
            m_telemetry->emergency_stop = req_emergency;
            m_telemetry->state = static_cast<uint32_t>(m_current_state);
            pthread_mutex_unlock(&(m_telemetry->mtx));
        }

        // 5. Publication FSM telemetry
        if (m_fsm_pub && m_fsm_pub->isValid()) {
            shared::fsm::FSMTelemetry fsm_telem{};
            fsm_telem.currentState = m_current_state;
            fsm_telem.requestedState = m_current_state;
            fsm_telem.flags = shared::fsm::flags::RDY;
            m_fsm_pub->publish(fsm_telem);
        }

        loop_count++;
        if (loop_count % m_hz == 0) {
            std::cout << "[FSM] Speed: " << bounded_velocity 
                      << " m/s (" << motor_power_percent << "%)"
                      << " | Steering: " << desired_steering << " deg"
                      << " | State: " << static_cast<int>(m_current_state) << std::endl;
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
