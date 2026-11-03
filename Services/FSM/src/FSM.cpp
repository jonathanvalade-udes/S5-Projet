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

void FSM::transition_to(shared::fsm::MachineStates new_state) {
    if (m_current_state != new_state) {
        std::cout << "[FSM] State transition: " 
                  << static_cast<int>(m_current_state) 
                  << " -> " 
                  << static_cast<int>(new_state) 
                  << std::endl;
        m_current_state = new_state;
        m_state_timer = 0.0f;
        m_state_distance = 0.0f;
    }
}

bool FSM::init() {
    std::cout << "[FSM] Initializing Decision & Navigation engine at " << m_hz << " Hz..." << std::endl;

    m_line_controller = std::make_unique<LineFollowerController>(0.65f, 0.20f, STRAIGHT_CRUISE_SPEED, MIN_CURVE_SPEED);
    m_motion_profiler = std::make_unique<MotionProfiler>(MAX_ACCELERATION, STRAIGHT_CRUISE_SPEED);

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

    transition_to(shared::fsm::MachineStates::Init);
    m_running.store(true);
    return true;
}

void FSM::run() {
    const auto period_ms = std::chrono::milliseconds(1000 / m_hz);
    const float dt = 1.0f / static_cast<float>(m_hz);

    const float TARGET_OBSTACLE_STOP_DISTANCE_CM = 20.0f;
    const float OBSTACLE_SLOWDOWN_THRESHOLD_CM = 45.0f;

    uint64_t loop_count = 0;
    std::cout << "[FSM] Navigation & Avoidance loop started (a_max <= " << MAX_ACCELERATION << " m/s²)." << std::endl;

    while (m_running.load()) {
        auto start_time = std::chrono::steady_clock::now();

        float current_dist_cm = 100.0f;
        uint8_t line_sensors[5] = {0, 0, 1, 0, 0};
        bool shm_emergency = false;
        bool sensors_alive = true; // false tant qu'aucun producteur de capteurs n'a écrit dans la SHM

        // Lecture immédiate des capteurs au temps N via MemoryShare ou fallback POSIX SHM
        bool got_sensors = false;
        if (m_sensor_sub && m_sensor_sub->isConnected()) {
            shared::sensors::SensorsTelemetry s_telem{};
            if (m_sensor_sub->read(s_telem)) {
                current_dist_cm = s_telem.ultrasonicSensor.calculatedDistance;
                for (int i = 0; i < 5; ++i) {
                    line_sensors[i] = (s_telem.lineFollower.sensorsData[i].normalizedValue > 0.5f ||
                                       s_telem.lineFollower.sensorsData[i].rawADCValue > 500) ? 1 : 0;
                }
                got_sensors = true;
                sensors_alive = true;
            }
        }

        if (!got_sensors && m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            current_dist_cm = m_telemetry->filtered_dist;
            for (int i = 0; i < 5; ++i) {
                line_sensors[i] = m_telemetry->line_sensors[i];
            }
            shm_emergency = m_telemetry->emergency_stop;
            sensors_alive = (m_telemetry->heartbeat > 0);
            pthread_mutex_unlock(&(m_telemetry->mtx));
        } else if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            shm_emergency = m_telemetry->emergency_stop;
            pthread_mutex_unlock(&(m_telemetry->mtx));
        }

        // 1. Détection arrêt d'urgence externe
        if (shm_emergency) {
            transition_to(shared::fsm::MachineStates::Failure);
        }

        float desired_velocity = 0.0f;
        float desired_steering = 0.0f;
        bool req_emergency = false;

        // 2. Machine à états finis (FSM)
        switch (m_current_state) {
            case shared::fsm::MachineStates::Init: {
                desired_velocity = 0.0f;
                desired_steering = 0.0f;
                // Attend les premières données capteurs (ex. connexion de Godot) avant de démarrer,
                // sinon la ligne serait vue "perdue" et le FSM tomberait en échec.
                if (sensors_alive) {
                    m_state_timer += dt;
                }
                // Transition automatique après validation des ressources
                if (m_state_timer >= 0.1f) {
                    transition_to(shared::fsm::MachineStates::Unarmed);
                }
                break;
            }

            case shared::fsm::MachineStates::Unarmed: {
                desired_velocity = 0.0f;
                desired_steering = 0.0f;
                m_state_timer += dt;
                // Stabilisation passive (actionneurs désactivés) pendant 0.4s
                if (m_state_timer >= 0.4f) {
                    transition_to(shared::fsm::MachineStates::Armed);
                }
                break;
            }

            case shared::fsm::MachineStates::Armed: {
                desired_velocity = 0.0f;
                desired_steering = 0.0f;
                m_state_timer += dt;
                // Prêt pour engagement nominal
                if (m_state_timer >= 0.3f) {
                    std::cout << "[FSM] Armed -> Top départ de suivi de ligne!" << std::endl;
                    transition_to(shared::fsm::MachineStates::LineFollowingForward);
                }
                break;
            }

            case shared::fsm::MachineStates::LineFollowingForward: {
                LineFollowerOutput line_out = m_line_controller ? m_line_controller->update(line_sensors, dt) 
                                                                : LineFollowerOutput{0.0f, 0.0f, STRAIGHT_CRUISE_SPEED, false, false};
                desired_steering = line_out.steer_degrees;

                // Vitesse adaptative dynamique :
                // 0.32 m/s en ligne droite pleine, diminuant continûment vers 0.14 m/s en virage serré
                float steer_ratio = std::abs(line_out.steer_normalized); // 0.0 à 1.0
                float adaptive_velocity = STRAIGHT_CRUISE_SPEED - ((STRAIGHT_CRUISE_SPEED - MIN_CURVE_SPEED) * steer_ratio);
                desired_velocity = adaptive_velocity;

                if (line_out.is_finished) {
                    // Arrivée : arrêt progressif (le MotionProfiler limite la décélération à a_max,
                    // donc la bille ne tombe pas) puis maintien définitif à l'arrêt dans Stopped.
                    std::cout << "[FSM] Barre d'arrivée détectée (capteurs "
                              << int(line_sensors[0]) << int(line_sensors[1]) << int(line_sensors[2])
                              << int(line_sensors[3]) << int(line_sensors[4]) << ") -> arrêt final." << std::endl;
                    m_course_finished = true;
                    desired_velocity = 0.0f;
                    transition_to(shared::fsm::MachineStates::Stopped);
                } else if (line_out.is_line_lost) {
                    // S5 : Décrochage de ligne -> tenter une marche arrière autonome de récupération
                    std::cout << "[FSM] Ligne perdue -> Engagement routine Reverse (S5) pour ré-acquisition!" << std::endl;
                    transition_to(shared::fsm::MachineStates::Reverse);
                } else if (current_dist_cm <= TARGET_OBSTACLE_STOP_DISTANCE_CM) {
                    desired_velocity = 0.0f;
                    transition_to(shared::fsm::MachineStates::CollisionRisk);
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
                // Transition dès immobilisation complète
                if (m_motion_profiler && m_motion_profiler->get_current_speed() < 0.03f) {
                    transition_to(shared::fsm::MachineStates::Stopped);
                }
                break;
            }

            case shared::fsm::MachineStates::Stopped: {
                desired_velocity = 0.0f;
                desired_steering = 0.0f;
                m_state_timer += dt;

                // Fin de parcours : on reste à l'arrêt, sans reprise du suivi de ligne
                if (m_course_finished) {
                    break;
                }

                // Confirmation rapide en 200 ms (sans latence superflue)
                if (m_state_timer >= 0.2f) {
                    if (current_dist_cm <= 25.0f) {
                        // Obstacle confirmé -> Évitement autonome (S4 RightDeviation)
                        std::cout << "[FSM] Obstacle confirmé à " << current_dist_cm << " cm -> Démarrage évitement S4!" << std::endl;
                        transition_to(shared::fsm::MachineStates::RightDeviation);
                    } else {
                        // Obstacle retiré -> reprise immédiate
                        transition_to(shared::fsm::MachineStates::LineFollowingForward);
                    }
                }
                break;
            }

            case shared::fsm::MachineStates::RightDeviation: {
                m_state_timer += dt;
                desired_steering = 25.0f;  // Angle de dégagement à droite
                desired_velocity = 0.16f;  // Vitesse douce respectant a <= 0.735 m/s²

                float curr_v = m_motion_profiler ? std::abs(m_motion_profiler->get_current_speed()) : desired_velocity;
                m_state_distance += curr_v * dt;

                // Dégagement latéral complété dès que ~18 cm sont parcourus ou timer de sécurité
                if (m_state_distance >= 0.18f || m_state_timer >= 1.2f) {
                    std::cout << "[FSM] Dégagement latéral achevé (" << (m_state_distance * 100.0f) << " cm) -> Recoupage (LineSearching)..." << std::endl;
                    transition_to(shared::fsm::MachineStates::LineSearching);
                }
                break;
            }

            case shared::fsm::MachineStates::LeftDeviation: {
                m_state_timer += dt;
                desired_steering = -25.0f; // Angle de dégagement à gauche
                desired_velocity = 0.16f;

                float curr_v = m_motion_profiler ? std::abs(m_motion_profiler->get_current_speed()) : desired_velocity;
                m_state_distance += curr_v * dt;

                if (m_state_distance >= 0.18f || m_state_timer >= 1.2f) {
                    transition_to(shared::fsm::MachineStates::LineSearching);
                }
                break;
            }

            case shared::fsm::MachineStates::LineSearching: {
                m_state_timer += dt;
                desired_steering = -24.0f; // Braquage vers la piste
                desired_velocity = 0.15f;

                bool line_seen = (line_sensors[0] || line_sensors[1] || line_sensors[2] || line_sensors[3] || line_sensors[4]);
                // Dès que le ruban est vu après au moins 250 ms de recherche
                if (line_seen && m_state_timer >= 0.25f) {
                    std::cout << "[FSM] Ruban noir détecté -> Transition vers LineDetected!" << std::endl;
                    transition_to(shared::fsm::MachineStates::LineDetected);
                } else if (m_state_timer >= 4.0f) {
                    std::cerr << "[FSM] Erreur : Timeout recherche de piste ! Failsafe." << std::endl;
                    transition_to(shared::fsm::MachineStates::LineSearchingTimedOut);
                    req_emergency = true;
                }
                break;
            }

            case shared::fsm::MachineStates::LineDetected: {
                m_state_timer += dt;
                desired_steering = 0.0f;
                desired_velocity = 0.14f;

                if (m_line_controller) {
                    m_line_controller->reset();
                }

                // Raccrochage immédiat dès que le capteur central est actif
                if (line_sensors[2] || m_state_timer >= 0.2f) {
                    std::cout << "[FSM] Verrouillage sur ruban réussi -> Reprise suivi nominal!" << std::endl;
                    transition_to(shared::fsm::MachineStates::LineFollowingForward);
                }
                break;
            }

            case shared::fsm::MachineStates::Reverse: {
                // Exigence S5 : Marche arrière guidée autonome (< 200 mm)
                m_state_timer += dt;
                desired_velocity = -0.12f; // Vitesse de recul bornée

                float err = 0.0f;
                if (m_line_controller) {
                    err = m_line_controller->update(line_sensors, dt).steer_degrees;
                }
                desired_steering = -err; // Inversion du braquage en marche arrière

                float curr_v = m_motion_profiler ? std::abs(m_motion_profiler->get_current_speed()) : std::abs(desired_velocity);
                m_state_distance += curr_v * dt;

                bool central_found = (line_sensors[1] || line_sensors[2] || line_sensors[3]);
                // Si la ligne est réacquise après avoir reculé d'au moins 3 cm :
                if (central_found && m_state_distance >= 0.03f) {
                    std::cout << "[FSM] Ruban réacquis en marche arrière (" << (m_state_distance * 1000.0f) << " mm) -> Reprise suivi nominal!" << std::endl;
                    if (m_line_controller) m_line_controller->reset();
                    transition_to(shared::fsm::MachineStates::LineFollowingForward);
                } else if (m_state_distance >= 0.18f || m_state_timer >= 2.0f) {
                    // Limite stricte S5 atteinte (< 200 mm) sans retrouver la ligne -> arrêt de sécurité
                    std::cerr << "[FSM] Limite de recul atteinte (" << (m_state_distance * 1000.0f) << " mm) sans ligne -> Arrêt d'urgence." << std::endl;
                    desired_velocity = 0.0f;
                    transition_to(shared::fsm::MachineStates::Failure);
                    req_emergency = true;
                }
                break;
            }

            case shared::fsm::MachineStates::CollisionOccured:
            case shared::fsm::MachineStates::Abort:
            case shared::fsm::MachineStates::LineSearchingTimedOut:
            case shared::fsm::MachineStates::Failure:
            default: {
                desired_velocity = 0.0f;
                desired_steering = 0.0f;
                req_emergency = true;
                break;
            }
        }

        // 3. Profil trapézoïdal et limitation cinématique stricte
        float bounded_velocity = m_motion_profiler ? m_motion_profiler->update(desired_velocity, dt) : desired_velocity;
        float motor_power_percent = (bounded_velocity / STRAIGHT_CRUISE_SPEED) * 50.0f;
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

        // Réception et synchronisation temps N sans délai artificiel
        bool woke_by_shm = false;
        if (m_sensor_sub && m_sensor_sub->isConnected()) {
            woke_by_shm = m_sensor_sub->waitForUpdate(period_ms.count());
        }

        if (!woke_by_shm) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed < period_ms) {
                std::this_thread::sleep_for(period_ms - elapsed);
            }
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
