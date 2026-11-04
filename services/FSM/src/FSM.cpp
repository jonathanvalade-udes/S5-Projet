#include "FSM.hpp"
#include <iostream>
#include <cstdlib>
#include <string>
#include <chrono>
#include <thread>
#include <algorithm>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#if !defined(_WIN32)
#include <unistd.h>
#else
#define O_NONBLOCK 0
inline int mkfifo(const char*, int) { return 0; }
#endif

namespace services {

FSM::FSM(int hz) : m_running(false), m_hz(hz) {}

FSM::~FSM() {
    stop();
}

bool FSM::check_start_trigger() {
    if (m_event_pipe_fd < 0) {
        struct stat st;
        if (stat(m_event_pipe_path.c_str(), &st) == 0 && S_ISSOCK(st.st_mode)) {
            int sfd = ::socket(AF_UNIX, SOCK_STREAM, 0);
            if (sfd >= 0) {
                int flags = fcntl(sfd, F_GETFL, 0);
                if (flags >= 0) fcntl(sfd, F_SETFL, flags | O_NONBLOCK);
                sockaddr_un addr{};
                addr.sun_family = AF_UNIX;
                strncpy(addr.sun_path, m_event_pipe_path.c_str(), sizeof(addr.sun_path) - 1);
                if (::connect(sfd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == 0 || errno == EINPROGRESS) {
                    m_event_pipe_fd = sfd;
                    std::cout << "[FSM] Connecté au socket d'événements " << m_event_pipe_path << " (attente du signal de départ)" << std::endl;
                } else {
                    close(sfd);
                }
            }
        } else if (stat(m_event_pipe_path.c_str(), &st) == 0 && S_ISFIFO(st.st_mode)) {
            m_event_pipe_fd = open(m_event_pipe_path.c_str(), O_RDWR | O_NONBLOCK);
        }
        if (m_event_pipe_fd < 0) return false;
    }

    char buf[128];
    ssize_t n = read(m_event_pipe_fd, buf, sizeof(buf) - 1);
    if (n > 0) {
        buf[n] = '\0';
        std::string msg(buf);
        std::string lower_msg = msg;
        std::transform(lower_msg.begin(), lower_msg.end(), lower_msg.begin(), ::tolower);
        if (lower_msg.find("trigger_start") != std::string::npos || 
            lower_msg.find("start") != std::string::npos ||
            msg.find("1") != std::string::npos) {
            std::cout << "[FSM] Événement POSIX '" << msg << "' reçu -> Top départ !" << std::endl;
            return true;
        }
    } else if (n == 0) {
        // Déconnexion ou fermeture du socket côté serveur
        close(m_event_pipe_fd);
        m_event_pipe_fd = -1;
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        close(m_event_pipe_fd);
        m_event_pipe_fd = -1;
    }
    return false;
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
        m_dev_phase = 0;
        m_dev_phase_distance = 0.0f;
        m_dev_heading = 0.0f;
        m_dev_steer_real = 0.0f;
        m_dev_x = 0.0f;
        m_dev_y = 0.0f;

        // Si on entre dans Armed, purger tout ancien événement résiduel dans le tube
        if (new_state == shared::fsm::MachineStates::Armed && m_event_pipe_fd >= 0) {
            char drain[128];
            while (read(m_event_pipe_fd, drain, sizeof(drain)) > 0) {}
        }
    }
}

bool FSM::init() {
    std::cout << "[FSM] Initializing Decision & Navigation engine at " << m_hz << " Hz..." << std::endl;

    // Chargement de la configuration via fsm.conf (avec fallback automatique sur picar.conf et picar_constants.hpp)
    if (m_config.load("fsm.conf")) {
        std::cout << "[FSM] Configuration chargée depuis " << m_config.get_active_filepath() << std::endl;
        m_max_acceleration      = m_config.get_float("MAX_ACCELERATION", m_max_acceleration);
        m_straight_cruise_speed = m_config.get_float("STRAIGHT_CRUISE_SPEED", m_config.get_float("CRUISE_SPEED", m_straight_cruise_speed));
        m_min_curve_speed       = m_config.get_float("MIN_CURVE_SPEED", m_min_curve_speed);
        m_curve_safety_factor   = m_config.get_float("CURVE_SAFETY_FACTOR", m_curve_safety_factor);
        m_predictive_brake_gain = m_config.get_float("PREDICTIVE_BRAKE_GAIN", m_predictive_brake_gain);
        m_line_kp               = m_config.get_float("LINE_KP", m_config.get_float("KP", m_line_kp));
        m_line_kd               = m_config.get_float("LINE_KD", m_config.get_float("KD", m_line_kd));
        m_obstacle_stop_dist_cm = m_config.get_float("OBSTACLE_STOP_DIST_CM", m_obstacle_stop_dist_cm);
        m_reverse_speed         = m_config.get_float("REVERSE_SPEED", m_reverse_speed);
        m_avoid_side            = m_config.get_float("AVOID_SIDE", m_avoid_side);
        m_auto_start_delay      = m_config.get_float("AUTO_START_DELAY", m_auto_start_delay);
    } else {
        auto cfg = config::PicarConfig::read_config();
        m_max_acceleration      = cfg.max_acceleration;
        m_straight_cruise_speed = cfg.cruise_speed;
        m_min_curve_speed       = cfg.min_curve_speed;
        m_curve_safety_factor   = cfg.curve_safety_factor;
        m_predictive_brake_gain = cfg.predictive_brake_gain;
        m_line_kp               = cfg.line_kp;
        m_line_kd               = cfg.line_kd;
        m_obstacle_stop_dist_cm = cfg.obstacle_stop_dist_cm;
        m_reverse_speed         = cfg.reverse_speed;
        m_auto_start_delay      = cfg.auto_start_delay;
    }

    // Côté de contournement des obstacles : variable d'environnement AVOID_SIDE (gauche|droite|left|right)
    if (const char* side_env = std::getenv("AVOID_SIDE")) {
        const std::string side(side_env);
        if (side == "gauche" || side == "left" || side == "g" || side == "l") m_avoid_side = -1.0f;
        else if (side == "droite" || side == "right" || side == "d" || side == "r") m_avoid_side = 1.0f;
        else std::cerr << "[FSM] AVOID_SIDE='" << side << "' inconnu (gauche|droite) -> droite par défaut" << std::endl;
    }
    std::cout << "[FSM] Contournement des obstacles par la " << (m_avoid_side < 0.0f ? "GAUCHE" : "DROITE") << std::endl;

    m_line_controller = std::make_unique<LineFollowerController>(m_line_kp, m_line_kd, m_straight_cruise_speed, m_min_curve_speed);
    m_motion_profiler = std::make_unique<MotionProfiler>(m_max_acceleration, m_straight_cruise_speed);

    try {
        m_shm_mgr = std::make_unique<SharedMemoryManager>();
        m_telemetry = m_shm_mgr->open_shm(true);
        if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            m_telemetry->emergency_stop = false;
            m_telemetry->state = static_cast<int>(shared::fsm::MachineStates::Init);
            // On publie notre configuration (picar.conf) dans la SHM, même si un autre service l'a créée avec ses valeurs
            // par défaut (SimBridge en simulation). Sinon les autres services utilisent une autre vitesse de croisière
            // que le FSM : SimBridge convertirait le % moteur en m/s avec 0,40 au lieu de la vitesse configurée.
            {
                m_telemetry->cfg_max_acceleration = m_max_acceleration;
                m_telemetry->cfg_cruise_speed     = m_straight_cruise_speed;
                m_telemetry->cfg_min_curve_speed  = m_min_curve_speed;
                m_telemetry->cfg_curve_safety_factor = m_curve_safety_factor;
                m_telemetry->cfg_predictive_brake_gain = m_predictive_brake_gain;
                m_telemetry->cfg_line_kp          = m_line_kp;
                m_telemetry->cfg_line_kd          = m_line_kd;
                m_telemetry->cfg_version          = m_telemetry->cfg_version + 1;
            }
            m_current_cfg_version = m_telemetry->cfg_version;
            pthread_mutex_unlock(&(m_telemetry->mtx));
        }
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

    // Canal d'événements POSIX (connexion dynamique au socket de PeripheralsService)
    m_event_pipe_fd = -1;
    check_start_trigger();

    // Initialisation du serveur socket IPC (/tmp/fsm.sock)
    if (!m_ipc_server.init(m_ipc_socket_path, shared::commands::ServiceID::FSM, [this](const std::string& k, float v) {
        return this->apply_config_key(k, v);
    })) {
        std::cerr << "[FSM] Attention : impossible d'ouvrir le socket IPC " << m_ipc_socket_path << std::endl;
    } else {
        std::cout << "[FSM] Serveur socket IPC en écoute sur " << m_ipc_socket_path << std::endl;
    }

    transition_to(shared::fsm::MachineStates::Init);
    m_running.store(true);
    return true;
}

// Contournement d'un obstacle posé sur la ligne, en boucle ouverte sur l'odométrie (aucun capteur de cap) :
//   0. recul de 10 cm (le pare-chocs est trop près pour braquer sans toucher l'obstacle),
//   1. arc à braquage maximal du côté `side` jusqu'à 82° de cap,
//   2. arc inverse jusqu'au cap initial : la voiture roule parallèle à la ligne, ~20 cm à côté,
//   3. ligne droite jusqu'à ce que l'arrière de la voiture ait dépassé l'obstacle,
//   4. virage modéré vers la ligne jusqu'à ~35° de cap, puis 5. tout droit jusqu'au premier ruban vu (la voiture arrive
//      de l'extérieur : c'est la ligne d'origine). Un braquage à fond jusqu'au ruban la ferait arriver à plus de 90° :
//      elle couperait la ligne à l'envers et le suivi repartirait dans le mauvais sens.
// L'estimation de cap suppose un servo un peu lent (STEER_LAG_S). Réglages validés en simulation pour une voiture
// parfaite et pour la voiture "réaliste" (biais, jeu, latence) : en dessous de 70° ou sans recul, l'arrière de la
// voiture touche l'obstacle ; plus loin, la voiture risque de s'accrocher à une autre branche du parcours.
// side = +1 : droite (RightDeviation), -1 : gauche.
// Vitesse d'une manoeuvre d'évitement : la vitesse voulue, mais jamais sous MIN_CURVE_SPEED
float FSM::avoidance_speed(float nominal) const {
    return std::max(nominal, m_min_curve_speed);
}

FSM::DeviationResult FSM::run_obstacle_deviation(float side, float dt, const float darkness[5],
                                                 float& steering_deg, float& velocity) {
    constexpr float REVERSE_DISTANCE_M = 0.10f;
    constexpr float FORWARD_SPEED_NOMINAL = 0.16f;
    constexpr float REVERSE_SPEED_NOMINAL = 0.12f;
    constexpr float RETURN_SPEED_NOMINAL = 0.15f;
    constexpr float RETURN_STEER_DEG = 30.0f;            // braquage du virage de retour (moins que ARC_STEER_DEG)
    constexpr float RETURN_HEADING_RAD = 35.0f * 3.14159265f / 180.0f;  // angle d'approche de la ligne
    constexpr float RETURN_STRAIGHT_MAX_M = 0.60f;       // sans ruban sur cette distance : LineSearching
    constexpr float ARC_STEER_DEG = 45.0f;
    constexpr float ARC_HEADING_RAD = 82.0f * 3.14159265f / 180.0f;
    constexpr float WHEELBASE_M = 0.14f;
    constexpr float STEER_DEG_TO_REAL = 48.0f / 45.0f;   // le simulateur convertit 45° commandés en 48° de roue
    constexpr float STEER_LAG_S = 0.06f;                 // retard du servo de direction
    constexpr float OBSTACLE_DEPTH_M = 0.15f;            // profondeur supposée de l'obstacle
    constexpr float CAR_REAR_M = 0.15f;                  // distance de l'origine de la voiture à son arrière
    constexpr float CLEARANCE_M = 0.03f;
    constexpr float TIMEOUT_S = 14.0f;

    // Les moteurs ne décollent pas à très basse puissance : on ne descend jamais sous la plus petite vitesse que le
    // suivi de ligne utilise déjà (MIN_CURVE_SPEED), qui est validée sur le robot.
    const float FORWARD_SPEED = avoidance_speed(FORWARD_SPEED_NOMINAL);
    const float REVERSE_SPEED = avoidance_speed(REVERSE_SPEED_NOMINAL);
    const float RETURN_SPEED = avoidance_speed(RETURN_SPEED_NOMINAL);

    m_state_timer += dt;
    const float v = m_motion_profiler ? m_motion_profiler->get_current_speed() : 0.0f;
    m_dev_phase_distance += std::abs(v) * dt;

    // Position estimée : x vers l'avant, cap en rad (+ = vers le côté de la déviation)
    auto integrate = [&](float steer_deg) {
        // Braquage réellement obtenu : commande + correction apprise, limitée à ±45°, puis biais des roues (= -correction)
        const float applied = std::clamp(steer_deg + m_dev_trim_deg, -45.0f, 45.0f) - m_dev_trim_deg;
        m_dev_steer_real += (applied - m_dev_steer_real) * std::min(1.0f, dt / STEER_LAG_S);
        m_dev_heading += v / WHEELBASE_M * std::tan(m_dev_steer_real * STEER_DEG_TO_REAL * 3.14159265f / 180.0f) * dt * side;
        m_dev_x += v * std::cos(m_dev_heading) * dt;
        m_dev_y += v * std::sin(m_dev_heading) * dt;
    };
    auto next_phase = [&]() {
        std::cout << "[FSM] Evitement phase " << m_dev_phase << " -> " << (m_dev_phase + 1) << " (x=" << m_dev_x << " y=" << m_dev_y
                  << " cap=" << (m_dev_heading * 57.2958f) << " deg, t=" << m_state_timer << " s)" << std::endl;
        m_dev_phase++;
        m_dev_phase_distance = 0.0f;
    };

    switch (m_dev_phase) {
        case 0:  // recul
            steering_deg = 0.0f;
            velocity = -REVERSE_SPEED;
            integrate(steering_deg);
            if (m_dev_phase_distance >= REVERSE_DISTANCE_M) {
                m_dev_obstacle_far_x = m_dev_obstacle_dist + OBSTACLE_DEPTH_M;  // x = 0 : position à l'arrêt
                next_phase();
            }
            break;
        case 1:  // arc vers l'extérieur
            steering_deg = side * ARC_STEER_DEG;
            velocity = FORWARD_SPEED;
            integrate(steering_deg);
            if (m_dev_heading >= ARC_HEADING_RAD) next_phase();
            break;
        case 2:  // arc inverse : parallèle à la ligne
            steering_deg = -side * ARC_STEER_DEG;
            velocity = FORWARD_SPEED;
            integrate(steering_deg);
            if (m_dev_heading <= 0.0f) next_phase();
            break;
        case 3:  // tout droit jusqu'à avoir dépassé l'obstacle
            steering_deg = 0.0f;
            velocity = FORWARD_SPEED;
            integrate(steering_deg);
            if (m_dev_x - CAR_REAR_M >= m_dev_obstacle_far_x + CLEARANCE_M) next_phase();
            break;
        case 4:  // virage modéré vers la ligne, jusqu'à l'angle d'approche voulu
        default: {  // 5 : tout droit jusqu'au premier ruban
            const bool turning = (m_dev_phase == 4);
            steering_deg = turning ? -side * RETURN_STEER_DEG : 0.0f;
            velocity = RETURN_SPEED;
            integrate(steering_deg);
            const float max_darkness = std::max({darkness[0], darkness[1], darkness[2], darkness[3], darkness[4]});
            const float total_darkness = darkness[0] + darkness[1] + darkness[2] + darkness[3] + darkness[4];
            // Détection de ligne : au moins un capteur sur le ruban (>=0.50f) ou seuil simulation
            if (max_darkness >= 0.50f || (total_darkness >= 0.35f && max_darkness >= 0.35f)) return DeviationResult::LineFound;
            if (turning && m_dev_heading <= -RETURN_HEADING_RAD) next_phase();
            else if (!turning && m_dev_phase_distance >= RETURN_STRAIGHT_MAX_M) return DeviationResult::SearchLine;
            break;
        }
    }

    // Correction du biais de direction apprise sur la ligne droite précédente (la position estimée suppose un braquage exact)
    steering_deg = std::clamp(steering_deg + m_dev_trim_deg, -45.0f, 45.0f);
    return m_state_timer >= TIMEOUT_S ? DeviationResult::SearchLine : DeviationResult::Running;
}

void FSM::run() {
    constexpr float IGNORE_FINISH_AFTER_AVOIDANCE_S = 1.0f;  // décompté seulement pendant le suivi de ligne
    const auto period_ms = std::chrono::milliseconds(1000 / m_hz);
    const float dt = 1.0f / static_cast<float>(m_hz);

    const float OBSTACLE_SLOWDOWN_THRESHOLD_CM = 45.0f;
    const int max_watchdog_cycles = (m_hz * 250) / 1000; // 250 ms de timeout capteurs

    uint64_t loop_count = 0;
    std::cout << "[FSM] Navigation & Avoidance loop started (a_max <= " << m_max_acceleration 
              << " m/s², cruise=" << m_straight_cruise_speed << " m/s)." << std::endl;

    while (m_running.load()) {
        auto start_time = std::chrono::steady_clock::now();

        // Traitement des commandes IPC de configuration reçues sur /tmp/fsm.sock
        m_ipc_server.process_pending_commands();

        float current_dist_cm = 100.0f;
        uint8_t line_sensors[5] = {0, 0, 0, 0, 0};          // Fallback strict: aucune ligne par défaut
        float line_darkness[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        bool shm_emergency = false;
        bool sensors_alive = true; // false tant qu'aucun producteur de capteurs n'a écrit dans la SHM

        // 0. Synchronisation dynamique de la configuration en direct via POSIX SHM
        if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            if (m_telemetry->cfg_version > m_current_cfg_version && m_telemetry->cfg_version > 0) {
                m_current_cfg_version   = m_telemetry->cfg_version;
                m_max_acceleration      = m_telemetry->cfg_max_acceleration;
                m_straight_cruise_speed = m_telemetry->cfg_cruise_speed;
                m_min_curve_speed       = m_telemetry->cfg_min_curve_speed;
                m_curve_safety_factor   = (m_telemetry->cfg_curve_safety_factor > 0.1f) 
                                            ? m_telemetry->cfg_curve_safety_factor 
                                            : m_curve_safety_factor;
                m_predictive_brake_gain = (m_telemetry->cfg_predictive_brake_gain >= 0.0f)
                                            ? m_telemetry->cfg_predictive_brake_gain
                                            : m_predictive_brake_gain;
                m_line_kp               = m_telemetry->cfg_line_kp;
                m_line_kd               = m_telemetry->cfg_line_kd;

                if (m_motion_profiler) {
                    m_motion_profiler->set_max_accel(m_max_acceleration);
                    m_motion_profiler->set_max_speed(m_straight_cruise_speed);
                }
                if (m_line_controller) {
                    m_line_controller->set_kp(m_line_kp);
                    m_line_controller->set_kd(m_line_kd);
                    m_line_controller->set_cruise_speed(m_straight_cruise_speed);
                    m_line_controller->set_min_curve_speed(m_min_curve_speed);
                }
                std::cout << "[FSM] >>> CONFIG APPLIQUÉE EN DIRECT (v" << m_current_cfg_version 
                          << ") : a_max=" << m_max_acceleration << " m/s², cruise=" 
                          << m_straight_cruise_speed << " m/s, curve=" << m_min_curve_speed 
                          << " m/s, Kp=" << m_line_kp << ", Kd=" << m_line_kd << " <<<" << std::endl;
            }

            // Watchdog heartbeat capteurs
            uint64_t current_hb = m_telemetry->heartbeat;
            if (current_hb == m_last_heartbeat) {
                m_heartbeat_starvation_cycles++;
            } else {
                m_heartbeat_starvation_cycles = 0;
                m_last_heartbeat = current_hb;
            }
            pthread_mutex_unlock(&(m_telemetry->mtx));
        }

        // Si le heartbeat n'avance plus pendant plus de 250 ms hors état Init, arrêt de sécurité
        if (m_heartbeat_starvation_cycles > max_watchdog_cycles && m_current_state != shared::fsm::MachineStates::Init) {
            if (m_current_state != shared::fsm::MachineStates::Failure) {
                std::cerr << "[FSM] Watchdog: Perte de télémétrie capteurs (heartbeat figé depuis 250 ms) -> Arrêt d'urgence." << std::endl;
                transition_to(shared::fsm::MachineStates::Failure);
            }
        }

        // Lecture immédiate des capteurs au temps N via MemoryShare ou fallback POSIX SHM
        bool got_sensors = false;
        if (m_sensor_sub && m_sensor_sub->isConnected()) {
            shared::sensors::SensorsTelemetry s_telem{};
            if (m_sensor_sub->read(s_telem)) {
                current_dist_cm = s_telem.ultrasonicSensor.calculatedDistance;
                for (int i = 0; i < 5; ++i) {
                    line_darkness[i] = std::clamp(s_telem.lineFollower.sensorsData[i].normalizedValue, 0.0f, 1.0f);
                    line_sensors[i] = (line_darkness[i] > 0.5f) ? 1 : 0;
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
                line_darkness[i] = m_telemetry->line_darkness[i];
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

                bool triggered = check_start_trigger();
                if (!triggered && m_auto_start_delay > 0.0f && m_state_timer >= m_auto_start_delay) {
                    std::cout << "[FSM] Délai d'attente atteint (" << m_auto_start_delay << "s) -> Top départ de secours !" << std::endl;
                    triggered = true;
                }

                if (triggered) {
                    std::cout << "[FSM] Top départ de suivi de ligne !" << std::endl;
                    if (m_line_controller) m_line_controller->reset();
                    if (m_motion_profiler) m_motion_profiler->reset(0.0f);
                    m_course_finished = false;
                    transition_to(shared::fsm::MachineStates::LineFollowingForward);
                }
                break;
            }

            case shared::fsm::MachineStates::LineFollowingForward: {
                // Juste après un évitement, la barre d'arrivée est ignorée (la voiture recoupe la ligne en biais)
                m_ignore_finish_s = std::max(0.0f, m_ignore_finish_s - dt);
                if (m_line_controller) m_line_controller->set_finish_detection(m_ignore_finish_s <= 0.0f);
                LineFollowerOutput line_out = m_line_controller ? m_line_controller->update_analog(line_darkness, dt)
                                                                : LineFollowerOutput{0.0f, 0.0f, m_straight_cruise_speed, 0.0f, false, false};
                desired_steering = line_out.steer_degrees;

                // Vitesse adaptative basée sur la physique centrifuge d'Ackermann :
                // Limite centrifuge : a_c = v² / R <= a_max  ==>  v = safety_factor * sqrt(a_max * R)
                // Sous ~3° (ligne droite), la vitesse reste 100% à m_straight_cruise_speed.
                float adaptive_velocity = m_motion_profiler
                    ? m_motion_profiler->calculate_curve_speed_ackermann(
                          desired_steering, m_straight_cruise_speed, m_min_curve_speed, m_curve_safety_factor)
                    : m_straight_cruise_speed;

                // Feedforward prédictif : si l'erreur de ligne évolue rapidement (entrée de virage imminente),
                // on freine AVANT que l'angle d'Ackermann soit au maximum — compense la latence ~60-80 ms.
                // Sans lui, le robot est déjà en plein virage quand le MotionProfiler ralentit.
                if (m_motion_profiler && m_predictive_brake_gain > 0.0f) {
                    adaptive_velocity = m_motion_profiler->apply_predictive_brake(
                        line_out.d_error_dt, adaptive_velocity, m_predictive_brake_gain);
                }

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
                } else if (current_dist_cm <= m_obstacle_stop_dist_cm) {
                    desired_velocity = 0.0f;
                    transition_to(shared::fsm::MachineStates::CollisionRisk);
                } else if (current_dist_cm < OBSTACLE_SLOWDOWN_THRESHOLD_CM) {
                    float distance_to_target_m = (current_dist_cm - m_obstacle_stop_dist_cm) / 100.0f;
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

                // Fin de parcours au T : arrêt progressif (~0.5s) puis réarmement automatique vers Armed
                if (m_course_finished) {
                    if (m_state_timer >= 0.5f) {
                        std::cout << "[FSM] Fin de parcours au T achevée -> Réarmement automatique (Armed). En attente d'un nouveau 'trigger_start'..." << std::endl;
                        m_course_finished = false;
                        transition_to(shared::fsm::MachineStates::Armed);
                    }
                    break;
                }

                // Confirmation rapide en 200 ms (sans latence superflue)
                if (m_state_timer >= 0.2f) {
                    if (current_dist_cm <= 25.0f) {
                        // Obstacle confirmé -> Évitement autonome (S4 RightDeviation)
                        std::cout << "[FSM] Obstacle confirmé à " << current_dist_cm << " cm -> Démarrage évitement S4!" << std::endl;
                        transition_to(m_avoid_side < 0.0f ? shared::fsm::MachineStates::LeftDeviation
                                                          : shared::fsm::MachineStates::RightDeviation);
                        m_dev_obstacle_dist = current_dist_cm / 100.0f;
                        m_dev_trim_deg = m_line_controller ? m_line_controller->steering_trim() * 45.0f : 0.0f;
                        m_ignore_finish_s = IGNORE_FINISH_AFTER_AVOIDANCE_S;
                        std::cout << "[FSM] Correction de braquage apprise (biais des roues) : " << m_dev_trim_deg << " deg" << std::endl;
                        {
                            const float to_pct = (m_straight_cruise_speed > 0.01f) ? 50.0f / m_straight_cruise_speed : 0.0f;
                            std::cout << "[FSM] Evitement : avance " << avoidance_speed(0.16f) << " m/s (" << avoidance_speed(0.16f) * to_pct
                                      << " %), recul " << avoidance_speed(0.12f) << " m/s (" << avoidance_speed(0.12f) * to_pct << " %)" << std::endl;
                        }
                    } else {
                        // Obstacle retiré -> reprise immédiate
                        transition_to(shared::fsm::MachineStates::LineFollowingForward);
                    }
                }
                break;
            }

            case shared::fsm::MachineStates::RightDeviation:
            case shared::fsm::MachineStates::LeftDeviation: {
                const float side = (m_current_state == shared::fsm::MachineStates::RightDeviation) ? 1.0f : -1.0f;
                switch (run_obstacle_deviation(side, dt, line_darkness, desired_steering, desired_velocity)) {
                    case DeviationResult::LineFound:
                        std::cout << "[FSM] Dégagement latéral achevé : ruban retrouvé -> LineDetected" << std::endl;
                        transition_to(shared::fsm::MachineStates::LineDetected);
                        break;
                    case DeviationResult::SearchLine:
                        std::cout << "[FSM] Dégagement latéral achevé -> Recoupage (LineSearching)..." << std::endl;
                        transition_to(shared::fsm::MachineStates::LineSearching);
                        break;
                    case DeviationResult::Running:
                        break;
                }
                break;
            }

            case shared::fsm::MachineStates::LineSearching: {
                m_state_timer += dt;
                desired_steering = -24.0f; // Braquage vers la piste
                desired_velocity = 0.15f;

                float max_darkness = std::max({line_darkness[0], line_darkness[1], line_darkness[2], line_darkness[3], line_darkness[4]});
                bool line_seen = (max_darkness >= 0.55f);
                // Dès que le ruban est vu après au moins 250 ms de recherche
                if (line_seen && m_state_timer >= 0.25f) {
                    std::cout << "[FSM] Ruban noir détecté (max_darkness=" << max_darkness << ") -> Transition vers LineDetected!" << std::endl;
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

                // Raccrochage dès que les capteurs centraux voient le ruban (> 0.55)
                float max_darkness = std::max({line_darkness[0], line_darkness[1], line_darkness[2], line_darkness[3], line_darkness[4]});
                bool center_seen = (line_darkness[2] >= 0.55f || (line_darkness[1] >= 0.55f && line_darkness[3] >= 0.55f));
                if (center_seen || (max_darkness >= 0.55f && m_state_timer >= 0.35f)) {
                    std::cout << "[FSM] Verrouillage sur ruban réussi -> Reprise suivi nominal!" << std::endl;
                    transition_to(shared::fsm::MachineStates::LineFollowingForward);
                }
                break;
            }

            case shared::fsm::MachineStates::Reverse: {
                // Exigence S5 : Marche arrière guidée autonome (< 200 mm)
                m_state_timer += dt;
                desired_velocity = -m_reverse_speed; // Vitesse de recul configurable

                float err = 0.0f;
                if (m_line_controller) {
                    err = m_line_controller->update_analog(line_darkness, dt).steer_degrees;
                }
                desired_steering = -err; // Inversion du braquage en marche arrière

                float curr_v = m_motion_profiler ? std::abs(m_motion_profiler->get_current_speed()) : std::abs(desired_velocity);
                m_state_distance += curr_v * dt;

                bool central_found = (line_darkness[1] >= 0.55f || line_darkness[2] >= 0.55f || line_darkness[3] >= 0.55f);
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
        float motor_power_percent = (m_straight_cruise_speed > 0.01f) 
            ? (bounded_velocity / m_straight_cruise_speed) * 50.0f 
            : 0.0f;
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
    m_ipc_server.stop();
    if (m_event_pipe_fd >= 0) {
        close(m_event_pipe_fd);
        m_event_pipe_fd = -1;
    }
}

bool FSM::apply_config_key(const std::string& key, float value) {
    bool valid = false;
    if (key == "MAX_ACCELERATION") {
        m_max_acceleration = std::clamp(value, 0.1f, 1.5f);
        if (m_motion_profiler) m_motion_profiler->set_max_accel(m_max_acceleration);
        valid = true;
    } else if (key == "STRAIGHT_CRUISE_SPEED" || key == "CRUISE_SPEED") {
        m_straight_cruise_speed = std::clamp(value, 0.05f, 0.60f);
        if (m_line_controller) m_line_controller->set_cruise_speed(m_straight_cruise_speed);
        if (m_motion_profiler) m_motion_profiler->set_max_speed(m_straight_cruise_speed);
        valid = true;
    } else if (key == "MIN_CURVE_SPEED") {
        m_min_curve_speed = std::clamp(value, 0.05f, 0.40f);
        if (m_line_controller) m_line_controller->set_min_curve_speed(m_min_curve_speed);
        valid = true;
    } else if (key == "CURVE_SAFETY_FACTOR") {
        m_curve_safety_factor = std::clamp(value, 0.30f, 1.20f);
        valid = true;
    } else if (key == "PREDICTIVE_BRAKE_GAIN") {
        m_predictive_brake_gain = std::clamp(value, 0.0f, 0.30f);
        valid = true;
    } else if (key == "LINE_KP" || key == "KP") {
        m_line_kp = std::clamp(value, 0.0f, 2.0f);
        if (m_line_controller) m_line_controller->set_kp(m_line_kp);
        valid = true;
    } else if (key == "LINE_KD" || key == "KD") {
        m_line_kd = std::clamp(value, 0.0f, 1.0f);
        if (m_line_controller) m_line_controller->set_kd(m_line_kd);
        valid = true;
    } else if (key == "OBSTACLE_STOP_DIST_CM") {
        m_obstacle_stop_dist_cm = std::clamp(value, 10.0f, 80.0f);
        valid = true;
    } else if (key == "REVERSE_SPEED") {
        m_reverse_speed = std::clamp(value, 0.05f, 0.30f);
        valid = true;
    } else if (key == "AVOID_SIDE") {
        m_avoid_side = (value < 0.0f) ? -1.0f : 1.0f;
        valid = true;
    } else if (key == "AUTO_START_DELAY") {
        m_auto_start_delay = std::max(0.0f, value);
        valid = true;
    }

    if (!valid) {
        std::cerr << "[FSM] SetConfigKey rejeté : clé inconnue '" << key << "'" << std::endl;
        return false;
    }

    // 1. Mise à jour de la SHM si disponible
    if (m_telemetry) {
        pthread_mutex_lock(&(m_telemetry->mtx));
        if (key == "MAX_ACCELERATION") m_telemetry->cfg_max_acceleration = m_max_acceleration;
        else if (key == "STRAIGHT_CRUISE_SPEED" || key == "CRUISE_SPEED") m_telemetry->cfg_cruise_speed = m_straight_cruise_speed;
        else if (key == "MIN_CURVE_SPEED") m_telemetry->cfg_min_curve_speed = m_min_curve_speed;
        else if (key == "CURVE_SAFETY_FACTOR") m_telemetry->cfg_curve_safety_factor = m_curve_safety_factor;
        else if (key == "PREDICTIVE_BRAKE_GAIN") m_telemetry->cfg_predictive_brake_gain = m_predictive_brake_gain;
        else if (key == "LINE_KP" || key == "KP") m_telemetry->cfg_line_kp = m_line_kp;
        else if (key == "LINE_KD" || key == "KD") m_telemetry->cfg_line_kd = m_line_kd;
        m_telemetry->cfg_version++;
        m_current_cfg_version = m_telemetry->cfg_version;
        pthread_mutex_unlock(&(m_telemetry->mtx));
    }

    // 2. Persistance dans fsm.conf
    m_config.save_key(key, value);
    std::cout << "[FSM] Configuration mise à jour : " << key << " = " << value 
              << " (sauvegardé dans " << m_config.get_active_filepath() << ")" << std::endl;
    return true;
}

} // namespace services
