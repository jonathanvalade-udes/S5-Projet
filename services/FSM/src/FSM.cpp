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

<<<<<<< Updated upstream
=======
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
    // Vitesse RÉELLE : la vitesse du FSM est convertie en puissance moteur avec la croisière = 50 %, donc elle ne vaut pas
    // des m/s réels (sur le robot, 50 % de puissance = ~0,32 m/s, que la croisière configurée soit 0,35 ou 0,45).
    const float odo_scale = picar::constants::PHYSICAL_SPEED_AT_HALF_THROTTLE / std::max(m_straight_cruise_speed, 0.05f);
    const float v = (m_motion_profiler ? m_motion_profiler->get_current_speed() : 0.0f) * odo_scale;
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

>>>>>>> Stashed changes
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
