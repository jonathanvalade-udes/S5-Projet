#include "TelemetryService.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <cmath>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

namespace services {

static const char* state_to_string(uint32_t state) {
    switch (static_cast<shared::fsm::MachineStates>(state)) {
        case shared::fsm::MachineStates::Init: return "Init";
        case shared::fsm::MachineStates::Unarmed: return "Unarmed";
        case shared::fsm::MachineStates::Armed: return "Armed";
        case shared::fsm::MachineStates::LineFollowingForward: return "LineFollowingForward";
        case shared::fsm::MachineStates::CollisionRisk: return "CollisionRisk";
        case shared::fsm::MachineStates::Stopped: return "Stopped";
        case shared::fsm::MachineStates::CollisionOccured: return "CollisionOccured";
        case shared::fsm::MachineStates::RightDeviation: return "RightDeviation";
        case shared::fsm::MachineStates::LeftDeviation: return "LeftDeviation";
        case shared::fsm::MachineStates::Reverse: return "Reverse";
        case shared::fsm::MachineStates::LineSearching: return "LineSearching";
        case shared::fsm::MachineStates::LineDetected: return "LineDetected";
        case shared::fsm::MachineStates::LineSearchingTimedOut: return "LineSearchingTimedOut";
        case shared::fsm::MachineStates::Abort: return "Abort";
        case shared::fsm::MachineStates::Failure: return "Failure";
        default: return "Undefined";
    }
}

TelemetryService::TelemetryService(int hz, bool enable_ui, int udp_port)
    : m_running(false), m_hz(hz), m_enable_ui(enable_ui), m_udp_port(udp_port) {}

TelemetryService::~TelemetryService() {
    stop();
    if (m_udp_fd >= 0) {
        close(m_udp_fd);
        m_udp_fd = -1;
    }
}

bool TelemetryService::init() {
    std::cout << "[TelemetryService] Initializing hub at " << m_hz << " Hz (UDP port " << m_udp_port << ")..." << std::endl;

    try {
        m_shm_mgr = std::make_unique<SharedMemoryManager>();
        m_telemetry = m_shm_mgr->open_shm(true);
    } catch (const std::exception& e) {
        std::cerr << "[TelemetryService] Warning: Failed to open POSIX SHM: " << e.what() << std::endl;
        m_telemetry = nullptr;
    }

    try {
        m_telemetry_pub = std::make_unique<shared::services::memory::MemoryShareComponentServer>("Telemetry");
        m_sensor_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("Sensors");
        m_fsm_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("FSM");
        m_motor_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("Motors");
    } catch (const std::exception& e) {
        std::cerr << "[TelemetryService] Warning: MemoryShare init: " << e.what() << std::endl;
    }

    // Initialize UDP broadcast socket
    if (m_udp_port > 0) {
        m_udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_udp_fd >= 0) {
            int broadcast = 1;
            setsockopt(m_udp_fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
            int flags = fcntl(m_udp_fd, F_GETFL, 0);
            fcntl(m_udp_fd, F_SETFL, flags | O_NONBLOCK);
        }
    }

    m_start_time = std::chrono::steady_clock::now();
    m_running.store(true);
    return true;
}

void TelemetryService::update_metrics(float dt) {
    if (m_telemetry) {
        pthread_mutex_lock(&(m_telemetry->mtx));
        m_metrics.filtered_dist_cm = m_telemetry->filtered_dist;
        m_metrics.target_speed_percent = m_telemetry->target_speed;
        m_metrics.target_steering_deg = m_telemetry->target_steering;
        m_metrics.fsm_state = m_telemetry->state;
        m_metrics.emergency_stop = m_telemetry->emergency_stop;
        m_metrics.heartbeat = m_telemetry->heartbeat;
        for (int i = 0; i < 5; ++i) {
            m_metrics.line_sensors[i] = m_telemetry->line_sensors[i];
        }
        pthread_mutex_unlock(&(m_telemetry->mtx));
    }

    // Calcul vitesse en m/s (50% = 0.32 m/s vitesse nominale)
    m_metrics.current_speed_ms = (m_metrics.target_speed_percent / 50.0f) * 0.32f;

    // Calcul accélération a = dv/dt
    m_metrics.current_accel_ms2 = (m_metrics.current_speed_ms - m_last_speed_ms) / dt;

    // Détection dépassement seuil critique FastDrink (0.735 m/s² + tolérance discrétisation 0.05)
    if (std::abs(m_metrics.current_accel_ms2) > 0.785f) {
        m_metrics.accel_violations++;
    }
    m_last_speed_ms = m_metrics.current_speed_ms;

    // Intégration odométrique
    m_metrics.total_distance_m += std::abs(m_metrics.current_speed_ms) * dt;

    // Watchdog de vivacité (heartbeat)
    if (m_metrics.heartbeat == m_last_heartbeat) {
        m_heartbeat_stalled_cycles++;
        if (m_heartbeat_stalled_cycles >= (m_hz / 2)) { // >= 500 ms de stagnation
            m_metrics.watchdog_stalled = true;
        }
    } else {
        m_heartbeat_stalled_cycles = 0;
        m_metrics.watchdog_stalled = false;
        m_last_heartbeat = m_metrics.heartbeat;
    }

    auto now = std::chrono::steady_clock::now();
    m_metrics.uptime_seconds = std::chrono::duration_cast<std::chrono::seconds>(now - m_start_time).count();
}

void TelemetryService::publish_ipc() {
    if (m_telemetry_pub && m_telemetry_pub->isValid()) {
        shared::car::CarTelemetry car_telem{};

        car_telem.fsm.currentState = static_cast<shared::fsm::MachineStates>(m_metrics.fsm_state);
        car_telem.fsm.requestedState = car_telem.fsm.currentState;
        car_telem.fsm.flags = m_metrics.watchdog_stalled ? shared::fsm::flags::STALLED : shared::fsm::flags::RDY;

        car_telem.sensors.ultrasonicSensor.calculatedDistance = m_metrics.filtered_dist_cm;
        for (int i = 0; i < 5; ++i) {
            car_telem.sensors.lineFollower.sensorsData[i].normalizedValue = m_metrics.line_sensors[i] ? 1.0f : 0.0f;
        }

        car_telem.servo.normalizedTargettedAngle = m_metrics.target_steering_deg / 45.0f;
        car_telem.servo.normalizedCurrentAngle = m_metrics.target_steering_deg / 45.0f;
        car_telem.motors.state = (std::abs(m_metrics.current_speed_ms) > 0.01f) 
                                 ? shared::motors::MotorStates::Running 
                                 : shared::motors::MotorStates::Idle;

        car_telem.services.flags = shared::services::flags::SYSTEM_STARTED | shared::services::flags::SHARE_MEMORY_MOUNTED;
        if (m_metrics.accel_violations > 0 || m_metrics.watchdog_stalled || m_metrics.emergency_stop) {
            car_telem.services.flags |= shared::services::flags::ANOMALIES_DETECTED;
        }
        if (m_metrics.watchdog_stalled) {
            car_telem.services.flags |= shared::services::flags::PROCESS_STALLED;
        }

        m_telemetry_pub->publish(car_telem);
    }
}

void TelemetryService::broadcast_udp() {
    if (m_udp_fd < 0 || m_udp_port <= 0) return;

    std::ostringstream json;
    json << std::fixed << std::setprecision(2);
    json << "{"
         << "\"uptime\":" << m_metrics.uptime_seconds << ","
         << "\"speed_ms\":" << m_metrics.current_speed_ms << ","
         << "\"speed_pct\":" << m_metrics.target_speed_percent << ","
         << "\"accel_ms2\":" << m_metrics.current_accel_ms2 << ","
         << "\"steer_deg\":" << m_metrics.target_steering_deg << ","
         << "\"dist_cm\":" << m_metrics.filtered_dist_cm << ","
         << "\"odometry_m\":" << m_metrics.total_distance_m << ","
         << "\"fsm_state\":\"" << state_to_string(m_metrics.fsm_state) << "\","
         << "\"line\":[" 
         << (int)m_metrics.line_sensors[0] << ","
         << (int)m_metrics.line_sensors[1] << ","
         << (int)m_metrics.line_sensors[2] << ","
         << (int)m_metrics.line_sensors[3] << ","
         << (int)m_metrics.line_sensors[4] << "],"
         << "\"heartbeat\":" << m_metrics.heartbeat << ","
         << "\"emergency\":" << (m_metrics.emergency_stop ? "true" : "false") << ","
         << "\"violations\":" << m_metrics.accel_violations
         << "}\n";

    std::string payload = json.str();

    struct sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(m_udp_port);
    dest_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    sendto(m_udp_fd, payload.data(), payload.size(), 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
}

void TelemetryService::render_dashboard() {
    std::string line_bar = "[";
    for (int i = 0; i < 5; ++i) {
        line_bar += m_metrics.line_sensors[i] ? "█" : "·";
        if (i < 4) line_bar += " ";
    }
    line_bar += "]";

    std::string accel_status = (std::abs(m_metrics.current_accel_ms2) <= 0.735f) ? "OK" : "ALERTE !";
    std::string watchdog_str = m_metrics.watchdog_stalled ? "STALLED !" : "ACTIF";

    std::cout << "\r\033[K"
              << "[TELEM] "
              << "FSM: " << std::left << std::setw(20) << state_to_string(m_metrics.fsm_state)
              << " | V: " << std::fixed << std::setprecision(2) << m_metrics.current_speed_ms << " m/s"
              << " | A: " << m_metrics.current_accel_ms2 << " m/s² (" << accel_status << ")"
              << " | Steer: " << m_metrics.target_steering_deg << "°"
              << " | US: " << m_metrics.filtered_dist_cm << " cm"
              << " | Line: " << line_bar
              << " | Odo: " << m_metrics.total_distance_m << " m"
              << " | WD: " << watchdog_str
              << std::flush;
}

void TelemetryService::run() {
    const auto period = std::chrono::milliseconds(1000 / m_hz);
    const float dt = 1.0f / static_cast<float>(m_hz);

    uint64_t loop_count = 0;
    std::cout << "[TelemetryService] Started hub running at " << m_hz << " Hz." << std::endl;

    while (m_running.load()) {
        auto start_time = std::chrono::steady_clock::now();

        update_metrics(dt);
        publish_ipc();
        broadcast_udp();

        if (m_enable_ui && (loop_count % m_ui_render_divider == 0)) {
            render_dashboard();
        }

        loop_count++;
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }

    std::cout << "\n[TelemetryService] Service loop terminated." << std::endl;
}

void TelemetryService::stop() {
    if (m_running.load()) {
        m_running.store(false);
    }
}

} // namespace services
