#include "SimBridge.hpp"
#include "control/picar_constants.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

namespace services {

SimBridge::SimBridge(int port, bool sim_mode)
    : m_running(false), m_port(port), m_sim_mode(sim_mode), m_tcp_server(port) {}

SimBridge::~SimBridge() {
    stop();
}

bool SimBridge::init() {
    std::cout << "[SimBridge] Initializing Bridge on TCP port " << m_port
              << " (mode: " << (m_sim_mode ? "SIMULATION" : "VEHICLE") << ")..." << std::endl;

    if (!m_shm_bridge.init()) {
        std::cerr << "[SimBridge] Failed to init SHM bridge!" << std::endl;
        return false;
    }

    if (!m_tcp_server.start()) {
        std::cerr << "[SimBridge] Failed to start TCP server!" << std::endl;
        return false;
    }

    m_running.store(true);
    return true;
}

void SimBridge::run() {
    uint64_t packet_count = 0;
    std::cout << "[SimBridge] Bridge running in RT mode." << std::endl;

    while (m_running.load(std::memory_order_relaxed)) {
        if (!m_tcp_server.poll_client()) {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
            continue;
        }

        std::string line;
        if (!m_tcp_server.read_line(line)) {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
            continue;
        }

        if (line.empty()) continue;

        std::string ack;
        bool req_emergency = false;
        bool req_start = false;
        bool is_config = false;
        std::string cfg_key;
        float cfg_val = 0.0f;

        if (SimProtocolHandler::handle_text_command(line, ack, req_emergency, req_start, is_config, cfg_key, cfg_val)) {
            if (req_start) {
                m_shm_bridge.forward_start_to_fifo();
                m_shm_bridge.set_emergency_stop(false);
            }
            if (req_emergency) {
                m_shm_bridge.set_emergency_stop(true, 0.0f);
            }
            if (is_config) {
                m_shm_bridge.apply_config_update(cfg_key, cfg_val);
            }
            m_tcp_server.send_response(ack);
            continue;
        }

        GodotTelemetryIn telem_in{};
        if (SimProtocolHandler::parse_telemetry(line, telem_in)) {
            packet_count++;
            if (m_sim_mode) {
                m_shm_bridge.inject_sim_telemetry(telem_in);
            }

            float target_spd = 0.0f;
            float target_steer = 0.0f;
            bool emergency = false;
            float cruise_cfg = 0.40f;
            m_shm_bridge.read_control_targets(target_spd, target_steer, emergency, cruise_cfg);

            GodotCommandOut cmd_out{};
            // Même relation puissance -> vitesse que sur le robot (50 % = ~0,32 m/s), quelle que soit la croisière configurée
            cmd_out.velocity = (target_spd / 50.0f) * picar::constants::PHYSICAL_SPEED_AT_HALF_THROTTLE;
            cmd_out.direction = -std::clamp(target_steer / 45.0f, -1.0f, 1.0f);
            cmd_out.brake = emergency;

            if (!m_sim_mode) {
                float v_dist = 100.0f;
                uint8_t v_lines[5] = {0};
                m_shm_bridge.read_vehicle_sensors(v_dist, v_lines);
                cmd_out.distance = v_dist;
                for (int i = 0; i < 5; ++i) cmd_out.line_sensors[i] = v_lines[i];
                cmd_out.has_sensors = true;
            }

            std::string response = SimProtocolHandler::serialize_command(cmd_out);
            m_tcp_server.send_response(response);

            if (packet_count % 1000 == 0) {
                std::cout << "[SimBridge RT] [" << (m_sim_mode ? "SIM" : "VEHICLE")
                          << "] Packets: " << packet_count << " | Vel: " << cmd_out.velocity
                          << " | Steer: " << cmd_out.direction << std::endl;
            }
        }
    }

    m_tcp_server.stop();
    std::cout << "[SimBridge] Service stopped." << std::endl;
}

void SimBridge::stop() {
    if (m_running.load()) {
        m_running.store(false);
        m_tcp_server.stop();
    }
}

} // namespace services
