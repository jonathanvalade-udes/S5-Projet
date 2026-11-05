#include "FsmShmBridge.hpp"
#include <iostream>

namespace services {

bool FsmShmBridge::init(shared::fsm::MachineStates initial_state) {
    try {
        m_sensor_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("Sensors");
        m_cmd_pub = std::make_unique<shared::services::memory::MemoryShareComponentServer>("FsmCommand");
        m_fsm_pub = std::make_unique<shared::services::memory::MemoryShareComponentServer>("FSM");
    } catch (const std::exception& e) {
        std::cerr << "[FsmShmBridge] Warning: MemoryShare subscriber/publisher initialization: " << e.what() << std::endl;
    }

    publish_control_commands(0.0f, 0.0f, false, initial_state);
    return true;
}

bool FsmShmBridge::read_sensors(float& dist, uint8_t line_sensors[5], float line_darkness[5],
                                bool& emergency, bool& sensors_alive) {
    emergency = false;
    if (!m_sensor_sub) {
        sensors_alive = false;
        m_heartbeat_starvation_cycles++;
        return false;
    }

    shared::sensors::SensorsTelemetry sensors_data{};
    if (!m_sensor_sub->read(sensors_data)) {
        sensors_alive = false;
        m_heartbeat_starvation_cycles++;
        return false;
    }

    dist = sensors_data.ultrasonicSensor.calculatedDistance;
    for (int i = 0; i < 5; ++i) {
        line_darkness[i] = sensors_data.lineFollower.sensorsData[i].normalizedValue;
        line_sensors[i] = (line_darkness[i] > 0.5f) ? 1 : 0;
    }

    sensors_alive = true;
    m_heartbeat_starvation_cycles = 0;
    return true;
}

bool FsmShmBridge::check_watchdog(int max_cycles) {
    return (m_heartbeat_starvation_cycles > max_cycles);
}

void FsmShmBridge::publish_control_commands(float speed_pct, float steering_deg, bool emergency,
                                           shared::fsm::MachineStates state, float steering_offset) {
    if (m_cmd_pub && m_cmd_pub->isValid()) {
        shared::fsm::FsmCommandPayload cmd{};
        cmd.target_speed = speed_pct;
        cmd.target_steering = steering_deg;
        cmd.steering_offset = steering_offset;
        cmd.emergency_stop = emergency ? 1 : 0;
        cmd.reserved[0] = cmd.reserved[1] = cmd.reserved[2] = 0;
        cmd.heartbeat = ++m_fsm_heartbeat;
        cmd.state = state;
        cmd.state_reserved[0] = cmd.state_reserved[1] = cmd.state_reserved[2] = 0;

        m_cmd_pub->publish(cmd);
    }

    publish_fsm_telemetry(state);
}

void FsmShmBridge::publish_fsm_telemetry(shared::fsm::MachineStates state) {
    if (m_fsm_pub && m_fsm_pub->isValid()) {
        shared::fsm::FSMTelemetry telem{};
        telem.currentState = state;
        telem.requestedState = state;
        telem.flags = shared::fsm::flags::DISCOVERABLE;
        telem.errorCode = 0;
        m_fsm_pub->publish(telem);
    }
}

void FsmShmBridge::emergency_stop_shm() {
    publish_control_commands(0.0f, 0.0f, true, shared::fsm::MachineStates::Abort, 0.0f);
}

} // namespace services
