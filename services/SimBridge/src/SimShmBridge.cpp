#include "SimShmBridge.hpp"
#include "control/picar_config.hpp"
#include "shared-services/socket/service_ipc_socket.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <cstring>
#include <iostream>

namespace services {

SimShmBridge::SimShmBridge() = default;

bool SimShmBridge::init() {
    try {
        m_sensor_pub = std::make_unique<shared::services::memory::MemoryShareComponentServer>("Sensors");
        m_motor_pub = std::make_unique<shared::services::memory::MemoryShareComponentServer>("Motors");
        m_fsm_cmd_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("FsmCommand");
        m_sensor_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("Sensors");
    } catch (const std::exception& e) {
        std::cerr << "[SimShmBridge] Warning: Shared memory setup: " << e.what() << std::endl;
        return false;
    }
    reset_pipeline(100.0f);
    return true;
}

void SimShmBridge::reset_pipeline(float dist) {
    m_us_pipeline.reset(dist);
}

void SimShmBridge::inject_sim_telemetry(const GodotTelemetryIn& telem) {
    float filtered_dist = m_us_pipeline.process(telem.distance, 0.02f, m_last_speed);

    if (m_sensor_pub && m_sensor_pub->isValid()) {
        shared::sensors::SensorsTelemetry sensors_telem{};
        sensors_telem.ultrasonicSensor.calculatedDistance = filtered_dist;
        sensors_telem.ultrasonicSensor.rawDistanceCm = filtered_dist;
        sensors_telem.ultrasonicSensor.flags = shared::sensors::ultrasonic::flags::SENSOR_ACTIVATED;

        // Godot est en miroir de la convention du robot (HAL : braquage + = droite, capteur D = droite)
        for (int i = 0; i < 5; ++i) {
            const int j = 4 - i;
            sensors_telem.lineFollower.sensorsData[i].normalizedValue = std::clamp(telem.line_darkness[j], 0.0f, 1.0f);
            sensors_telem.lineFollower.sensorsData[i].rawADCValue = telem.line_sensors[j] ? 1023 : 0;
        }
        sensors_telem.lineFollower.flags = shared::sensors::lineFollower::flags::LINE_FOLLOWED;
        sensors_telem.flags = shared::sensors::flags::DISCOVERABLE;

        m_sensor_pub->publish(sensors_telem);
    }
}

void SimShmBridge::read_control_targets(float& target_spd, float& target_steer, bool& emergency, float& cruise_cfg) {
    target_spd = 0.0f;
    target_steer = 0.0f;
    emergency = false;
    cruise_cfg = 0.40f;

    if (m_fsm_cmd_sub) {
        shared::fsm::FsmCommandPayload cmd{};
        if (m_fsm_cmd_sub->read(cmd)) {
            target_spd = cmd.target_speed;
            target_steer = cmd.target_steering;
            emergency = (cmd.emergency_stop != 0);
        }
    }

    m_last_speed = target_spd;

    if (m_motor_pub && m_motor_pub->isValid()) {
        shared::motors::MotorsComponentTelemetry motors_telem{};
        motors_telem.leftMotor.normalizedMotorThrottleApplied = target_spd / 100.0f;
        motors_telem.rightMotor.normalizedMotorThrottleApplied = target_spd / 100.0f;
        motors_telem.componentFlags = shared::motors::flags::component::DISCOVERABLE;
        m_motor_pub->publish(motors_telem);
    }
}

void SimShmBridge::read_vehicle_sensors(float& dist, uint8_t line_sensors[5]) {
    dist = 100.0f;
    for (int i = 0; i < 5; ++i) line_sensors[i] = 0;

    if (m_sensor_sub) {
        shared::sensors::SensorsTelemetry sensors_telem{};
        if (m_sensor_sub->read(sensors_telem)) {
            dist = sensors_telem.ultrasonicSensor.calculatedDistance;
            for (int i = 0; i < 5; ++i) {
                line_sensors[i] = (sensors_telem.lineFollower.sensorsData[i].normalizedValue > 0.5f) ? 1 : 0;
            }
        }
    }
}

void SimShmBridge::set_emergency_stop(bool emergency, float /*target_spd*/) {
    if (emergency) {
        shared::services::ipc::ServiceIPCSocketClient::send_emergency_stop(
            "/tmp/fsm.sock", shared::commands::ServiceID::FSM, 1, 0, 50);
        shared::services::ipc::ServiceIPCSocketClient::send_emergency_stop(
            "/tmp/motors.sock", shared::commands::ServiceID::MOTORS, 1, 0, 50);
    }
}

void SimShmBridge::apply_config_update(const std::string& key, float val) {
    config::PicarConfig::write_config(key, val);
    shared::services::ipc::ServiceIPCSocketClient::send_set_config(
        "/tmp/fsm.sock", shared::commands::ServiceID::FSM, key, val, 0, 50);
    shared::services::ipc::ServiceIPCSocketClient::send_set_config(
        "/tmp/motors.sock", shared::commands::ServiceID::MOTORS, key, val, 0, 50);
}

void SimShmBridge::forward_start_to_fifo() {
    int fifo = ::open("/tmp/picar_start", O_WRONLY | O_NONBLOCK);
    if (fifo >= 0) {
        const char msg = '1';
        ::write(fifo, &msg, 1);
        ::close(fifo);
    }
    shared::services::ipc::ServiceIPCSocketClient::send_set_active(
        "/tmp/fsm.sock", shared::commands::ServiceID::FSM, true, 0, 50);
}

} // namespace services
