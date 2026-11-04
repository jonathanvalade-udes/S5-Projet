#include "ShmCollector.hpp"
#include <cmath>

namespace services {

ShmCollector::ShmCollector(int hz) : m_hz(hz > 0 ? hz : 50) {}

ShmCollector::~ShmCollector() = default;

bool ShmCollector::init() {
    // 1. Open POSIX SHM read-only if present (fallback)
    try {
        m_shm_mgr = std::make_unique<SharedMemoryManager>();
        m_telemetry = m_shm_mgr->open_shm(false);
    } catch (...) {
        m_telemetry = nullptr;
    }

    // 2. Open subscribers for other services
    try {
        m_sensor_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("Sensors");
    } catch (...) {}

    try {
        m_fsm_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("FSM");
    } catch (...) {}

    try {
        m_motor_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("Motors");
    } catch (...) {}

    try {
        m_servo_sub = std::make_unique<shared::services::memory::MemoryShareComponentSubscriber>("Servo");
    } catch (...) {}

    m_start_time = std::chrono::steady_clock::now();
    return true;
}

void ShmCollector::collect(float dt) {
    // 1. Read shared memory regions of other services
    shared::sensors::SensorsTelemetry sensors_data{};
    bool has_sensors = m_sensor_sub && m_sensor_sub->read(sensors_data);

    shared::motors::MotorsComponentTelemetry motors_data{};
    bool has_motors = m_motor_sub && m_motor_sub->read(motors_data);

    shared::servos::ServoTelemetry servo_data{};
    bool has_servo = m_servo_sub && m_servo_sub->read(servo_data);

    shared::fsm::FSMTelemetry fsm_data{};
    bool has_fsm = m_fsm_sub && m_fsm_sub->read(fsm_data);

    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (has_sensors) {
            m_car_telemetry.sensors = sensors_data;
            m_metrics.filtered_dist_cm = sensors_data.ultrasonicSensor.calculatedDistance;
            for (int i = 0; i < 5; ++i) {
                m_metrics.line_sensors[i] = (sensors_data.lineFollower.sensorsData[i].normalizedValue > 0.5f ||
                                             sensors_data.lineFollower.sensorsData[i].rawADCValue > 500) ? 1 : 0;
            }
        }

        if (has_motors) {
            m_car_telemetry.motors = motors_data;
            m_metrics.target_speed_percent = motors_data.leftMotor.normalizedMotorThrottleApplied * 100.0f;
        }

        if (has_servo) {
            m_car_telemetry.servo = servo_data;
            m_metrics.target_steering_deg = servo_data.normalizedCurrentAngle * shared::servos::MAX_ANGLE;
        } else if (has_motors) {
            m_car_telemetry.servo = motors_data.steeringServo;
            m_metrics.target_steering_deg = motors_data.steeringServo.normalizedCurrentAngle * shared::servos::MAX_ANGLE;
        }

        if (has_fsm) {
            m_car_telemetry.fsm = fsm_data;
            m_metrics.fsm_state = static_cast<uint32_t>(fsm_data.currentState);
        }

        // 2. Fallback: Read POSIX SHM if services did not publish to individual memory regions
        if (!m_telemetry && m_shm_mgr) {
            try {
                m_telemetry = m_shm_mgr->open_shm(false);
            } catch (...) {
                m_telemetry = nullptr;
            }
        }

        if (m_telemetry) {
            pthread_mutex_lock(&(m_telemetry->mtx));
            if (!has_sensors) {
                m_metrics.filtered_dist_cm = m_telemetry->filtered_dist;
                for (int i = 0; i < 5; ++i) {
                    m_metrics.line_sensors[i] = m_telemetry->line_sensors[i];
                    m_car_telemetry.sensors.lineFollower.sensorsData[i].rawADCValue = m_telemetry->line_sensors[i] ? 1023 : 0;
                    m_car_telemetry.sensors.lineFollower.sensorsData[i].normalizedValue = m_telemetry->line_sensors[i] ? 1.0f : 0.0f;
                }
                m_car_telemetry.sensors.ultrasonicSensor.calculatedDistance = m_metrics.filtered_dist_cm;
                m_car_telemetry.sensors.ultrasonicSensor.rawDistanceCm = m_metrics.filtered_dist_cm;
                m_car_telemetry.sensors.flags = shared::sensors::flags::DISCOVERABLE;
            }
            if (!has_motors) {
                m_metrics.target_speed_percent = m_telemetry->target_speed;
                m_car_telemetry.motors.leftMotor.normalizedMotorThrottleApplied = m_metrics.target_speed_percent / 100.0f;
                m_car_telemetry.motors.rightMotor.normalizedMotorThrottleApplied = m_metrics.target_speed_percent / 100.0f;
                m_car_telemetry.motors.leftMotor.normalizedMotorThrottleTargetted = m_metrics.target_speed_percent / 100.0f;
                m_car_telemetry.motors.rightMotor.normalizedMotorThrottleTargetted = m_metrics.target_speed_percent / 100.0f;
                m_car_telemetry.motors.state = (std::abs(m_metrics.target_speed_percent) > 1.0f)
                                               ? shared::motors::MotorStates::Running
                                               : shared::motors::MotorStates::Idle;
                m_car_telemetry.motors.componentFlags = shared::motors::flags::component::DISCOVERABLE;
            }
            if (!has_servo) {
                m_metrics.target_steering_deg = m_telemetry->target_steering;
                m_car_telemetry.servo.normalizedCurrentAngle = m_metrics.target_steering_deg / shared::servos::MAX_ANGLE;
                m_car_telemetry.servo.normalizedTargettedAngle = m_metrics.target_steering_deg / shared::servos::MAX_ANGLE;
                m_car_telemetry.servo.flags = shared::servos::flags::RDY;
                m_car_telemetry.motors.steeringServo = m_car_telemetry.servo;
            }
            if (!has_fsm) {
                m_metrics.fsm_state = m_telemetry->state;
                m_car_telemetry.fsm.currentState = static_cast<shared::fsm::MachineStates>(m_metrics.fsm_state);
                m_car_telemetry.fsm.requestedState = m_car_telemetry.fsm.currentState;
                m_car_telemetry.fsm.flags = shared::fsm::flags::RDY;
            }
            m_metrics.emergency_stop = m_telemetry->emergency_stop;
            m_metrics.heartbeat = m_telemetry->heartbeat;
            m_metrics.steering_offset_deg = m_telemetry->steering_offset;
            pthread_mutex_unlock(&(m_telemetry->mtx));
        }

        // 3. Update service flags
        m_car_telemetry.services.flags = shared::services::flags::SYSTEM_STARTED |
                                         shared::services::flags::SHARE_MEMORY_MOUNTED |
                                         shared::services::flags::SOCKET_ENABLED |
                                         shared::services::flags::DISCOVERABLE;
        if (m_metrics.watchdog_stalled) {
            m_car_telemetry.services.flags |= shared::services::flags::PROCESS_STALLED;
            m_car_telemetry.services.flags |= shared::services::flags::ANOMALIES_DETECTED;
            m_car_telemetry.fsm.flags |= shared::fsm::flags::STALLED;
        }
        if (m_metrics.accel_violations > 0 || m_metrics.emergency_stop) {
            m_car_telemetry.services.flags |= shared::services::flags::ANOMALIES_DETECTED;
        }

        auto now = std::chrono::steady_clock::now().time_since_epoch();
        m_car_telemetry.services.timestamp = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
        m_car_telemetry.services.handlerErrorCode = 0;
        m_car_telemetry.services.deamonsLenght = 4;

        m_car_telemetry.services.daemons[0].id = 1;
        m_car_telemetry.services.daemons[0].status = has_sensors ? 1 : 0;
        m_car_telemetry.services.daemons[1].id = 2;
        m_car_telemetry.services.daemons[1].status = has_motors ? 1 : 0;
        m_car_telemetry.services.daemons[2].id = 3;
        m_car_telemetry.services.daemons[2].status = (has_servo || has_motors) ? 1 : 0;
        m_car_telemetry.services.daemons[3].id = 4;
        m_car_telemetry.services.daemons[3].status = has_fsm ? 1 : 0;
    }

    // 4. Update dynamic metrics
    m_metrics.current_speed_ms = (m_metrics.target_speed_percent / 50.0f) * 0.32f;
    m_metrics.current_accel_ms2 = (m_metrics.current_speed_ms - m_last_speed_ms) / dt;

    if (std::abs(m_metrics.current_accel_ms2) > 0.785f) {
        m_metrics.accel_violations++;
    }
    m_last_speed_ms = m_metrics.current_speed_ms;

    m_metrics.total_distance_m += std::abs(m_metrics.current_speed_ms) * dt;

    if (m_metrics.heartbeat == m_last_heartbeat) {
        m_heartbeat_stalled_cycles++;
        if (m_heartbeat_stalled_cycles >= (m_hz / 2)) {
            m_metrics.watchdog_stalled = true;
        }
    } else {
        m_heartbeat_stalled_cycles = 0;
        m_metrics.watchdog_stalled = false;
        m_last_heartbeat = m_metrics.heartbeat;
    }

    auto now_t = std::chrono::steady_clock::now();
    m_metrics.uptime_seconds = std::chrono::duration_cast<std::chrono::seconds>(now_t - m_start_time).count();
}

shared::car::CarTelemetry ShmCollector::get_telemetry() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_car_telemetry;
}

TelemetrySample ShmCollector::get_sample() const {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    uint64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(now).count();

    TelemetrySample sample{};
    sample.timestamp_us = now_us;
    sample.uptime_s = static_cast<uint32_t>(m_metrics.uptime_seconds);
    sample.fsm_state = m_metrics.fsm_state;
    sample.speed_ms = m_metrics.current_speed_ms;
    sample.accel_ms2 = m_metrics.current_accel_ms2;
    sample.steer_deg = m_metrics.target_steering_deg;
    sample.steer_offset_deg = m_metrics.steering_offset_deg;
    sample.dist_us_cm = m_metrics.filtered_dist_cm;
    sample.line_sensors[0] = m_metrics.line_sensors[0];
    sample.line_sensors[1] = m_metrics.line_sensors[1];
    sample.line_sensors[2] = m_metrics.line_sensors[2];
    sample.line_sensors[3] = m_metrics.line_sensors[3];
    sample.line_sensors[4] = m_metrics.line_sensors[4];
    sample.emergency_stop = m_metrics.emergency_stop ? 1 : 0;
    sample.accel_violations = static_cast<uint16_t>(m_metrics.accel_violations);
    sample.odometry_m = m_metrics.total_distance_m;
    return sample;
}

} // namespace services
