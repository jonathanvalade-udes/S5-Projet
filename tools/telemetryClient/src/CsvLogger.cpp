#include "CsvLogger.hpp"

#include <iostream>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace telemetry {

static inline float safe_val(float val, float fallback = 0.0f) {
    if (std::isnan(val) || std::isinf(val)) {
        return fallback;
    }
    return val;
}

CsvLogger::CsvLogger() = default;

CsvLogger::~CsvLogger() {
    close();
}

bool CsvLogger::open(const std::string& filepath) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_is_open.load()) {
        m_file.close();
    }

    m_filepath = filepath;
    m_file.open(m_filepath, std::ios::out | std::ios::app);
    if (!m_file.is_open()) {
        std::cerr << "[CsvLogger] Erreur: Impossible d'ouvrir le fichier CSV : " << m_filepath << std::endl;
        m_is_open.store(false);
        return false;
    }

    // Si le fichier est vide, écrire l'en-tête
    m_file.seekp(0, std::ios::end);
    if (m_file.tellp() == 0) {
        write_header();
    }

    m_is_open.store(true);
    std::cout << "[CsvLogger] Enregistrement CSV activé : " << m_filepath << std::endl;
    return true;
}

void CsvLogger::write_header() {
    m_file << "timestamp_ms,"
           << "seq_number,"
           // FSM
           << "fsm_current_state,"
           << "fsm_requested_state,"
           << "fsm_flags,"
           << "fsm_error_code,"
           // Sensors - Ultrasons
           << "sonar_raw_dist_cm,"
           << "sonar_calc_dist_cm,"
           << "sonar_flags,"
           << "sonar_error_code,"
           // Sensors - Suiveur de ligne (5 capteurs)
           << "line_flags,"
           << "s0_raw,s0_norm,s0_corr,s0_err,"
           << "s1_raw,s1_norm,s1_corr,s1_err,"
           << "s2_raw,s2_norm,s2_corr,s2_err,"
           << "s3_raw,s3_norm,s3_corr,s3_err,"
           << "s4_raw,s4_norm,s4_corr,s4_err,"
           << "sensors_comp_flags,"
           << "sensors_comp_err,"
           // Moteurs & Direction
           << "motors_state,"
           << "motors_comp_flags,"
           << "motors_error_code,"
           // Moteur gauche
           << "m_left_target_throttle,"
           << "m_left_applied_throttle,"
           << "m_left_encoder,"
           << "m_left_flags,"
           << "m_left_pid_flags,"
           << "m_left_kp,m_left_kd,m_left_ki,"
           // Moteur droit
           << "m_right_target_throttle,"
           << "m_right_applied_throttle,"
           << "m_right_encoder,"
           << "m_right_flags,"
           << "m_right_pid_flags,"
           << "m_right_kp,m_right_kd,m_right_ki,"
           // Servomoteur Direction (intégré motors)
           << "steering_target_angle,"
           << "steering_current_angle,"
           << "steering_target_ccr,"
           << "steering_current_ccr,"
           << "steering_coeff,"
           << "steering_flags,"
           << "steering_error_code,"
           // Servomoteur Auxiliaire
           << "aux_servo_target_angle,"
           << "aux_servo_current_angle,"
           << "aux_servo_target_ccr,"
           << "aux_servo_current_ccr,"
           << "aux_servo_coeff,"
           << "aux_servo_flags,"
           << "aux_servo_error_code,"
           // Services & Daemons
           << "services_flags,"
           << "services_timestamp,"
           << "services_handler_err,"
           << "daemons_count,"
           << "daemon1_status,daemon1_err,"
           << "daemon2_status,daemon2_err,"
           << "daemon3_status,daemon3_err,"
           << "daemon4_status,daemon4_err\n";
    m_file.flush();
}

void CsvLogger::log_record(uint16_t seq, const shared::car::CarTelemetry& telem) {
    if (!m_is_open.load()) return;

    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4);

    ss << now_ms << ","
       << seq << ","
       // FSM
       << static_cast<int>(telem.fsm.currentState) << ","
       << static_cast<int>(telem.fsm.requestedState) << ","
       << telem.fsm.flags << ","
       << telem.fsm.errorCode << ","
       // Ultrasons
       << safe_val(telem.sensors.ultrasonicSensor.rawDistanceCm) << ","
       << safe_val(telem.sensors.ultrasonicSensor.calculatedDistance) << ","
       << telem.sensors.ultrasonicSensor.flags << ","
       << telem.sensors.ultrasonicSensor.errorCode << ","
       // Ligne
       << telem.sensors.lineFollower.flags << ",";

    for (int i = 0; i < 5; ++i) {
        const auto& s = telem.sensors.lineFollower.sensorsData[i];
        ss << s.rawADCValue << ","
           << safe_val(s.normalizedValue) << ","
           << safe_val(s.correctorValue) << ","
           << s.errorCode << ",";
    }

    ss << telem.sensors.flags << ","
       << telem.sensors.errorCode << ","
       // Moteurs
       << static_cast<int>(telem.motors.state) << ","
       << telem.motors.componentFlags << ","
       << telem.motors.errorCode << ","
       // Moteur Gauche
       << safe_val(telem.motors.leftMotor.normalizedMotorThrottleTargetted) << ","
       << safe_val(telem.motors.leftMotor.normalizedMotorThrottleApplied) << ","
       << telem.motors.leftMotor.motorEncoder << ","
       << static_cast<int>(telem.motors.leftMotor.motorFlags) << ","
       << static_cast<int>(telem.motors.leftMotor.pidFlags) << ","
       << safe_val(telem.motors.leftMotor.pidInfo.powerCoefficient) << ","
       << safe_val(telem.motors.leftMotor.pidInfo.derivativeCoefficient) << ","
       << safe_val(telem.motors.leftMotor.pidInfo.integralCoefficient) << ","
       // Moteur Droit
       << safe_val(telem.motors.rightMotor.normalizedMotorThrottleTargetted) << ","
       << safe_val(telem.motors.rightMotor.normalizedMotorThrottleApplied) << ","
       << telem.motors.rightMotor.motorEncoder << ","
       << static_cast<int>(telem.motors.rightMotor.motorFlags) << ","
       << static_cast<int>(telem.motors.rightMotor.pidFlags) << ","
       << safe_val(telem.motors.rightMotor.pidInfo.powerCoefficient) << ","
       << safe_val(telem.motors.rightMotor.pidInfo.derivativeCoefficient) << ","
       << safe_val(telem.motors.rightMotor.pidInfo.integralCoefficient) << ","
       // Direction (Servo intégré)
       << safe_val(telem.motors.steeringServo.normalizedTargettedAngle) << ","
       << safe_val(telem.motors.steeringServo.normalizedCurrentAngle) << ","
       << telem.motors.steeringServo.targetedCCR << ","
       << telem.motors.steeringServo.currentCCR << ","
       << safe_val(telem.motors.steeringServo.coefficient) << ","
       << telem.motors.steeringServo.flags << ","
       << telem.motors.steeringServo.errorCode << ","
       // Aux Servo
       << safe_val(telem.servo.normalizedTargettedAngle) << ","
       << safe_val(telem.servo.normalizedCurrentAngle) << ","
       << telem.servo.targetedCCR << ","
       << telem.servo.currentCCR << ","
       << safe_val(telem.servo.coefficient) << ","
       << telem.servo.flags << ","
       << telem.servo.errorCode << ","
       // Services
       << telem.services.flags << ","
       << telem.services.timestamp << ","
       << telem.services.handlerErrorCode << ","
       << telem.services.deamonsLenght << ","
       << static_cast<int>(telem.services.daemons[0].status) << ","
       << static_cast<int>(telem.services.daemons[0].errorCode) << ","
       << static_cast<int>(telem.services.daemons[1].status) << ","
       << static_cast<int>(telem.services.daemons[1].errorCode) << ","
       << static_cast<int>(telem.services.daemons[2].status) << ","
       << static_cast<int>(telem.services.daemons[2].errorCode) << ","
       << static_cast<int>(telem.services.daemons[3].status) << ","
       << static_cast<int>(telem.services.daemons[3].errorCode) << "\n";

    std::string line = ss.str();

    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_file.is_open()) {
        m_file.write(line.data(), line.size());
        m_record_count.fetch_add(1, std::memory_order_relaxed);
        m_unflushed_count++;

        // Flush périodique par bloc de 50 trames (environ toutes les 500ms à 100Hz)
        if (m_unflushed_count >= 50) {
            m_file.flush();
            m_unflushed_count = 0;
        }
    }
}

void CsvLogger::flush() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_file.is_open()) {
        m_file.flush();
        m_unflushed_count = 0;
    }
}

void CsvLogger::close() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_is_open.load()) {
        m_file.flush();
        m_file.close();
        m_is_open.store(false);
        std::cout << "[CsvLogger] Fichier CSV fermé avec succès. Total enregistrements: " 
                  << m_record_count.load() << std::endl;
    }
}

} // namespace telemetry

