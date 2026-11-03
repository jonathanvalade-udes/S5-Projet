/**
 * picar_webots.cpp
 * Contrôleur C++ natif Webots pour le robot PiCar FastDrink S5-GI.
 * - Direction d'Ackermann 1:1 (Propulsion arrière + Servomoteurs de direction avant)
 * - Surveillance physique 3D de la bille libre (FastDrink S6)
 * - Asservissement PD + Motion Profiler a <= 0.735 m/s² avec gains Optuna
 */

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <memory>
#include <cmath>
#include <algorithm>

#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>

// Intégration directe des headers de notre firmware C++
#include "../../../firmware/include/filters/ultrasonic_pipeline.hpp"
#include "../../../firmware/include/control/motion_profiler.hpp"
#include "../../../firmware/include/control/line_follower.hpp"
#include "../../../firmware/include/control/optimized_params.hpp"
#include "../../../firmware/include/ipc/shared_memory.hpp"

using namespace webots;

int main() {
    auto robot = std::make_unique<Supervisor>();
    int timeStep = static_cast<int>(robot->getBasicTimeStep());
    if (timeStep <= 0) timeStep = 10;
    float dt = static_cast<float>(timeStep) / 1000.0f;

    std::cout << "==================================================" << std::endl;
    std::cout << " [WEBOTS] Contrôleur Ackermann 1:1 & Bille S6     " << std::endl;
    std::cout << " Pas de temps Webots : " << timeStep << " ms (" << (1000 / timeStep) << " Hz)" << std::endl;
    std::cout << " Paramètres chargés depuis Optuna :" << std::endl;
    std::cout << "  • Kp = " << Config::OPTIMIZED_KP << " | Kd = " << Config::OPTIMIZED_KD << std::endl;
    std::cout << "  • Vitesse croisière = " << Config::OPTIMIZED_CRUISE_SPEED << " m/s" << std::endl;
    std::cout << "==================================================" << std::endl;

    // 1. Initialisation des Moteurs de Propulsion Arrière
    Motor* rear_left_motor = robot->getMotor("rear_left_motor");
    Motor* rear_right_motor = robot->getMotor("rear_right_motor");
    if (rear_left_motor && rear_right_motor) {
        rear_left_motor->setPosition(INFINITY);
        rear_right_motor->setPosition(INFINITY);
        rear_left_motor->setVelocity(0.0);
        rear_right_motor->setVelocity(0.0);
    } else {
        std::cerr << "[ERREUR] Moteurs de propulsion arrière introuvables !" << std::endl;
    }

    // 2. Initialisation des Servomoteurs de Direction Avant (Ackermann)
    Motor* left_steer = robot->getMotor("left_steer_servo");
    Motor* right_steer = robot->getMotor("right_steer_servo");
    if (left_steer && right_steer) {
        left_steer->setPosition(0.0);
        right_steer->setPosition(0.0);
    } else {
        std::cerr << "[ERREUR] Servos de direction avant introuvables !" << std::endl;
    }

    // 3. Initialisation des Capteurs
    DistanceSensor* ultrasonic = robot->getDistanceSensor("ultrasonic");
    if (ultrasonic) ultrasonic->enable(timeStep);

    const char* ir_names[5] = {"ir_g2", "ir_g1", "ir_c", "ir_d1", "ir_d2"};
    DistanceSensor* ir_sensors[5];
    for (int i = 0; i < 5; ++i) {
        ir_sensors[i] = robot->getDistanceSensor(ir_names[i]);
        if (ir_sensors[i]) ir_sensors[i]->enable(timeStep);
    }

    // 4. Détecteur de Bille 3D (Surveillance chute S6)
    Node* ball_node = robot->getFromDef("BALL");
    bool ball_dropped_event = false;

    // 5. Modules Algorithmiques C++
    float kp = Config::OPTIMIZED_KP;
    float kd = Config::OPTIMIZED_KD;
    float cruise_speed = Config::OPTIMIZED_CRUISE_SPEED;
    float min_curve_speed = Config::OPTIMIZED_MIN_CURVE_SPEED;
    bool is_optuna_run = (std::getenv("OPTUNA_RUN") != nullptr);

    if (const char* env_kp = std::getenv("OPTUNA_KP")) kp = std::stof(env_kp);
    if (const char* env_kd = std::getenv("OPTUNA_KD")) kd = std::stof(env_kd);
    if (const char* env_v = std::getenv("OPTUNA_CRUISE")) cruise_speed = std::stof(env_v);
    if (const char* env_min = std::getenv("OPTUNA_MIN_SPEED")) min_curve_speed = std::stof(env_min);

    std::cout << " Paramètres effectifs :" << std::endl;
    std::cout << "  • Kp = " << kp << " | Kd = " << kd << std::endl;
    std::cout << "  • Vitesse croisière = " << cruise_speed << " m/s | Min virage = " << min_curve_speed << " m/s" << std::endl;
    if (is_optuna_run) std::cout << "  >>> MODE OPTUNA HEADLESS ACTIF <<<" << std::endl;
    std::cout << "==================================================" << std::endl;

    UltrasonicPipeline us_pipeline(5, 0.25f);
    us_pipeline.reset(100.0f);

    LineFollowerController line_controller(kp, kd, cruise_speed, min_curve_speed);
    MotionProfiler motion_profiler(0.735f, cruise_speed);

    // 6. Synchronisation SHM POSIX (/dev/shm)
    SharedMemoryManager shm_mgr;
    RobotTelemetry* telemetry = nullptr;
    try {
        telemetry = shm_mgr.open_shm(true);
    } catch (...) {}

    const float WHEEL_RADIUS = 0.032f;  // Rayon de roue 32 mm
    const float WHEELBASE = 0.130f;     // Empattement 130 mm
    uint64_t step_count = 0;
    bool mission_success = false;

    int current_sector = 0;
    float accumulated_sq_error = 0.0f;
    uint64_t line_samples = 0;

    // 7. Boucle Principale de Contrôle Webots
    while (robot->step(timeStep) != -1) {
        step_count++;

        // A. Surveillance physique de la bille libre
        if (ball_node && !ball_dropped_event) {
            const double* ball_pos = ball_node->getPosition();
            // Si la bille tombe au sol (Z < 0.03 m)
            if (ball_pos[2] < 0.025) {
                ball_dropped_event = true;
                std::cout << "\n🚨 [DÉVERSEMENT] LA BILLE EST TOMBÉE DE LA COUPELLE ! (a > 0.735 m/s²)\n" << std::endl;
            }
        }

        // B. Lecture et barycentre analogique continu des 5 capteurs optiques
        double raw_ir[5] = {0.0};
        const float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
        const float WHITE_LEVEL = 599.0f; // Réflectance du sol blanc mat
        float absorption[5] = {0.0f};
        float sum_abs = 0.0f;
        float weighted_sum = 0.0f;
        uint8_t line_sensors[5] = {0, 0, 0, 0, 0};

        for (int i = 0; i < 5; ++i) {
            if (ir_sensors[i]) {
                raw_ir[i] = ir_sensors[i]->getValue();
                // Absorption continue : 0 sur blanc, ~600 sur ruban noir
                absorption[i] = std::max(0.0f, WHITE_LEVEL - static_cast<float>(raw_ir[i]));
                sum_abs += absorption[i];
                weighted_sum += weights[i] * absorption[i];
                line_sensors[i] = (absorption[i] > 200.0f) ? 1 : 0;
            }
        }

        // Ligne détectée si l'absorption totale dépasse le seuil de bruit
        bool has_line = (sum_abs > 150.0f);
        float continuous_error = 0.0f;
        if (has_line) {
            continuous_error = weighted_sum / sum_abs;
        }

        // Ligne d'arrêt en T terminale
        int active_count = 0;
        for (int i = 0; i < 5; ++i) if (line_sensors[i]) active_count++;
        bool is_t_stop = (active_count >= 4);

        // C. Lecture et filtrage de l'ultrason
        float raw_dist = 400.0f;
        if (ultrasonic) {
            raw_dist = static_cast<float>(ultrasonic->getValue());
        }
        float filtered_dist = us_pipeline.process(raw_dist, dt, motion_profiler.get_current_speed() * 100.0f);

        // D. Asservissement suivi de ligne (PD continu analogique haute fidélité)
        LineFollowerOutput line_out = line_controller.update_continuous(continuous_error, has_line, is_t_stop, dt);

        // E. Gestion d'arrêt précis à 20 cm devant l'obstacle (S1 : +/- 30 mm)
        const float TARGET_DISTANCE_CM = 20.0f;
        float desired_velocity = line_out.target_velocity;

        if (filtered_dist <= TARGET_DISTANCE_CM) {
            desired_velocity = 0.0f; // Distance cible atteinte
            if (!mission_success && step_count > 100) {
                mission_success = true;
                std::cout << "\n🏆 [ARRÊT RÉUSSI] Arrêt d'urgence validé à " 
                          << filtered_dist << " cm de l'obstacle ! "
                          << (ball_dropped_event ? "❌ (Mais bille perdue)" : "✅ (Bille 100% intacte dans la coupelle !)") 
                          << "\n" << std::endl;
            }
        } else if (filtered_dist < (TARGET_DISTANCE_CM + 25.0f)) {
            // Rampe de décélération progressive pour respecter a <= 0.735 m/s²
            float dist_m = (filtered_dist - TARGET_DISTANCE_CM) / 100.0f;
            desired_velocity = std::min(desired_velocity, motion_profiler.calculate_max_safe_approach_speed(dist_m));
        }

        // Si fin de parcours (T-Stop)
        if (line_out.is_finished && step_count > 100) {
            desired_velocity = 0.0f;
        }

        // F. Régulation cinématique Motion Profiler (Anti-déversement bille)
        float bounded_velocity = motion_profiler.update(desired_velocity, dt);

        // G. Cinématique d'Ackermann 100% Pure et Différentiel Électronique Actif
        // Angle de consigne central en radians [-45°, +45°] (borné à +/-0.780 rad)
        float steer_angle_rad = std::clamp(-line_out.steer_normalized * 0.785398f, -0.780f, 0.780f);
        
        const float TRACK_WIDTH = 0.136f; // Voie entre roues (m)
        float left_steer_angle = steer_angle_rad;
        float right_steer_angle = steer_angle_rad;

        if (std::abs(steer_angle_rad) > 0.01f) {
            float R_center = WHEELBASE / std::tan(std::abs(steer_angle_rad));
            if (steer_angle_rad < 0) {
                // Virage à droite : roue droite = intérieure (angle plus grand), gauche = extérieure
                right_steer_angle = -std::atan(WHEELBASE / std::max(0.04f, R_center - TRACK_WIDTH * 0.5f));
                left_steer_angle = -std::atan(WHEELBASE / (R_center + TRACK_WIDTH * 0.5f));
            } else {
                // Virage à gauche : roue gauche = intérieure (angle plus grand), droite = extérieure
                left_steer_angle = std::atan(WHEELBASE / std::max(0.04f, R_center - TRACK_WIDTH * 0.5f));
                right_steer_angle = std::atan(WHEELBASE / (R_center + TRACK_WIDTH * 0.5f));
            }
        }

        if (left_steer && right_steer) {
            left_steer->setPosition(std::clamp(left_steer_angle, -0.780f, 0.780f));
            right_steer->setPosition(std::clamp(right_steer_angle, -0.780f, 0.780f));
        }

        // Différentiel électronique actif (couple de lacet éliminant le sous-virage)
        float diff_ratio = (TRACK_WIDTH / (2.0f * WHEELBASE)) * std::tan(steer_angle_rad);
        float v_left = bounded_velocity * (1.0f - diff_ratio);
        float v_right = bounded_velocity * (1.0f + diff_ratio);

        if (rear_left_motor && rear_right_motor) {
            rear_left_motor->setVelocity(-v_left / WHEEL_RADIUS);
            rear_right_motor->setVelocity(-v_right / WHEEL_RADIUS);
        }

        // H. Synchronisation SHM POSIX
        if (telemetry) {
            pthread_mutex_lock(&(telemetry->mtx));
            telemetry->filtered_dist = filtered_dist;
            telemetry->speed = bounded_velocity;
            telemetry->target_steering = line_out.steer_degrees;
            for (int i = 0; i < 5; ++i) telemetry->line_sensors[i] = line_sensors[i];
            telemetry->emergency_stop = ball_dropped_event;
            telemetry->heartbeat++;
            pthread_mutex_unlock(&(telemetry->mtx));
        }

        // I. Suivi de circuit et métriques pour Optuna
        const double* r_pos = robot->getSelf()->getPosition();
        if (current_sector == 0 && r_pos[1] > 0.40) {
            current_sector = 1;
            std::cout << ">>> SECTEUR 1 ATTEINT (Virage Nord)" << std::endl;
        }
        if (current_sector == 1 && r_pos[0] > 0.20) {
            current_sector = 2;
            std::cout << ">>> SECTEUR 2 ATTEINT (Ligne Est)" << std::endl;
        }
        if (current_sector == 2 && r_pos[1] < -0.20) {
            current_sector = 3;
            std::cout << ">>> SECTEUR 3 ATTEINT (Virage Sud)" << std::endl;
        }
        if (current_sector == 3 && r_pos[0] < -0.30 && r_pos[1] < -0.40) {
            current_sector = 4;
            std::cout << ">>> SECTEUR 4 ATTEINT (Tour complet)" << std::endl;
        }

        if (has_line) {
            accumulated_sq_error += continuous_error * continuous_error;
            line_samples++;
        }

        // J. Gestion de terminaison pour Optuna Headless
        if (is_optuna_run) {
            float cur_time = static_cast<float>(step_count) * dt;
            float mse = line_samples > 0 ? (accumulated_sq_error / static_cast<float>(line_samples)) : 9.0f;

            if (ball_dropped_event) {
                std::ofstream res("/tmp/optuna_result.txt");
                res << "success 0 lap_time " << cur_time << " mse " << mse << " ball_ok 0 sectors " << current_sector << std::endl;
                res.close();
                robot->simulationQuit(1);
                break;
            }

            if (line_out.is_line_lost) {
                std::ofstream res("/tmp/optuna_result.txt");
                res << "success 0 lap_time " << cur_time << " mse " << mse << " ball_ok 1 sectors " << current_sector << std::endl;
                res.close();
                robot->simulationQuit(2);
                break;
            }

            if (current_sector == 4) {
                std::ofstream res("/tmp/optuna_result.txt");
                res << "success 1 lap_time " << cur_time << " mse " << mse << " ball_ok 1 sectors 4" << std::endl;
                res.close();
                std::cout << "\n🏆🏆 [TOUR COMPLET RÉUSSI !] Temps: " << cur_time << "s | MSE: " << mse << std::endl;
                robot->simulationQuit(0);
                break;
            }

            if (step_count > 4000) { // Timeout 40s
                std::ofstream res("/tmp/optuna_result.txt");
                res << "success 0 lap_time 40.0 mse " << mse << " ball_ok 1 sectors " << current_sector << std::endl;
                res.close();
                robot->simulationQuit(3);
                break;
            }
        }

        // Log périodique
        if ((step_count % (1000 / timeStep) == 0) || (r_pos[1] > 0.35 && step_count % 10 == 0)) {
            std::cout << "[WEBOTS] Step: " << step_count
                      << " | Pos: (" << r_pos[0] << ", " << r_pos[1] << ")"
                      << " | V: " << bounded_velocity
                      << " | Err: " << continuous_error
                      << " | Steer: " << (line_out.steer_degrees) << "°"
                      << " | Ligne: [" << (int)line_sensors[0] << "," << (int)line_sensors[1] 
                      << "," << (int)line_sensors[2] << "," << (int)line_sensors[3] 
                      << "," << (int)line_sensors[4] << "]"
                      << std::endl;
        }
    }

    return 0;
}
