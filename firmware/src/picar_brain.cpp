#include <iostream>
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>
#include <string>
#include <algorithm>

#include "../include/ipc/shared_memory.hpp"
#include "../include/control/motion_profiler.hpp"
#include "../include/control/line_follower.hpp"

// Gestionnaire d'arrêt propre
static std::atomic<bool> g_running{true};

void sig_handler(int signum) {
    (void)signum;
    g_running = false;
}

int main(int argc, char* argv[]) {
    // 1. Fréquence paramétrable (défaut: 50 Hz, ou 100 Hz)
    int hz = 50;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--hz" && i + 1 < argc) {
            hz = std::max(1, std::atoi(argv[++i]));
        }
    }

    const auto period_ms = std::chrono::milliseconds(1000 / hz);
    const float dt = 1.0f / static_cast<float>(hz);

    std::cout << "==================================================" << std::endl;
    std::cout << " [SERVICE] picar_brain (Navigation) à " << hz << " Hz" << std::endl;
    std::cout << " Contrainte physique : a <= 0.735 m/s² (Bille FastDrink)" << std::endl;
    std::cout << "==================================================" << std::endl;

    std::signal(SIGINT, sig_handler);
    std::signal(SIGTERM, sig_handler);

    // 2. Connexion à la mémoire partagée POSIX SHM
    SharedMemoryManager shm_mgr;
    RobotTelemetry* telemetry = nullptr;
    try {
        telemetry = shm_mgr.open_shm(true);
    } catch (const std::exception& e) {
        std::cerr << "[ERREUR CRITIQUE] picar_brain : Impossible d'accéder à la SHM : " 
                  << e.what() << std::endl;
        return 1;
    }

    // 3. Instanciation des contrôleurs de navigation
    LineFollowerController line_controller(0.65f, 0.20f, 0.22f, 0.10f);
    MotionProfiler motion_profiler(0.735f, 0.25f); // a_max = 0.735 m/s², v_max = 0.25 m/s

    // Paramètres d'arrêt d'urgence obstacle (S1 : arrêt précis +/- 30 mm)
    const float TARGET_OBSTACLE_STOP_DISTANCE_CM = 20.0f; // 20 cm
    const float OBSTACLE_SLOWDOWN_THRESHOLD_CM = 45.0f;   // Commence à ralentir à 45 cm

    uint64_t loop_count = 0;

    // 4. Boucle décisionnelle cadencée
    while (g_running) {
        auto start_time = std::chrono::steady_clock::now();

        // Lecture des capteurs filtrés depuis la SHM
        float current_dist_cm = 100.0f;
        uint8_t line_sensors[5] = {0, 0, 0, 0, 0};

        pthread_mutex_lock(&(telemetry->mtx));
        current_dist_cm = telemetry->filtered_dist;
        for (int i = 0; i < 5; ++i) {
            line_sensors[i] = telemetry->line_sensors[i];
        }
        pthread_mutex_unlock(&(telemetry->mtx));

        // Étape A : Suivi de ligne optique (PD)
        LineFollowerOutput line_out = line_controller.update(line_sensors, dt);

        // Étape B : Gestion de l'obstacle par rampe cinématique
        float desired_velocity = line_out.target_velocity;

        if (line_out.is_finished) {
            // Arrivée détectée (barre transversale)
            desired_velocity = 0.0f;
        } else if (current_dist_cm <= TARGET_OBSTACLE_STOP_DISTANCE_CM) {
            // Seuil d'arrêt atteint devant obstacle
            desired_velocity = 0.0f;
        } else if (current_dist_cm < OBSTACLE_SLOWDOWN_THRESHOLD_CM) {
            // Distance résiduelle avant la cible d'arrêt
            float distance_to_target_m = (current_dist_cm - TARGET_OBSTACLE_STOP_DISTANCE_CM) / 100.0f;
            float safe_approach_speed = motion_profiler.calculate_max_safe_approach_speed(distance_to_target_m);
            desired_velocity = std::min(desired_velocity, safe_approach_speed);
        }

        // Étape C : Générateur de profil trapézoïdal borné par a_max (0.735 m/s²)
        float bounded_velocity = motion_profiler.update(desired_velocity, dt);

        // Étape D : Conversion vitesse physique (m/s) -> Consigne moteur [-100%, +100%]
        // (0.25 m/s = ~50% de PWM nominal pour les moteurs DC)
        float motor_power_percent = (bounded_velocity / 0.25f) * 50.0f;
        motor_power_percent = std::clamp(motor_power_percent, 0.0f, 100.0f);

        // Si fin de parcours ou perte prolongée
        bool req_emergency = line_out.is_line_lost;

        // Étape E : Écriture atomique des consignes vers picar_motor
        pthread_mutex_lock(&(telemetry->mtx));
        telemetry->target_speed = motor_power_percent;
        telemetry->target_steering = line_out.steer_degrees;
        telemetry->emergency_stop = req_emergency;
        if (line_out.is_finished) {
            telemetry->state = 3; // FINISHED
        }
        pthread_mutex_unlock(&(telemetry->mtx));

        // Affichage périodique (1 fois par seconde)
        loop_count++;
        if (loop_count % hz == 0) {
            std::cout << "[BRAIN] Vitesse: " << bounded_velocity 
                      << " m/s (" << motor_power_percent 
                      << "%) | Direction: " << line_out.steer_degrees 
                      << "° | Dist: " << current_dist_cm 
                      << " cm | T-Stop: " << (line_out.is_finished ? "OUI" : "NON")
                      << std::endl;
        }

        // Maintien de la période d'échantillonnage
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < period_ms) {
            std::this_thread::sleep_for(period_ms - elapsed);
        }
    }

    // Sortie propre : forcer consigne nulle
    if (telemetry != nullptr) {
        pthread_mutex_lock(&(telemetry->mtx));
        telemetry->target_speed = 0.0f;
        telemetry->target_steering = 0.0f;
        pthread_mutex_unlock(&(telemetry->mtx));
    }

    std::cout << "\n[SERVICE] Arrêt propre de picar_brain." << std::endl;
    return 0;
}
