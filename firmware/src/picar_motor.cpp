#include <iostream>
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>
#include <string>
#include <memory>

#include "../include/ipc/shared_memory.hpp"
#include "../include/hal/motor_driver.hpp"

// Gestionnaire d'arrêt propre et déterministe
static std::atomic<bool> g_running{true};

void sig_handler(int signum) {
    (void)signum;
    // Rompt immédiatement la boucle d'exécution motrice
    g_running = false;
}

int main(int argc, char* argv[]) {
    // 1. Fréquence paramétrable (défaut: 50 Hz, ex: 100 Hz pour réactivité accrue)
    int hz = 50;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--hz" && i + 1 < argc) {
            hz = std::max(1, std::atoi(argv[++i]));
        }
    }

    const auto period_ms = std::chrono::milliseconds(1000 / hz);

    std::cout << "==================================================" << std::endl;
    std::cout << " [SERVICE] picar_motor démarré à " << hz << " Hz" << std::endl;
    std::cout << " Période de boucle : " << period_ms.count() << " ms" << std::endl;
    std::cout << "==================================================" << std::endl;

    // 2. Sûreté Matérielle Absolue (Section 3 du Manuel d'Ingénierie)
    // Capture obligatoire de SIGINT et SIGTERM
    std::signal(SIGINT, sig_handler);
    std::signal(SIGTERM, sig_handler);

    // 3. Connexion à la mémoire partagée POSIX (/dev/shm)
    SharedMemoryManager shm_mgr;
    RobotTelemetry* telemetry = nullptr;
    try {
        // Ouvre la mémoire partagée (crée si absente)
        telemetry = shm_mgr.open_shm(true);
    } catch (const std::exception& e) {
        std::cerr << "[ERREUR CRITIQUE] Impossible d'accéder à la mémoire partagée : " 
                  << e.what() << std::endl;
        return 1;
    }

    // 4. Instanciation du pilote moteur HAL
#ifdef __linux__
    auto motor_driver = std::make_unique<PiCarHatsMotorDriver>();
#else
    auto motor_driver = std::make_unique<MockMotorDriver>(false);
#endif

    // Initialisation sécurisée : forcer les moteurs à l'arrêt complet
    motor_driver->emergency_stop();

    uint64_t loop_count = 0;
    uint64_t last_heartbeat = 0;
    int watchdog_starvation_cycles = 0;
    const int max_watchdog_misses = (hz * 250) / 1000; // Seuil = 250 ms sans heartbeat

    // 5. Boucle temps-réel cadencée (Section 8 du Manuel)
    while (g_running) {
        auto start_time = std::chrono::steady_clock::now();

        float target_speed = 0.0f;
        float target_steering = 0.0f;
        bool emergency_stop_requested = false;
        uint64_t current_heartbeat = 0;

        // Lecture atomique de la mémoire partagée
        pthread_mutex_lock(&(telemetry->mtx));
        target_speed = telemetry->target_speed;
        target_steering = telemetry->target_steering;
        emergency_stop_requested = telemetry->emergency_stop;
        current_heartbeat = telemetry->heartbeat;
        pthread_mutex_unlock(&(telemetry->mtx));

        // Surveillance Chien de Garde (Watchdog)
        if (current_heartbeat == last_heartbeat) {
            watchdog_starvation_cycles++;
        } else {
            watchdog_starvation_cycles = 0;
            last_heartbeat = current_heartbeat;
        }

        // Si timeout Watchdog dépassé (250 ms sans signe de vie des capteurs/décision)
        if (watchdog_starvation_cycles > max_watchdog_misses) {
            target_speed = 0.0f; // Coupure de sécurité automatique
        }

        // Application matérielle
        if (emergency_stop_requested) {
            motor_driver->emergency_stop();
            pthread_mutex_lock(&(telemetry->mtx));
            telemetry->speed = 0.0f;
            telemetry->state = 99; // FAILSAFE
            pthread_mutex_unlock(&(telemetry->mtx));
        } else {
            motor_driver->set_speed(target_speed);
            motor_driver->set_steering(target_steering);

            pthread_mutex_lock(&(telemetry->mtx));
            telemetry->speed = target_speed;
            telemetry->state = (std::abs(target_speed) > 0.001f) ? 2 : 1; // 2=RUNNING, 1=IDLE
            pthread_mutex_unlock(&(telemetry->mtx));
        }

        // Affichage console périodique (1 fois par seconde)
        loop_count++;
        if (loop_count % hz == 0) {
            std::cout << "[MOTOR] Vitesse appliquée: " << target_speed 
                      << "% | Direction: " << target_steering << "°"
                      << " | Watchdog misses: " << watchdog_starvation_cycles << std::endl;
        }

        // Maintien strict de la cadence temps-réel
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < period_ms) {
            std::this_thread::sleep_for(period_ms - elapsed);
        }
    }

    // =========================================================================
    // SÉCURITÉ MATÉRIELLE ABSOLUE À L'ARRÊT
    // Extinction immédiate des ponts en H et forçage GPIO / PWM à 0 V
    // =========================================================================
    std::cout << "\n[FAILSAFE] Signal reçu : extinction matérielle forcée des moteurs..." << std::endl;
    motor_driver->emergency_stop();

    // Mise à jour de l'état SHM
    if (telemetry != nullptr) {
        pthread_mutex_lock(&(telemetry->mtx));
        telemetry->speed = 0.0f;
        telemetry->target_speed = 0.0f;
        telemetry->state = 0; // STOP
        pthread_mutex_unlock(&(telemetry->mtx));
    }

    std::cout << "[SERVICE] picar_motor arrêté avec succès en état sécurisé." << std::endl;
    return 0;
}
