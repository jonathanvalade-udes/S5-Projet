#include <iostream>
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>
#include <string>
#include <memory>

#include "../include/ipc/shared_memory.hpp"
#include "../include/filters/ultrasonic_pipeline.hpp"
#include "../include/hal/sensor_driver.hpp"

// Gestionnaire d'arrêt propre
static std::atomic<bool> g_running{true};

void sig_handler(int signum) {
    (void)signum;
    g_running = false;
}

// Calcul de l'erreur barycentrique de suivi de ligne [-1.0, +1.0]
float compute_line_error(const uint8_t sensors[5]) {
    const float weights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
    float weighted_sum = 0.0f;
    int active_count = 0;

    for (int i = 0; i < 5; ++i) {
        if (sensors[i]) {
            weighted_sum += weights[i];
            active_count++;
        }
    }

    if (active_count == 0) {
        return 0.0f; // Aucune ligne détectée, maintient l'angle neutre
    }
    // Normalisation sur [-1.0, 1.0] (division par max weight = 2.0 * active_count)
    return (weighted_sum / (2.0f * active_count));
}

int main(int argc, char* argv[]) {
    // 1. Fréquence paramétrable (défaut: 50 Hz)
    int hz = 50;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--hz" && i + 1 < argc) {
            hz = std::max(1, std::atoi(argv[++i]));
        }
    }

    const auto period_ms = std::chrono::milliseconds(1000 / hz);

    std::cout << "==================================================" << std::endl;
    std::cout << " [SERVICE] picar_sensors démarré à " << hz << " Hz" << std::endl;
    std::cout << " Période de boucle : " << period_ms.count() << " ms" << std::endl;
    std::cout << "==================================================" << std::endl;

    // 2. Capture des signaux POSIX pour un arrêt déterministe
    std::signal(SIGINT, sig_handler);
    std::signal(SIGTERM, sig_handler);

    // 3. Initialisation de la mémoire partagée POSIX SHM
    SharedMemoryManager shm_mgr;
    RobotTelemetry* telemetry = nullptr;
    try {
        telemetry = shm_mgr.open_shm(true);
    } catch (const std::exception& e) {
        std::cerr << "[ERREUR CRITIQUE] Impossible d'ouvrir la mémoire partagée : " 
                  << e.what() << std::endl;
        return 1;
    }

    // 4. Instanciation du pilote de capteur (HAL) et du pipeline de filtrage
#ifdef __linux__
    auto sensor_driver = std::make_unique<PiCarHatsSensorDriver>();
#else
    auto sensor_driver = std::make_unique<MockSensorDriver>();
#endif

    UltrasonicPipeline us_pipeline(5, 0.25f);
    us_pipeline.reset(100.0f);

    uint64_t loop_count = 0;
    float dt = 1.0f / static_cast<float>(hz);

    // 5. Boucle temps-réel d'acquisition et de filtrage
    while (g_running) {
        auto start_time = std::chrono::steady_clock::now();

        // Lecture brute des capteurs
        float raw_dist = sensor_driver->read_raw_ultrasonic_cm();
        uint8_t line_sensors[5];
        sensor_driver->read_line_sensors(line_sensors);

        // Lecture de la vitesse actuelle pour la prédiction Kalman
        float current_speed = 0.0f;
        pthread_mutex_lock(&(telemetry->mtx));
        current_speed = telemetry->speed;
        pthread_mutex_unlock(&(telemetry->mtx));

        // Filtrage numérique temps-réel
        float filtered_dist = us_pipeline.process(raw_dist, dt, current_speed);
        float line_error = compute_line_error(line_sensors);

        // Écriture atomique dans la mémoire partagée (/dev/shm)
        pthread_mutex_lock(&(telemetry->mtx));
        telemetry->filtered_dist = filtered_dist;
        telemetry->line_error = line_error;
        for (int i = 0; i < 5; ++i) {
            telemetry->line_sensors[i] = line_sensors[i];
        }
        telemetry->heartbeat++; // Progression du Watchdog
        pthread_mutex_unlock(&(telemetry->mtx));

        // Affichage console périodique (1 fois par seconde)
        loop_count++;
        if (loop_count % hz == 0) {
            std::cout << "[SENSORS] Distance filtrée: " << filtered_dist 
                      << " cm (brute: " << raw_dist 
                      << " cm) | Erreur ligne: " << line_error 
                      << " | Heartbeat: " << telemetry->heartbeat << std::endl;
        }

        // Maintien strict de la cadence
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed < period_ms) {
            std::this_thread::sleep_for(period_ms - elapsed);
        }
    }

    std::cout << "\n[SERVICE] Arrêt propre de picar_sensors." << std::endl;
    return 0;
}
