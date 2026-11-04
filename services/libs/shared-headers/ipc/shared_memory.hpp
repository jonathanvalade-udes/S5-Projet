#pragma once

#include <cstdint>
#include <pthread.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

#define PICAR_SHM_NAME "/picar_shm"

/**
 * @brief Structure de télémétrie et de commande partagée entre sous-systèmes.
 * Conforme à la Section 9 du PiCar Master Architecture & Engineering Manual.
 * Accès zero-copy (< 1 µs) via /dev/shm.
 */
struct RobotTelemetry {
    // Mesures capteurs filtrées
    float filtered_dist;     // Distance ultrason HC-SR04 filtrée (cm)
    float speed;             // Vitesse actuelle estimée (cm/s ou m/s)
    float line_error;        // Erreur de suivi de ligne [-1.0, 1.0]
    uint8_t line_sensors[5]; // État binaire (noirceur > 0.5) des 5 capteurs optiques TCRT5000
    float line_darkness[5];  // Noirceur analogique des 5 capteurs [0.0 = blanc, 1.0 = noir]

    // Consignes de commande (écrites par la navigation, lues par les moteurs)
    float target_speed;      // Consigne de vitesse
    float target_steering;   // Consigne d'angle de braquage des roues
    float steering_offset;   // Offset logiciel de compensation matérielle (degrés)

    // États du système et sécurité
    uint32_t state;          // État FSM (0=STOP, 1=IDLE, 2=RUNNING, 99=FAILSAFE)
    bool emergency_stop;     // Drapeau d'arrêt d'urgence immédiat
    uint64_t heartbeat;      // Watchdog temporel (incrémenté à chaque cycle 50 Hz)

    // Synchronisation inter-processus POSIX
    pthread_mutex_t mtx;

    // Configuration dynamique modifiable on-the-fly (WiFi / picar.conf)
    float cfg_max_acceleration;       // m/s²
    float cfg_cruise_speed;           // m/s
    float cfg_min_curve_speed;        // m/s
    float cfg_line_kp;                // Gain proportionnel
    float cfg_line_kd;                // Gain dérivé
    float cfg_curve_safety_factor;    // Facteur de sécurité virage [0.5, 1.0]
    float cfg_predictive_brake_gain;  // Gain de freinage prédictif [0.0, 0.3]
    uint32_t cfg_version;             // Incrémenté lors d'une mise à jour
};

/**
 * @brief Gestionnaire de mémoire partagée POSIX SHM.
 */
class SharedMemoryManager {
private:
    int shm_fd;
    RobotTelemetry* shm_ptr;
    bool is_creator;

public:
    SharedMemoryManager() : shm_fd(-1), shm_ptr(nullptr), is_creator(false) {}

    ~SharedMemoryManager() {
        close_shm();
    }

    /**
     * @brief Ouvre ou crée le segment de mémoire partagée.
     * @param create_if_missing Si true, initialise la structure et le mutex partagé.
     */
    RobotTelemetry* open_shm(bool create_if_missing = true) {
        bool newly_created = false;
        if (create_if_missing) {
            shm_fd = shm_open(PICAR_SHM_NAME, O_RDWR | O_CREAT | O_EXCL, 0666);
            if (shm_fd >= 0) {
                newly_created = true;
            } else if (errno == EEXIST) {
                shm_fd = shm_open(PICAR_SHM_NAME, O_RDWR, 0666);
            }
        } else {
            shm_fd = shm_open(PICAR_SHM_NAME, O_RDWR, 0666);
        }

        if (shm_fd < 0) {
            throw std::runtime_error("Échec de shm_open sur " PICAR_SHM_NAME);
        }

        if (newly_created) {
            if (ftruncate(shm_fd, sizeof(RobotTelemetry)) != 0) {
                // Erreur ftruncate
            }
        }

        void* addr = mmap(nullptr, sizeof(RobotTelemetry), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (addr == MAP_FAILED) {
            throw std::runtime_error("Échec de mmap sur le segment SHM");
        }

        shm_ptr = static_cast<RobotTelemetry*>(addr);

        if (newly_created) {
            is_creator = true;
            // Configuration du mutex pour partage inter-processus
            pthread_mutexattr_t attr;
            pthread_mutexattr_init(&attr);
            pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
            pthread_mutex_init(&(shm_ptr->mtx), &attr);
            pthread_mutexattr_destroy(&attr);

            // Initialisation des valeurs par défaut sécurisées (aucun capteur actif par défaut)
            shm_ptr->filtered_dist = 100.0f;
            shm_ptr->speed = 0.0f;
            shm_ptr->line_error = 0.0f;
            for (int i = 0; i < 5; ++i) {
                shm_ptr->line_sensors[i] = 0;
                shm_ptr->line_darkness[i] = 0.0f; // Aucune ligne vue au boot
            }
            shm_ptr->target_speed = 0.0f;
            shm_ptr->target_steering = 0.0f;
            shm_ptr->steering_offset = 0.0f;
            shm_ptr->state = 0;
            shm_ptr->emergency_stop = false;
            shm_ptr->heartbeat = 0;
            shm_ptr->cfg_max_acceleration = 0.735f;
            shm_ptr->cfg_cruise_speed = 0.40f;
            shm_ptr->cfg_min_curve_speed = 0.14f;
            shm_ptr->cfg_line_kp = 0.65f;
            shm_ptr->cfg_line_kd = 0.05f;
            shm_ptr->cfg_curve_safety_factor = 0.85f;
            shm_ptr->cfg_predictive_brake_gain = 0.06f;
            shm_ptr->cfg_version = 1;
        } else if (create_if_missing) {
            // Segment existant réouvert en mode superviseur : réinitialiser l'arrêt d'urgence
            pthread_mutex_lock(&(shm_ptr->mtx));
            shm_ptr->emergency_stop = false;
            pthread_mutex_unlock(&(shm_ptr->mtx));
        }

        return shm_ptr;
    }

    void close_shm() {
        if (shm_ptr != nullptr) {
            munmap(shm_ptr, sizeof(RobotTelemetry));
            shm_ptr = nullptr;
        }
        if (shm_fd >= 0) {
            close(shm_fd);
            shm_fd = -1;
        }
        if (is_creator) {
            shm_unlink(PICAR_SHM_NAME);
            is_creator = false;
        }
    }

    RobotTelemetry* get() { return shm_ptr; }
};
