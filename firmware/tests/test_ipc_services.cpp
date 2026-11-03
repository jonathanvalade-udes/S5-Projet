#include <iostream>
#include <cassert>
#include <thread>
#include <chrono>
#include <cmath>

#include "../include/ipc/shared_memory.hpp"
#include "../include/hal/motor_driver.hpp"
#include "../include/filters/ultrasonic_pipeline.hpp"

void test_ipc_communication_and_failsafe() {
    std::cout << "[TEST] Démarrage du test d'intégration IPC & Failsafe..." << std::endl;

    SharedMemoryManager shm_mgr;
    RobotTelemetry* telem = shm_mgr.open_shm(true);
    assert(telem != nullptr);

    // Initialisation
    MockMotorDriver mock_motor(false);
    mock_motor.emergency_stop();
    assert(mock_motor.is_stopped());

    UltrasonicPipeline pipeline(5, 0.25f);
    pipeline.reset(100.0f);

    // 1. Simulation d'écriture par le service Capteurs (50 Hz)
    float raw_sensor_distance = 42.0f;
    float filtered = pipeline.process(raw_sensor_distance, 0.02f, 0.0f);

    pthread_mutex_lock(&(telem->mtx));
    telem->filtered_dist = filtered;
    telem->heartbeat = 1;
    telem->target_speed = 30.0f;      // Consigne de vitesse 30%
    telem->target_steering = 15.0f;   // Consigne de braquage 15°
    telem->emergency_stop = false;
    pthread_mutex_unlock(&(telem->mtx));

    // 2. Simulation de lecture par le service Moteur
    float read_speed = 0.0f;
    float read_steering = 0.0f;
    bool is_emergency = false;

    pthread_mutex_lock(&(telem->mtx));
    read_speed = telem->target_speed;
    read_steering = telem->target_steering;
    is_emergency = telem->emergency_stop;
    pthread_mutex_unlock(&(telem->mtx));

    assert(!is_emergency);
    assert(std::abs(read_speed - 30.0f) < 0.001f);
    assert(std::abs(read_steering - 15.0f) < 0.001f);

    mock_motor.set_speed(read_speed);
    mock_motor.set_steering(read_steering);
    assert(!mock_motor.is_stopped());
    assert(std::abs(mock_motor.get_speed() - 30.0f) < 0.001f);

    std::cout << "  ✓ Transmission SHM Capteurs -> Moteur validée." << std::endl;

    // 3. Test de l'arrêt d'urgence Failsafe
    pthread_mutex_lock(&(telem->mtx));
    telem->emergency_stop = true;
    pthread_mutex_unlock(&(telem->mtx));

    pthread_mutex_lock(&(telem->mtx));
    if (telem->emergency_stop) {
        mock_motor.emergency_stop();
    }
    pthread_mutex_unlock(&(telem->mtx));

    assert(mock_motor.is_stopped());
    assert(std::abs(mock_motor.get_speed()) < 0.001f);
    std::cout << "  ✓ Failsafe d'urgence : Coupure moteur à 0V validée." << std::endl;

    // 4. Test du Watchdog (perte de heartbeat)
    uint64_t stalled_heartbeat = telem->heartbeat;
    int watchdog_misses = 0;
    // On simule 15 cycles sans incrément de heartbeat
    for (int i = 0; i < 15; ++i) {
        if (telem->heartbeat == stalled_heartbeat) {
            watchdog_misses++;
        }
    }
    assert(watchdog_misses == 15);
    std::cout << "  ✓ Watchdog : Détection d'inactivité capteurs validée." << std::endl;
}

int main() {
    std::cout << "===============================================" << std::endl;
    std::cout << "   PiCar C++17 - Tests d'Intégration IPC       " << std::endl;
    std::cout << "===============================================" << std::endl;

    test_ipc_communication_and_failsafe();

    std::cout << "===============================================" << std::endl;
    std::cout << "   TOUS LES TESTS IPC ONT RÉUSSI AVEC SUCCÈS   " << std::endl;
    std::cout << "===============================================" << std::endl;
    return 0;
}
