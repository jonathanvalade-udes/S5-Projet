#include <iostream>
#include <cassert>
#include <cmath>
#include <thread>
#include <chrono>

#include "../include/ipc/shared_memory.hpp"
#include "../TelemetryService/inc/TelemetryService.hpp"

static bool approx_equal(float a, float b, float eps = 0.02f) {
    return std::abs(a - b) <= eps;
}

void test_telemetry_metrics_and_watchdog() {
    std::cout << "[TEST] TelemetryService - Agrégation, Odométrie et Watchdog..." << std::endl;

    SharedMemoryManager shm_mgr;
    RobotTelemetry* telem = shm_mgr.open_shm(true);
    assert(telem != nullptr);

    // Initialisation SHM
    pthread_mutex_lock(&(telem->mtx));
    telem->filtered_dist = 42.5f;
    telem->target_speed = 50.0f;     // 50% = 0.32 m/s
    telem->target_steering = -12.0f; // Braquage -12°
    telem->state = 3;               // LineFollowingForward
    telem->emergency_stop = false;
    telem->heartbeat = 100;
    telem->line_sensors[0] = 0;
    telem->line_sensors[1] = 1;
    telem->line_sensors[2] = 1;
    telem->line_sensors[3] = 0;
    telem->line_sensors[4] = 0;
    pthread_mutex_unlock(&(telem->mtx));

    // Instance de TelemetryService à 20 Hz, sans affichage UI console
    services::TelemetryService telem_service(20, false, 0);
    assert(telem_service.init());

    // Lance le service en arrière-plan
    std::thread th([&]() {
        telem_service.run();
    });

    // Laisse tourner pendant 250 ms (5 cycles à 20 Hz)
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    const auto& metrics = telem_service.get_metrics();
    assert(approx_equal(metrics.filtered_dist_cm, 42.5f));
    assert(approx_equal(metrics.target_speed_percent, 50.0f));
    assert(approx_equal(metrics.current_speed_ms, 0.32f));
    assert(approx_equal(metrics.target_steering_deg, -12.0f));
    assert(metrics.fsm_state == 3);
    assert(metrics.line_sensors[1] == 1 && metrics.line_sensors[2] == 1);
    std::cout << "  ✓ Agrégation des capteurs et états SHM validée." << std::endl;

    // Test Odométrie : après ~0.25s à 0.32 m/s, d ~= 0.08 m
    assert(metrics.total_distance_m > 0.05f && metrics.total_distance_m < 0.15f);
    std::cout << "  ✓ Intégration odométrique validée (d = " << metrics.total_distance_m << " m)." << std::endl;

    // Test Watchdog : heartbeat constant depuis > 500 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    const auto& metrics_stalled = telem_service.get_metrics();
    assert(metrics_stalled.watchdog_stalled == true);
    std::cout << "  ✓ Watchdog : Stagnation heartbeat correctement détectée (> 500 ms)." << std::endl;

    // Réanimation du Watchdog
    pthread_mutex_lock(&(telem->mtx));
    telem->heartbeat = 200;
    pthread_mutex_unlock(&(telem->mtx));

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    assert(telem_service.get_metrics().watchdog_stalled == false);
    std::cout << "  ✓ Watchdog : Réarmement après reprise du heartbeat validé." << std::endl;

    telem_service.stop();
    if (th.joinable()) th.join();
}

void test_telemetry_acceleration_alert() {
    std::cout << "[TEST] TelemetryService - Détection d'accélération critique (> 0.735 m/s²)..." << std::endl;

    SharedMemoryManager shm_mgr;
    RobotTelemetry* telem = shm_mgr.open_shm(true);
    assert(telem != nullptr);

    pthread_mutex_lock(&(telem->mtx));
    telem->target_speed = 0.0f;
    telem->heartbeat = 1;
    pthread_mutex_unlock(&(telem->mtx));

    services::TelemetryService telem_service(20, false, 0);
    assert(telem_service.init());

    std::thread th([&]() {
        telem_service.run();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Injection d'une accélération brutale (changement instantané de 0% à 100% en un pas dt = 50ms)
    // dv = 0.64 m/s en 0.05s => a = 12.8 m/s² >> 0.735 m/s²
    pthread_mutex_lock(&(telem->mtx));
    telem->target_speed = 100.0f;
    telem->heartbeat++;
    pthread_mutex_unlock(&(telem->mtx));

    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    const auto& metrics = telem_service.get_metrics();
    assert(metrics.accel_violations > 0);
    std::cout << "  ✓ Alerte accélération critique : Violation détectée (" 
              << metrics.accel_violations << " violations, a = " 
              << metrics.current_accel_ms2 << " m/s²)." << std::endl;

    telem_service.stop();
    if (th.joinable()) th.join();
}

int main() {
    std::cout << "===============================================" << std::endl;
    std::cout << "   PiCar C++17 - Tests Unitaires Télémétrie   " << std::endl;
    std::cout << "===============================================" << std::endl;

    test_telemetry_metrics_and_watchdog();
    test_telemetry_acceleration_alert();

    std::cout << "===============================================" << std::endl;
    std::cout << "   TOUS LES TESTS DE TÉLÉMÉTRIE ONT RÉUSSI !   " << std::endl;
    std::cout << "===============================================" << std::endl;

    return 0;
}
