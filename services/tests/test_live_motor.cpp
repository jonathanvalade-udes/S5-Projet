#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

#include "ipc/shared_memory.hpp"

std::atomic<bool> g_running(true);

void sig_handler(int) {
    g_running = false;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    std::cout << "==================================================" << std::endl;
    std::cout << "  TEST DIRECT DU SERVICE PICAR_MOTOR (SHM POSIX)   " << std::endl;
    std::cout << "==================================================" << std::endl;

    SharedMemoryManager shm_mgr;
    RobotTelemetry* telem = shm_mgr.open_shm(true);
    if (!telem) {
        std::cerr << "[ERREUR] Impossible d'ouvrir la mémoire partagée /dev/shm/picar_shm !" << std::endl;
        return 1;
    }

    auto send_command = [&](float speed, float steering, int duration_ms) {
        std::cout << "-> Consigne: Vitesse = " << speed << "%, Angle = " << steering 
                  << "° (pendant " << duration_ms << " ms)..." << std::endl;
        
        int steps = duration_ms / 20; // 50 Hz = cycle de 20 ms
        for (int i = 0; i < steps && g_running; ++i) {
            pthread_mutex_lock(&(telem->mtx));
            telem->target_speed = speed;
            telem->target_steering = steering;
            telem->emergency_stop = false;
            telem->heartbeat++; // Nourrit le Watchdog matériel
            pthread_mutex_unlock(&(telem->mtx));

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    };

    std::cout << "[1/5] Centrage de la direction (0°)..." << std::endl;
    send_command(0.0f, 0.0f, 1000);

    std::cout << "[2/5] Braquage Gauche (-25°)..." << std::endl;
    send_command(0.0f, -25.0f, 1000);

    std::cout << "[3/5] Braquage Droite (+25°)..." << std::endl;
    send_command(0.0f, 25.0f, 1000);

    std::cout << "[4/5] Retour au centre & Propulsion AVANT (25%)..." << std::endl;
    send_command(25.0f, 0.0f, 1500);

    std::cout << "[5/5] Propulsion ARRIÈRE (25%)..." << std::endl;
    send_command(-25.0f, 0.0f, 1500);

    // Arrêt complet
    pthread_mutex_lock(&(telem->mtx));
    telem->target_speed = 0.0f;
    telem->target_steering = 0.0f;
    telem->emergency_stop = true;
    pthread_mutex_unlock(&(telem->mtx));

    std::cout << "\n>>> Test terminé : Moteurs arrêtés et Failsafe engagé. <<<" << std::endl;
    return 0;
}
