#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

#include "fsm/fsm_command.hpp"
#include "shared-services/memory/server.hpp"

std::atomic<bool> g_running(true);

void sig_handler(int) {
    g_running = false;
}

int main(int /*argc*/, char* /*argv*/[]) {
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    std::cout << "==================================================" << std::endl;
    std::cout << "  TEST DIRECT DU SERVICE PICAR_MOTOR (SHM FsmCommand) " << std::endl;
    std::cout << "==================================================" << std::endl;

    shared::services::memory::MemoryShareComponentServer fsm_pub("FsmCommand");
    if (!fsm_pub.isValid()) {
        std::cerr << "[ERREUR] Impossible d'ouvrir la mémoire partagée /dev/shm/car/FsmCommand !" << std::endl;
        return 1;
    }

    uint32_t heartbeat = 0;
    auto send_command = [&](float speed, float steering, int duration_ms) {
        std::cout << "-> Consigne: Vitesse = " << speed << "%, Angle = " << steering 
                  << "° (pendant " << duration_ms << " ms)..." << std::endl;
        
        int steps = duration_ms / 20; // 50 Hz = cycle de 20 ms
        for (int i = 0; i < steps && g_running; ++i) {
            shared::fsm::FsmCommandPayload cmd{};
            cmd.target_speed = speed;
            cmd.target_steering = steering;
            cmd.steering_offset = 0.0f;
            cmd.emergency_stop = 0;
            cmd.heartbeat = ++heartbeat;
            cmd.state = shared::fsm::MachineStates::LineFollowingForward;
            fsm_pub.publish(cmd);

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

    std::cout << "Arrêt complet des moteurs." << std::endl;
    send_command(0.0f, 0.0f, 200);

    return 0;
}
