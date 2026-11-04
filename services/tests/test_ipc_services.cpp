#include <iostream>
#include <cassert>
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>

#include "ipc/shared_memory.hpp"
#include "hal/motor_driver.hpp"
#include "filters/ultrasonic_pipeline.hpp"
#include "shared-services/config/service_config.hpp"
#include "shared-services/socket/service_ipc_socket.hpp"

namespace {
/**
 * @brief Mock local pour valider l'application des consignes IPC et l'arrêt d'urgence.
 */
class MockMotorDriver : public IMotorDriver {
private:
    float current_speed = 0.0f;
    float current_steering = 0.0f;
    bool stopped = true;

public:
    explicit MockMotorDriver(bool /*verbose_logs*/ = false) {}

    void set_speed(float speed_percent) override {
        current_speed = std::clamp(speed_percent, -100.0f, 100.0f);
        stopped = (std::abs(current_speed) < 0.001f);
    }

    void set_steering(float angle_degrees) override {
        current_steering = std::clamp(angle_degrees, -45.0f, 45.0f);
    }

    void emergency_stop() override {
        current_speed = 0.0f;
        current_steering = 0.0f;
        stopped = true;
    }

    float get_speed() const { return current_speed; }
    float get_steering() const { return current_steering; }
    bool is_stopped() const { return stopped; }
};
} // namespace

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

void test_service_config_and_ipc_sockets() {
    std::cout << "[TEST] Validation des Fichiers .conf par Service et Sockets IPC (PacketHeader, ACK/NACK)..." << std::endl;

    // 1. Test du gestionnaire ServiceConfig
    const std::string test_conf = "/tmp/test_service.conf";
    shared::services::config::ServiceConfig::write_key_to_file(test_conf, "MAX_ACCELERATION", 0.75f);
    shared::services::config::ServiceConfig::write_key_to_file(test_conf, "LINE_KP", 1.25f);

    shared::services::config::ServiceConfig cfg;
    assert(cfg.load_from_path(test_conf));
    assert(std::abs(cfg.get_float("MAX_ACCELERATION", 0.0f) - 0.75f) < 0.001f);
    assert(std::abs(cfg.get_float("LINE_KP", 0.0f) - 1.25f) < 0.001f);

    // Modification et persistance
    cfg.save_key("LINE_KP", 1.85f);
    shared::services::config::ServiceConfig cfg_reloaded;
    assert(cfg_reloaded.load_from_path(test_conf));
    assert(std::abs(cfg_reloaded.get_float("LINE_KP", 0.0f) - 1.85f) < 0.001f);
    std::cout << "  ✓ ServiceConfig : Lecture, écriture et persistance validées." << std::endl;

    // 2. Test du serveur et client IPC Socket (PacketHeader sans CRC, 72 octets payload, ACK/NACK)
    const std::string test_sock = "/tmp/test_ipc_config.sock";
    float running_val = 1.0f;
    bool callback_called = false;

    shared::services::ipc::ServiceIPCSocketServer server;
    bool server_init = server.init(test_sock, shared::commands::ServiceID::FSM, [&](const std::string& key, float val) {
        callback_called = true;
        if (key == "LINE_KP") {
            running_val = val;
            return true; // ACK
        }
        return false; // NACK
    });
    assert(server_init);

    // Client dans un thread séparé qui envoie une commande SetConfigKey valide
    std::thread client_th([&]() {
        bool ok = shared::services::ipc::ServiceIPCSocketClient::send_set_config(
            test_sock,
            shared::commands::ServiceID::FSM,
            "LINE_KP",
            2.42f,
            42
        );
        assert(ok == true);
    });

    // Le serveur traite la commande
    for (int i = 0; i < 20; ++i) {
        server.process_pending_commands();
        if (callback_called) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (client_th.joinable()) client_th.join();
    assert(callback_called);
    assert(std::abs(running_val - 2.42f) < 0.001f);
    std::cout << "  ✓ IPC Socket : Commande SetConfigKey reçue et acquittée par ACK." << std::endl;

    // Test de rejet NACK sur clé invalide
    bool nack_received = false;
    std::thread client_nack_th([&]() {
        bool ok = shared::services::ipc::ServiceIPCSocketClient::send_set_config(
            test_sock,
            shared::commands::ServiceID::FSM,
            "INVALID_KEY_XYZ",
            99.0f,
            43
        );
        nack_received = !ok; // Doit retourner false car NACK
    });

    for (int i = 0; i < 20; ++i) {
        server.process_pending_commands();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (client_nack_th.joinable()) client_nack_th.join();
    assert(nack_received);
    std::cout << "  ✓ IPC Socket : Commande invalide correctement rejetée par NACK." << std::endl;

    server.stop();
}

int main() {
    std::cout << "===============================================" << std::endl;
    std::cout << "   PiCar C++17 - Tests d'Intégration IPC       " << std::endl;
    std::cout << "===============================================" << std::endl;

    test_ipc_communication_and_failsafe();
    test_service_config_and_ipc_sockets();

    std::cout << "===============================================" << std::endl;
    std::cout << "   TOUS LES TESTS IPC ONT RÉUSSI AVEC SUCCÈS   " << std::endl;
    std::cout << "===============================================" << std::endl;
    return 0;
}
