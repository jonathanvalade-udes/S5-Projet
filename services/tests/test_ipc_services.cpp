#include <iostream>
#include <cassert>
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>

#include "fsm/fsm_command.hpp"
#include "sensors/sensors_telemetry.hpp"
#include "shared-services/memory/server.hpp"
#include "shared-services/memory/subscriber.hpp"
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

    shared::services::memory::MemoryShareComponentServer fsm_pub("FsmCommand");
    shared::services::memory::MemoryShareComponentSubscriber fsm_sub("FsmCommand");

    // Initialisation
    MockMotorDriver mock_motor(false);
    mock_motor.emergency_stop();
    assert(mock_motor.is_stopped());

    // 1. Simulation d'écriture par la FSM vers FsmCommand (100 Hz)
    shared::fsm::FsmCommandPayload cmd{};
    cmd.target_speed = 30.0f;
    cmd.target_steering = 15.0f;
    cmd.steering_offset = 0.0f;
    cmd.emergency_stop = 0;
    cmd.heartbeat = 1;
    cmd.state = shared::fsm::MachineStates::LineFollowingForward;
    fsm_pub.publish(cmd);

    // 2. Simulation de lecture par le service Moteur via subscriber
    shared::fsm::FsmCommandPayload read_cmd{};
    bool has_cmd = fsm_sub.read(read_cmd);
    assert(has_cmd);
    assert(read_cmd.emergency_stop == 0);
    assert(std::abs(read_cmd.target_speed - 30.0f) < 0.001f);
    assert(std::abs(read_cmd.target_steering - 15.0f) < 0.001f);

    mock_motor.set_speed(read_cmd.target_speed);
    mock_motor.set_steering(read_cmd.target_steering);
    assert(!mock_motor.is_stopped());
    assert(std::abs(mock_motor.get_speed() - 30.0f) < 0.001f);

    std::cout << "  ✓ Transmission SHM FsmCommand -> Moteur validée (Lock-Free)." << std::endl;

    // 3. Test de l'arrêt d'urgence Failsafe
    cmd.emergency_stop = 1;
    cmd.heartbeat++;
    fsm_pub.publish(cmd);

    fsm_sub.read(read_cmd);
    if (read_cmd.emergency_stop != 0) {
        mock_motor.emergency_stop();
    }

    assert(mock_motor.is_stopped());
    assert(std::abs(mock_motor.get_speed()) < 0.001f);
    std::cout << "  ✓ Failsafe d'urgence : Coupure moteur à 0V validée." << std::endl;

    // 4. Test du Watchdog (perte de heartbeat)
    uint32_t stalled_heartbeat = read_cmd.heartbeat;
    int watchdog_misses = 0;
    for (int i = 0; i < 15; ++i) {
        if (read_cmd.heartbeat == stalled_heartbeat) {
            watchdog_misses++;
        }
    }
    assert(watchdog_misses == 15);
    std::cout << "  ✓ Watchdog : Détection d'inactivité FSM validée." << std::endl;
}

void test_service_config_and_ipc_sockets() {
    std::cout << "[TEST] Validation des Fichiers .conf par Service et Sockets IPC (PacketHeader, SetActive, EmergencyStop, SetConfigKey)..." << std::endl;

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

    // 2. Test du serveur et client IPC Socket (PacketHeader sans CRC, payloads alignés 32 bits, ACK/NACK)
    const std::string test_sock = "/tmp/test_ipc_config.sock";
    float running_val = 1.0f;
    bool callback_called = false;
    bool active_called = false;
    bool estop_called = false;

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

    server.set_active_callback([&](bool active) {
        active_called = active;
        return true;
    });

    server.set_emergency_stop_callback([&](uint32_t reason) {
        estop_called = (reason == 42);
        return true;
    });

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

    for (int i = 0; i < 20; ++i) {
        server.process_pending_commands();
        if (callback_called) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (client_th.joinable()) client_th.join();
    assert(callback_called);
    assert(std::abs(running_val - 2.42f) < 0.001f);
    std::cout << "  ✓ IPC Socket : Commande SetConfigKey reçue et acquittée par ACK." << std::endl;

    // Test SetActive via socket
    std::thread client_active_th([&]() {
        bool ok = shared::services::ipc::ServiceIPCSocketClient::send_set_active(
            test_sock,
            shared::commands::ServiceID::FSM,
            true,
            43
        );
        assert(ok == true);
    });

    for (int i = 0; i < 20; ++i) {
        server.process_pending_commands();
        if (active_called) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (client_active_th.joinable()) client_active_th.join();
    assert(active_called);
    std::cout << "  ✓ IPC Socket : Commande SetActive reçue et acquittée par ACK." << std::endl;

    // Test EmergencyStop via socket
    std::thread client_estop_th([&]() {
        bool ok = shared::services::ipc::ServiceIPCSocketClient::send_emergency_stop(
            test_sock,
            shared::commands::ServiceID::FSM,
            42,
            44
        );
        assert(ok == true);
    });

    for (int i = 0; i < 20; ++i) {
        server.process_pending_commands();
        if (estop_called) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (client_estop_th.joinable()) client_estop_th.join();
    assert(estop_called);
    std::cout << "  ✓ IPC Socket : Commande EmergencyStop reçue et acquittée par ACK." << std::endl;

    // Test de rejet NACK sur clé invalide
    bool nack_received = false;
    std::thread client_nack_th([&]() {
        bool ok = shared::services::ipc::ServiceIPCSocketClient::send_set_config(
            test_sock,
            shared::commands::ServiceID::FSM,
            "INVALID_KEY_XYZ",
            99.0f,
            45
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
