#include <cassert>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>

#include "PeripheralsService/inc/PeripheralsService.hpp"

void test_peripherals_posix_socket_lifecycle() {
    std::cout << "[TEST] Running test_peripherals_posix_socket_lifecycle..." << std::endl;

    const std::string test_sock_path = "/tmp/test_peripherals_ipc.sock";
    ::unlink(test_sock_path.c_str());

    services::PeripheralsService service(test_sock_path);

    // 1. Initialisation : le socket file doit être créé
    bool init_res = service.init();
    assert(init_res);
    assert(service.get_server_fd() >= 0);
    assert(access(test_sock_path.c_str(), F_OK) == 0);
    std::cout << "  ✓ Socket file created successfully on filesystem at: " << test_sock_path << std::endl;

    // 2. Démarrage du thread de service
    std::atomic<bool> msg_received{false};
    std::string received_msg;
    service.set_message_callback([&](int, const std::vector<uint8_t>& data) {
        received_msg = std::string(data.begin(), data.end());
        msg_received.store(true);
    });

    std::thread service_thread([&service]() {
        service.run();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 3. Connexion d'un client IPC via POSIX socket file
    int client_fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    assert(client_fd >= 0);

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, test_sock_path.c_str(), sizeof(addr.sun_path) - 1);

    int conn_res = ::connect(client_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    assert(conn_res == 0);
    std::cout << "  ✓ IPC client successfully connected via POSIX socket." << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    assert(service.is_client_connected());
    assert(service.get_connected_clients_count() == 1);

    // 4. Envoi de données du client vers le service
    const std::string ping_payload = "PERIPHERAL_PING";
    ssize_t sent = ::send(client_fd, ping_payload.c_str(), ping_payload.size(), 0);
    assert(sent == static_cast<ssize_t>(ping_payload.size()));

    // Attendre la réception
    int retries = 20;
    while (!msg_received.load() && retries-- > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    assert(msg_received.load());
    assert(received_msg == ping_payload);
    std::cout << "  ✓ IPC data received by service: " << received_msg << std::endl;

    // 5. Broadcast du service vers le client
    const std::string broadcast_msg = "PERIPHERAL_STATUS_OK";
    bool bcast_res = service.broadcast(broadcast_msg);
    assert(bcast_res);

    char rx_buf[128];
    ssize_t n = ::recv(client_fd, rx_buf, sizeof(rx_buf) - 1, 0);
    assert(n > 0);
    rx_buf[n] = '\0';
    assert(std::string(rx_buf) == broadcast_msg);
    std::cout << "  ✓ IPC data received by client from broadcast: " << rx_buf << std::endl;

    // 6. Déconnexion du client et arrêt du service
    ::close(client_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    service.stop();
    if (service_thread.joinable()) {
        service_thread.join();
    }

    // 7. Vérifier que le socket file a bien été supprimé
    assert(access(test_sock_path.c_str(), F_OK) != 0);
    std::cout << "  ✓ Socket file cleanly removed upon shutdown." << std::endl;
}

void test_peripherals_buzzer_sounds() {
    std::cout << "[TEST] Running test_peripherals_buzzer_sounds..." << std::endl;

    services::PeripheralsService service("/tmp/test_buzzer.sock");
    service.init();

    // Verify init buzzer was executed
    assert(service.get_last_buzzer_sound() == services::BuzzerSound::Init);
    std::cout << "  ✓ Init buzzer sound verified." << std::endl;

    service.wifi_buzzer();
    assert(service.get_last_buzzer_sound() == services::BuzzerSound::Wifi);
    std::cout << "  ✓ WiFi buzzer sound verified." << std::endl;

    service.wifi_disconnect_buzzer();
    assert(service.get_last_buzzer_sound() == services::BuzzerSound::WifiDisconnect);
    std::cout << "  ✓ WiFi disconnect buzzer sound verified." << std::endl;

    service.start_buzzer();
    assert(service.get_last_buzzer_sound() == services::BuzzerSound::Start);
    std::cout << "  ✓ Start buzzer sound verified." << std::endl;

    service.stop();
}

void test_button_debounce_and_triggers() {
    std::cout << "[TEST] Running test_button_debounce_and_triggers..." << std::endl;

    services::PeripheralsService service("/tmp/test_buttons.sock");
    service.init();
    service.set_mock_gpio_enabled(true);

    std::atomic<int> wifi_trigger_count{0};
    std::atomic<int> start_trigger_count{0};

    service.set_wifi_callback([&]() {
        wifi_trigger_count++;
    });
    service.set_start_callback([&]() {
        start_trigger_count++;
    });

    // 1. Appui court (< 10 ms) sur le bouton WiFi -> aucun déclenchement
    service.set_mock_button_state(services::WIFI_BTN_PIN, true);
    service.poll_buttons();
    std::this_thread::sleep_for(std::chrono::milliseconds(5)); // seulement 5 ms (< 10 ms)
    service.poll_buttons();
    service.set_mock_button_state(services::WIFI_BTN_PIN, false);
    service.poll_buttons();

    assert(wifi_trigger_count.load() == 0);
    std::cout << "  ✓ Debounce: Short press (< 10ms) did NOT trigger WiFi." << std::endl;

    // 2. Appui valide (>= 10 ms) sur le bouton WiFi -> déclenchement unique et son WiFi connecté
    service.set_mock_button_state(services::WIFI_BTN_PIN, true);
    service.poll_buttons();
    std::this_thread::sleep_for(std::chrono::milliseconds(15)); // 15 ms (>= 10 ms)
    service.poll_buttons();

    assert(wifi_trigger_count.load() == 1);
    assert(service.get_last_buzzer_sound() == services::BuzzerSound::Wifi);
    std::cout << "  ✓ Valid press (>= 10ms) triggered WiFi and WiFi connected buzzer sound." << std::endl;

    // 3. Maintien prolongé du bouton WiFi -> pas de déclenchements multiples répétés
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    service.poll_buttons();
    assert(wifi_trigger_count.load() == 1);
    std::cout << "  ✓ Continuous hold does not repeat trigger." << std::endl;

    // Relâchement du bouton
    service.set_mock_button_state(services::WIFI_BTN_PIN, false);
    service.poll_buttons();

    // 3b. Deuxième appui valide (>= 10 ms) sur le bouton WiFi -> déconnexion et son WiFi déconnecté
    service.set_mock_button_state(services::WIFI_BTN_PIN, true);
    service.poll_buttons();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    service.poll_buttons();

    assert(wifi_trigger_count.load() == 2);
    assert(service.get_last_buzzer_sound() == services::BuzzerSound::WifiDisconnect);
    std::cout << "  ✓ Toggle WiFi OFF triggered WiFi disconnect buzzer sound." << std::endl;

    // Relâchement du bouton
    service.set_mock_button_state(services::WIFI_BTN_PIN, false);
    service.poll_buttons();

    // 4. Appui court (< 10 ms) sur le bouton Start -> aucun déclenchement
    service.set_mock_button_state(services::START_BTN_PIN, true);
    service.poll_buttons();
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
    service.poll_buttons();
    service.set_mock_button_state(services::START_BTN_PIN, false);
    service.poll_buttons();

    assert(start_trigger_count.load() == 0);
    std::cout << "  ✓ Debounce: Short press (< 10ms) did NOT trigger Start." << std::endl;

    // 5. Appui valide (>= 10 ms) sur le bouton Start -> déclenchement
    service.set_mock_button_state(services::START_BTN_PIN, true);
    service.poll_buttons();
    std::this_thread::sleep_for(std::chrono::milliseconds(15)); // >= 10 ms
    service.poll_buttons();

    assert(start_trigger_count.load() == 1);
    assert(service.get_last_buzzer_sound() == services::BuzzerSound::Start);
    std::cout << "  ✓ Valid press (>= 10ms) triggered Start and Start buzzer sound." << std::endl;

    service.set_mock_button_state(services::START_BTN_PIN, false);
    service.poll_buttons();

    service.stop();
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << " RUNNING PERIPHERALS SERVICE TESTS      " << std::endl;
    std::cout << "========================================" << std::endl;

    test_peripherals_posix_socket_lifecycle();
    test_peripherals_buzzer_sounds();
    test_button_debounce_and_triggers();

    std::cout << "========================================" << std::endl;
    std::cout << " ALL PERIPHERALS SERVICE TESTS PASSED!  " << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
