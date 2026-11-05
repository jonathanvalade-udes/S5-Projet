#include <iostream>
#include <cassert>
#include <thread>
#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "utils/simple_json.hpp"
#include "fsm/fsm_command.hpp"
#include "sensors/sensors_telemetry.hpp"
#include "shared-services/memory/server.hpp"

void test_json_parsing() {
    std::cout << "[TEST] SimpleJson - Parsing et Sérialisation Godot..." << std::endl;

    std::string sample = "{\"distance\": 54.3, \"line_follower\": [0, 1, 0, 0, 0], \"velocity\": 0.18, \"ball_dropped\": false}\n";
    GodotTelemetryIn telem;
    bool ok = SimpleJson::parse_godot_telemetry(sample, telem);
    assert(ok);
    assert(std::abs(telem.distance - 54.3f) < 0.01f);
    assert(std::abs(telem.velocity - 0.18f) < 0.01f);
    assert(!telem.ball_dropped);
    assert(telem.line_sensors[0] == 0);
    assert(telem.line_sensors[1] == 1);
    assert(telem.line_sensors[2] == 0);
    assert(telem.line_sensors[3] == 0);
    assert(telem.line_sensors[4] == 0);

    GodotCommandOut cmd;
    cmd.velocity = 0.22f;
    cmd.direction = -0.5f;
    cmd.brake = false;
    std::string serialized = SimpleJson::serialize_command(cmd);
    assert(serialized.find("\"velocity\":0.22") != std::string::npos);
    assert(serialized.find("\"direction\":-0.5") != std::string::npos);

    // Test avec télémesure capteurs activée (mode véhicule)
    cmd.has_sensors = true;
    cmd.distance = 28.5f;
    cmd.line_sensors[2] = 1;
    std::string serialized_sensors = SimpleJson::serialize_command(cmd);
    assert(serialized_sensors.find("\"distance\":28.5") != std::string::npos);
    assert(serialized_sensors.find("\"line_follower\":[0,0,1,0,0]") != std::string::npos);

    // Trame sans "line_analog" : repli sur le binaire (0.0 / 1.0)
    assert(!telem.has_analog);
    assert(telem.line_darkness[1] == 1.0f && telem.line_darkness[2] == 0.0f);

    // Trame avec "line_analog" : valeurs continues conservées (et bornées à [0,1])
    std::string analog = "{\"distance\": 80.0, \"line_follower\": [0, 0, 1, 1, 0], "
                         "\"line_analog\": [0.0, 0.1, 0.62, 0.38, 1.7], \"velocity\": 0.1, \"ball_dropped\": false}\n";
    GodotTelemetryIn t2;
    assert(SimpleJson::parse_godot_telemetry(analog, t2));
    assert(t2.has_analog);
    assert(std::abs(t2.line_darkness[1] - 0.1f) < 1e-4f);
    assert(std::abs(t2.line_darkness[2] - 0.62f) < 1e-4f);
    assert(std::abs(t2.line_darkness[3] - 0.38f) < 1e-4f);
    assert(t2.line_darkness[4] == 1.0f); // 1.7 borné
    assert(t2.line_sensors[3] == 1);     // le champ binaire reste lu tel quel

    // Tableau analogique incomplet : ignoré (pas de valeurs à moitié lues)
    GodotTelemetryIn t3;
    assert(SimpleJson::parse_godot_telemetry("{\"line_follower\": [0,0,1,0,0], \"line_analog\": [0.2, 0.4]}\n", t3));
    assert(!t3.has_analog);
    assert(t3.line_darkness[2] == 1.0f);

    std::cout << "  ✓ Sérialisation & Dé-sérialisation JSON validées." << std::endl;
}

void test_tcp_sim_roundtrip() {
    std::cout << "[TEST] Échange TCP direct Godot <-> C++ Bridge..." << std::endl;

    const int test_port = 9876;

    // 1. Initialisation SHM
    shared::services::memory::MemoryShareComponentServer fsm_pub("FsmCommand");
    shared::services::memory::MemoryShareComponentServer sensor_pub("Sensors");
    shared::fsm::FsmCommandPayload fsm_cmd{};
    fsm_cmd.target_speed = 40.0f;     // 40%
    fsm_cmd.target_steering = 15.0f;  // 15°
    fsm_cmd.emergency_stop = 0;
    fsm_pub.publish(fsm_cmd);

    // 2. Serveur TCP Socket
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr.sin_port = htons(test_port);

    bind(server_fd, (sockaddr*)&addr, sizeof(addr));
    listen(server_fd, 1);

    // 3. Client Simulant Godot dans un thread
    std::thread client_thread([]() {
        const int port = 9876;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
        inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);

        if (connect(sock, (sockaddr*)&serv_addr, sizeof(serv_addr)) == 0) {
            // Envoi de télémétrie Godot
            std::string msg = "{\"distance\": 33.5, \"line_follower\": [0, 0, 1, 0, 0], \"velocity\": 0.2, \"ball_dropped\": false}\n";
            send(sock, msg.c_str(), msg.size(), 0);

            // Réception de la commande C++
            char buf[256];
            ssize_t n = recv(sock, buf, sizeof(buf) - 1, 0);
            if (n > 0) {
                buf[n] = '\0';
                std::string resp(buf);
                assert(resp.find("\"velocity\":") != std::string::npos);
            }
        }
        close(sock);
    });

    // 4. Acceptation côté serveur
    sockaddr_in client_addr{};
    socklen_t len = sizeof(client_addr);
    int client_fd = accept(server_fd, (sockaddr*)&client_addr, &len);
    assert(client_fd >= 0);

    char rx[256];
    ssize_t r = recv(client_fd, rx, sizeof(rx) - 1, 0);
    assert(r > 0);
    rx[r] = '\0';

    GodotTelemetryIn in;
    SimpleJson::parse_godot_telemetry(rx, in);
    assert(std::abs(in.distance - 33.5f) < 0.1f);

    // Mise à jour de la SHM
    shared::sensors::SensorsTelemetry sensors_telem{};
    sensors_telem.ultrasonicSensor.calculatedDistance = in.distance;
    sensor_pub.publish(sensors_telem);

    // Envoi de la réponse vers le client Godot (avec vitesse normalisée à 0.32 m/s)
    GodotCommandOut cmd;
    cmd.velocity = (fsm_cmd.target_speed / 50.0f) * 0.32f;
    cmd.direction = fsm_cmd.target_steering / 45.0f;
    std::string out_str = SimpleJson::serialize_command(cmd);
    send(client_fd, out_str.c_str(), out_str.size(), 0);

    client_thread.join();
    close(client_fd);
    close(server_fd);

    std::cout << "  ✓ Aller-retour TCP direct Godot <-> SHM validé avec succès." << std::endl;
}

int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "   PiCar C++17 - Tests Pont TCP Virtuel Godot     " << std::endl;
    std::cout << "==================================================" << std::endl;

    test_json_parsing();
    test_tcp_sim_roundtrip();

    std::cout << "==================================================" << std::endl;
    std::cout << "   TOUS LES TESTS DU PONT GODOT ONT RÉUSSI !      " << std::endl;
    std::cout << "==================================================" << std::endl;
    return 0;
}
