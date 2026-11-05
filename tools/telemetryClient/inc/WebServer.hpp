#pragma once

#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>

namespace telemetry {

using CommandCallback = std::function<void(const std::string&)>;

class WebServer {
public:
    WebServer(int port = 8080, const std::string& web_root = "web");
    ~WebServer();

    // Démarrage du serveur web dans un thread d'arrière-plan
    bool start();

    // Arrêt du serveur
    void stop();

    // Enregistrement du callback déclenché lorsqu'une commande est reçue depuis l'UI
    void set_command_callback(CommandCallback cb) {
        m_command_cb = cb;
    }

    // Définit le chemin du fichier CSV pour téléchargement direct via /api/export_csv
    void set_csv_filepath(const std::string& path) {
        std::lock_guard<std::mutex> lock(m_csv_mutex);
        m_csv_filepath = path;
    }

    // Diffusion temps réel d'un paquet de télémétrie JSON vers tous les navigateurs connectés (SSE)
    void broadcast_telemetry(const std::string& json_data);

    // Diffusion d'un statut système (ex: connecté/déconnecté du robot)
    void broadcast_status(bool robot_connected, const std::string& robot_ip, int robot_port);

    int get_port() const { return m_port; }

private:
    void server_loop();
    void handle_http_request(int client_fd, const std::string& request);
    void serve_file(int client_fd, const std::string& filepath, const std::string& content_type);
    void serve_csv_download(int client_fd);
    void send_response(int client_fd, int status_code, const std::string& content_type, const std::string& body);

    int m_port{8080};
    std::string m_web_root{"web"};
    std::atomic<bool> m_running{false};
    int m_server_fd{-1};
    std::thread m_thread;

    CommandCallback m_command_cb;

    std::mutex m_csv_mutex;
    std::string m_csv_filepath{"telemetry_data.csv"};

    // Gestion des clients Server-Sent Events (SSE)
    std::mutex m_sse_mutex;
    std::vector<int> m_sse_clients;

    // Dernier état pour les nouveaux clients web
    std::mutex m_last_state_mutex;
    std::string m_last_telemetry_json{"{}"};
    std::string m_last_status_json{"{}"};
};

} // namespace telemetry
