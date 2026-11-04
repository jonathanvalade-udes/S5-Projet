#include "TelemetryClient.hpp"

#include <iostream>
#include <csignal>
#include <cstdlib>
#include <string>

static telemetry::TelemetryClient* g_client = nullptr;

static void signal_handler(int sig) {
    std::cout << "\n[Main] Signal " << sig << " reçu. Arrêt propre du client..." << std::endl;
    if (g_client) {
        g_client->stop();
    }
}

static void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [OPTIONS]\n"
              << "Options:\n"
              << "  --ip <host>          Adresse IP du robot / TelemetryService (défaut: 127.0.0.1 ou PICAR_HOST)\n"
              << "  --port <port>        Port TCP du TelemetryService (défaut: 91 ou PICAR_PORT)\n"
              << "  --web-port <port>    Port HTTP pour l'UI Web (défaut: 8080 ou WEB_PORT)\n"
              << "  --web-root <dir>     Chemin vers les assets web HTML/JS/CSS (défaut: web)\n"
              << "  --no-web             Désactiver l'interface web locale\n"
              << "  --help               Afficher cette aide\n";
}

int main(int argc, char* argv[]) {
    telemetry::TelemetryClientConfig config;

    // 1. Lire d'abord les variables d'environnement si présentes
    if (const char* env_host = std::getenv("PICAR_HOST")) {
        config.robot_ip = env_host;
    } else if (const char* env_ip = std::getenv("ROBOT_IP")) {
        config.robot_ip = env_ip;
    }

    if (const char* env_port = std::getenv("PICAR_PORT")) {
        config.robot_port = std::stoi(env_port);
    } else if (const char* env_rport = std::getenv("ROBOT_PORT")) {
        config.robot_port = std::stoi(env_rport);
    }

    if (const char* env_web = std::getenv("WEB_PORT")) {
        config.web_port = std::stoi(env_web);
    }

    if (const char* env_root = std::getenv("WEB_ROOT")) {
        config.web_root = env_root;
    }

    // 2. Parser les arguments CLI prioritaires
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--ip" || arg == "-i") && i + 1 < argc) {
            config.robot_ip = argv[++i];
        } else if ((arg == "--port" || arg == "-p") && i + 1 < argc) {
            config.robot_port = std::stoi(argv[++i]);
        } else if ((arg == "--web-port" || arg == "-w") && i + 1 < argc) {
            config.web_port = std::stoi(argv[++i]);
        } else if (arg == "--web-root" && i + 1 < argc) {
            config.web_root = argv[++i];
        } else if (arg == "--no-web") {
            config.enable_web = false;
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    telemetry::TelemetryClient client(config);
    g_client = &client;

    if (!client.init()) {
        std::cerr << "[Main] Échec d'initialisation du client télémétrie." << std::endl;
        return 1;
    }

    client.run();
    std::cout << "[Main] Client terminé proprement." << std::endl;
    return 0;
}
