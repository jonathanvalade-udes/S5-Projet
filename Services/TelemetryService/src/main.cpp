#include "TelemetryService.hpp"
#include <csignal>
#include <iostream>
#include <string>

static services::TelemetryService* g_service_instance = nullptr;

static void signal_handler(int sig) {
    std::cout << "\n[TelemetryService] Signal " << sig << " received, stopping..." << std::endl;
    if (g_service_instance) {
        g_service_instance->stop();
    }
}

int main(int argc, char* argv[]) {
    int hz = 20;
    bool enable_ui = true;
    int tcp_port = 91;
    int udp_port = 9002;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--hz" && i + 1 < argc) {
            hz = std::stoi(argv[++i]);
        } else if (arg == "--no-ui") {
            enable_ui = false;
        } else if ((arg == "--port" || arg == "--tcp-port") && i + 1 < argc) {
            tcp_port = std::stoi(argv[++i]);
        } else if (arg == "--udp-port" && i + 1 < argc) {
            udp_port = std::stoi(argv[++i]);
        }
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    services::TelemetryService service(hz, enable_ui, tcp_port, udp_port);
    g_service_instance = &service;

    if (!service.init()) {
        std::cerr << "[TelemetryService] Failed to initialize!" << std::endl;
        return 1;
    }

    service.run();
    std::cout << "[TelemetryService] Exited cleanly." << std::endl;
    return 0;
}
