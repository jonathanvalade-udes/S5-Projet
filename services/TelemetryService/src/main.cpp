#include "TelemetryService.hpp"
#include <csignal>
#include <string>

static services::TelemetryService* g_service_instance = nullptr;

static void signal_handler(int /*sig*/) {
    if (g_service_instance) {
        g_service_instance->stop();
    }
}

int main(int argc, char* argv[]) {
    int hz = 100000;
    bool enable_ui = false;
    int tcp_port = 91;
    int udp_port = 9002;
    std::string posix_socket = "/tmp/peripherals.sock";

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
        } else if ((arg == "--posix-socket" || arg == "--sock") && i + 1 < argc) {
            posix_socket = argv[++i];
        }
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    services::TelemetryService service(hz, enable_ui, tcp_port, udp_port, posix_socket);
    g_service_instance = &service;

    if (!service.init()) {
        return 1;
    }

    service.run();
    return 0;
}
