#include "SimBridge.hpp"
#include <csignal>
#include <iostream>
#include <cstdlib>
#include <string>

static services::SimBridge* g_instance = nullptr;

static void signal_handler(int sig) {
    std::cout << "\n[SimBridge] Caught signal " << sig << ", stopping..." << std::endl;
    if (g_instance) {
        g_instance->stop();
    }
}

static void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  --port <port>       Port to listen on (default: 9000)\n"
              << "  --sim               Run in Simulation mode (default, Godot injects sensor telemetry)\n"
              << "  --vehicle, --real   Run in Vehicle mode on real car (streams real telemetry without overwriting sensors)\n"
              << "  -h, --help          Show this help message\n";
}

int main(int argc, char* argv[]) {
    int port = 9000;
    bool sim_mode = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = std::atoi(argv[++i]);
        } else if (arg == "--vehicle" || arg == "--real" || arg == "--telemetry-only") {
            sim_mode = false;
        } else if (arg == "--sim") {
            sim_mode = true;
        } else if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
#ifdef SIGPIPE
    std::signal(SIGPIPE, SIG_IGN);
#endif

    services::SimBridge bridge(port, sim_mode);
    g_instance = &bridge;

    if (!bridge.init()) {
        std::cerr << "[SimBridge] Initialization failed!" << std::endl;
        return 1;
    }

    bridge.run();
    return 0;
}
