#include "SimBridge.hpp"
#include <csignal>
#include <iostream>
#include <cstdlib>

static services::SimBridge* g_instance = nullptr;

static void signal_handler(int sig) {
    std::cout << "[SimBridge] Caught signal " << sig << ", stopping..." << std::endl;
    if (g_instance) {
        g_instance->stop();
    }
}

int main(int argc, char* argv[]) {
    int port = 9000;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = std::atoi(argv[++i]);
        }
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGPIPE, SIG_IGN);

    services::SimBridge bridge(port);
    g_instance = &bridge;

    if (!bridge.init()) {
        std::cerr << "[SimBridge] Initialization failed!" << std::endl;
        return 1;
    }

    bridge.run();
    return 0;
}
