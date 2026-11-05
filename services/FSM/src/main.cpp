#include "FSM.hpp"
#include <csignal>
#include <iostream>
#include <cstdarg>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>

// Shared libraries headers included from libs/shared-headers
#include "header/header.hpp"
#include "services/services_telemetry.hpp"

static services::FSM* g_service_instance = nullptr;

static void log_msg(int fd, const char* format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len <= 0) return;

    ssize_t written = -1;
    if (fd >= 0) {
        written = write(fd, buffer, len);
    }
    if (written < 0) {
        int cfd = open("/dev/console", O_WRONLY | O_NOCTTY);
        if (cfd >= 0) {
            ssize_t ret = write(cfd, buffer, len);
            (void)ret;
            close(cfd);
        }
    }
}

static void signal_handler(int sig) {
    log_msg(1, "[FSM] Received signal %d, initiating shutdown...\n", sig);
    if (g_service_instance) {
        g_service_instance->stop();
    }
}

int main(int argc, char* argv[]) {
    int hz = picar::constants::DEFAULT_HZ;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--hz" && i + 1 < argc) {
            hz = std::stoi(argv[++i]);
        }
    }

    log_msg(1, "[FSM] Starting up (Hz=%d)...\n", hz);

    // Register POSIX signal handlers
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    services::FSM service(hz);
    g_service_instance = &service;

    if (!service.init()) {
        log_msg(2, "[FSM] Failed to initialize service!\n");
        return 1;
    }

    service.run();

    log_msg(1, "[FSM] Exited cleanly.\n");
    return 0;
}
