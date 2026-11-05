#include "Motors.hpp"
#include <csignal>
#include <iostream>
#include <cstdarg>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>

// Shared libraries headers included from libs/shared-headers
#include "header/header.hpp"
#include "services/services_telemetry.hpp"

static services::Motors* g_service_instance = nullptr;

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
    log_msg(1, "[Motors] Received signal %d, initiating shutdown...\n", sig);
    if (g_service_instance) {
        g_service_instance->stop();
    }
}

int main(int argc, char* argv[]) {
    int hz = 100;
    bool invert = false;
    float steering_offset = picar::i2c::calibration::DEFAULT_STEERING_OFFSET_DEG;
    bool offset_specified = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--hz" && i + 1 < argc) {
            hz = std::atoi(argv[++i]);
        } else if (arg == "--invert" || arg == "--reverse") {
            invert = true;
        } else if ((arg == "--offset" || arg == "--steering-offset") && i + 1 < argc) {
            steering_offset = static_cast<float>(std::atof(argv[++i]));
            offset_specified = true;
        }
    }

    if (offset_specified) {
        log_msg(1, "[Motors] Starting up (Hz=%d, Invert=%s, CLI Offset=%.2f°)...\n", 
                hz, invert ? "true" : "false", steering_offset);
    } else {
        log_msg(1, "[Motors] Starting up (Hz=%d, Invert=%s, Offset=Auto)...\n", 
                hz, invert ? "true" : "false");
    }

    // Register POSIX signal handlers
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    services::Motors service(hz, invert, steering_offset, offset_specified);
    g_service_instance = &service;

    if (!service.init()) {
        log_msg(2, "[Motors] Failed to initialize service!\n");
        return 1;
    }

    service.run();

    log_msg(1, "[Motors] Exited cleanly.\n");
    return 0;
}
