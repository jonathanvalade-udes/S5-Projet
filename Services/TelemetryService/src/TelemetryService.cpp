#include "TelemetryService.hpp"
#include <iostream>
#include <chrono>
#include <thread>

namespace services {

TelemetryService::TelemetryService() : m_running(false) {}

TelemetryService::~TelemetryService() {
    stop();
}

bool TelemetryService::init() {
    std::cout << "[TelemetryService] Initializing service..." << std::endl;
    m_running.store(true);
    return true;
}

void TelemetryService::run() {
    std::cout << "[TelemetryService] Service started and running." << std::endl;
    while (m_running.load()) {
        // Main service task processing loop
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "[TelemetryService] Service loop terminated." << std::endl;
}

void TelemetryService::stop() {
    if (m_running.load()) {
        std::cout << "[TelemetryService] Stopping service..." << std::endl;
        m_running.store(false);
    }
}

} // namespace services
