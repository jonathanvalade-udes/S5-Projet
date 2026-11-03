#include "Sensors.hpp"
#include <iostream>
#include <chrono>
#include <thread>

namespace services {

Sensors::Sensors() : m_running(false) {}

Sensors::~Sensors() {
    stop();
}

bool Sensors::init() {
    std::cout << "[Sensors] Initializing service..." << std::endl;
    m_running.store(true);
    return true;
}

void Sensors::run() {
    std::cout << "[Sensors] Service started and running." << std::endl;
    while (m_running.load()) {
        // Main service task processing loop
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "[Sensors] Service loop terminated." << std::endl;
}

void Sensors::stop() {
    if (m_running.load()) {
        std::cout << "[Sensors] Stopping service..." << std::endl;
        m_running.store(false);
    }
}

} // namespace services
