#include "Motors.hpp"
#include <iostream>
#include <chrono>
#include <thread>

namespace services {

Motors::Motors() : m_running(false) {}

Motors::~Motors() {
    stop();
}

bool Motors::init() {
    std::cout << "[Motors] Initializing service..." << std::endl;
    m_running.store(true);
    return true;
}

void Motors::run() {
    std::cout << "[Motors] Service started and running." << std::endl;
    while (m_running.load()) {
        // Main service task processing loop
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "[Motors] Service loop terminated." << std::endl;
}

void Motors::stop() {
    if (m_running.load()) {
        std::cout << "[Motors] Stopping service..." << std::endl;
        m_running.store(false);
    }
}

} // namespace services
