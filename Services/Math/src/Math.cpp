#include "Math.hpp"
#include <iostream>
#include <chrono>
#include <thread>

namespace services {

Math::Math() : m_running(false) {}

Math::~Math() {
    stop();
}

bool Math::init() {
    std::cout << "[Math] Initializing service..." << std::endl;
    m_running.store(true);
    return true;
}

void Math::run() {
    std::cout << "[Math] Service started and running." << std::endl;
    while (m_running.load()) {
        // Main service task processing loop
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "[Math] Service loop terminated." << std::endl;
}

void Math::stop() {
    if (m_running.load()) {
        std::cout << "[Math] Stopping service..." << std::endl;
        m_running.store(false);
    }
}

} // namespace services
