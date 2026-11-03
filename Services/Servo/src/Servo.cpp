#include "Servo.hpp"
#include <iostream>
#include <chrono>
#include <thread>

namespace services {

Servo::Servo() : m_running(false) {}

Servo::~Servo() {
    stop();
}

bool Servo::init() {
    std::cout << "[Servo] Initializing service..." << std::endl;
    m_running.store(true);
    return true;
}

void Servo::run() {
    std::cout << "[Servo] Service started and running." << std::endl;
    while (m_running.load()) {
        // Main service task processing loop
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "[Servo] Service loop terminated." << std::endl;
}

void Servo::stop() {
    if (m_running.load()) {
        std::cout << "[Servo] Stopping service..." << std::endl;
        m_running.store(false);
    }
}

} // namespace services
