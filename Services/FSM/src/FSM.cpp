#include "FSM.hpp"
#include <iostream>
#include <chrono>
#include <thread>

namespace services {

FSM::FSM() : m_running(false) {}

FSM::~FSM() {
    stop();
}

bool FSM::init() {
    std::cout << "[FSM] Initializing service..." << std::endl;
    m_running.store(true);
    return true;
}

void FSM::run() {
    std::cout << "[FSM] Service started and running." << std::endl;
    while (m_running.load()) {
        // Main service task processing loop
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "[FSM] Service loop terminated." << std::endl;
}

void FSM::stop() {
    if (m_running.load()) {
        std::cout << "[FSM] Stopping service..." << std::endl;
        m_running.store(false);
    }
}

} // namespace services
