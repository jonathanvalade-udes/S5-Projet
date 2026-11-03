#include "dumpBin.hpp"
#include <iostream>
#include <chrono>
#include <thread>

namespace services {

dumpBin::dumpBin() : m_running(false) {}

dumpBin::~dumpBin() {
    stop();
}

bool dumpBin::init() {
    std::cout << "[dumpBin] Initializing service..." << std::endl;
    m_running.store(true);
    return true;
}

void dumpBin::run() {
    std::cout << "[dumpBin] Service started and running." << std::endl;
    while (m_running.load()) {
        // Main service task processing loop
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "[dumpBin] Service loop terminated." << std::endl;
}

void dumpBin::stop() {
    if (m_running.load()) {
        std::cout << "[dumpBin] Stopping service..." << std::endl;
        m_running.store(false);
    }
}

} // namespace services
