#pragma once

#include <atomic>
#include <string>

// Shared headers from libs
#include "header/header.hpp"
#include "services/services_telemetry.hpp"

namespace services {

class Motors {
public:
    Motors();
    ~Motors();

    bool init();
    void run();
    void stop();

    bool is_running() const { return m_running.load(); }

private:
    std::atomic<bool> m_running{false};
};

} // namespace services
