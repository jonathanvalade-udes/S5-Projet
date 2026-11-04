#pragma once

#include <string>

namespace services {

class FsmEventPipe {
public:
    explicit FsmEventPipe(const std::string& path = "/tmp/peripherals.sock");
    ~FsmEventPipe();

    bool check_start_trigger();
    void close_pipe();

private:
    std::string m_path;
    int m_fd{-1};
};

} // namespace services
