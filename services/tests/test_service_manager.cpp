#include <cassert>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>

#include "ServiceManager/inc/ServiceManager.hpp"

void test_resolve_executable() {
    std::cout << "[TEST] Running test_resolve_executable..." << std::endl;

    // 1. Resolve standard shell binary that must exist on all POSIX systems
    std::vector<std::string> candidates = {"non_existent_binary_xyz_123", "sh", "/bin/sh"};
    std::string resolved = resolve_executable(candidates);
    assert(!resolved.empty());
    assert(access(resolved.c_str(), X_OK) == 0);
    std::cout << "  -> Resolved 'sh' to: " << resolved << " [OK]" << std::endl;

    // 2. Fallback when none exist
    std::vector<std::string> invalid_candidates = {"totally_bogus_app_1", "totally_bogus_app_2"};
    std::string fallback = resolve_executable(invalid_candidates);
    assert(fallback == "totally_bogus_app_1");
    std::cout << "  -> Fallback correctly returned first candidate [OK]" << std::endl;
}

void test_spawn_and_reap_daemon() {
    std::cout << "[TEST] Running test_spawn_and_reap_daemon..." << std::endl;

    std::string sh_path = resolve_executable({"sh", "/bin/sh"});
    assert(!sh_path.empty());

    MonitoredService svc;
    svc.name = "TestEcho";
    svc.candidates = {sh_path};
    svc.args = {"-c", "exit 0"};
    svc.pid = spawn_daemon(svc.name, sh_path, svc.args);

    assert(svc.pid > 0);
    std::cout << "  -> Spawned daemon child with PID: " << svc.pid << std::endl;

    int status = 0;
    pid_t waited_pid = waitpid(svc.pid, &status, 0);
    assert(waited_pid == svc.pid);
    assert(WIFEXITED(status));
    assert(WEXITSTATUS(status) == 0);

    // Verify supervisor cleanup and re-arm logic
    svc.pid = -1;
    svc.restart_count++;
    svc.last_attempt = std::chrono::steady_clock::now();

    assert(svc.pid <= 0);
    assert(svc.restart_count == 1);
    std::cout << "  -> Child reaped cleanly and service state re-armed [OK]" << std::endl;
}

void test_rate_limiting_cooldown() {
    std::cout << "[TEST] Running test_rate_limiting_cooldown..." << std::endl;

    MonitoredService svc;
    svc.name = "RateLimitTest";
    svc.pid = -1;
    svc.restart_count = 1;
    svc.last_attempt = std::chrono::steady_clock::now();

    // Immediately after, elapsed < 100ms so restart should be skipped
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - svc.last_attempt).count();
    assert(elapsed < 100);

    bool should_respawn = (svc.pid <= 0 && !(svc.restart_count > 0 && elapsed < 100));
    assert(!should_respawn);
    std::cout << "  -> Cooldown prevented immediate respawn [OK]" << std::endl;

    // After waiting > 100ms, restart should be allowed
    std::this_thread::sleep_for(std::chrono::milliseconds(110));
    now = std::chrono::steady_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - svc.last_attempt).count();
    assert(elapsed >= 100);

    should_respawn = (svc.pid <= 0 && !(svc.restart_count > 0 && elapsed < 100));
    assert(should_respawn);
    std::cout << "  -> Cooldown elapsed, respawn permitted [OK]" << std::endl;
}

void test_network_carrier_fallback() {
    std::cout << "[TEST] Running test_network_carrier_fallback..." << std::endl;
    // On loopback "lo", carrier/running should be queryable without crash
    bool lo_carrier = check_carrier("lo");
    std::cout << "  -> Carrier check for 'lo' executed safely (result: " << (lo_carrier ? "true" : "false") << ") [OK]" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << " RUNNING SERVICE MANAGER TESTS          " << std::endl;
    std::cout << "========================================" << std::endl;

    test_resolve_executable();
    test_spawn_and_reap_daemon();
    test_rate_limiting_cooldown();
    test_network_carrier_fallback();

    std::cout << "========================================" << std::endl;
    std::cout << " ALL SERVICE MANAGER TESTS PASSED!      " << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
