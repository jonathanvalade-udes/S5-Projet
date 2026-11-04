#pragma once

#include <vector>
#include <string>
#include <chrono>
#include <sys/types.h>

struct MonitoredService {
    std::string name;
    std::vector<std::string> candidates;
    std::vector<std::string> args;
    pid_t pid = -1;
    int restart_count = 0;
    std::chrono::steady_clock::time_point last_attempt{};
    bool permanently_failed = false;
};

// Logging helper to stdout/stderr, /dev/console, and /dev/kmsg
void log_msg(int fd, const char* format, ...);

// Filesystem and rootfs mount helpers
void remount_rootfs_rw();
void mount_essential_fs(const char* source, const char* target, const char* type, unsigned long flags);

// Network configuration helpers
void interface_admin_up(const char* iface);
bool check_carrier(const std::string& iface);
void configure_static_ip(const char* iface, const char* ip, const char* netmask);
std::string find_ethernet_interface();

// Process management helpers
std::string resolve_executable(const std::vector<std::string>& candidates);
pid_t spawn_daemon(const std::string& name, const std::string& exec_path, const std::vector<std::string>& args = {});
pid_t spawn_dropbear();
pid_t spawn_shell();
void setup_console();
