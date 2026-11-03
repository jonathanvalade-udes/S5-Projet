#include "ServiceManager.hpp"

#include <vector>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <sys/stat.h>
#if defined(__linux__)
#include <sys/mount.h>
#endif
#include <chrono>

#ifndef MS_NOSUID
#define MS_NOSUID 0
#endif
#ifndef MS_NODEV
#define MS_NODEV 0
#endif
#ifndef MS_NOEXEC
#define MS_NOEXEC 0
#endif

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    umask(0);
    bool is_pid_1 = (getpid() == 1);

    if (is_pid_1) {
        remount_rootfs_rw();
        setup_console();

        mount_essential_fs("devtmpfs", "/dev", "devtmpfs", MS_NOSUID);
        setup_console();

        mount_essential_fs("proc", "/proc", "proc", MS_NOSUID | MS_NODEV | MS_NOEXEC);
        mount_essential_fs("sysfs", "/sys", "sysfs", MS_NOSUID | MS_NODEV | MS_NOEXEC);

        mkdir("/dev/shm", 01777); 
        mkdir("/dev/pts", 0755);
        mount_essential_fs("tmpfs", "/dev/shm", "tmpfs", MS_NOSUID | MS_NODEV);
        mount_essential_fs("devpts", "/dev/pts", "devpts", MS_NOSUID | MS_NOEXEC);

        mkdir("/run", 0755);
        mount_essential_fs("tmpfs", "/run", "tmpfs", MS_NOSUID | MS_NODEV);
    } else {
        log_msg(1, "[INIT] Running in non-PID-1 user mode (PID: %d)\n", getpid());
    }

    setenv("PATH", "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/root:.", 1);

    log_msg(1, "[INIT] STARTING THE SYSTEM...\n");

    if (is_pid_1) {
        configure_static_ip("lo", "127.0.0.1", "255.0.0.0");
        interface_admin_up("lo");
    }

    const char* eth0_ip = "192.168.0.100";
    bool eth_configured = false;
    pid_t pid_dropbear = -1;
    pid_t pid_shell = -1;

    // Optional console shell for direct UART/console access
    if (is_pid_1) {
        pid_shell = spawn_shell();
    }

    // Monitored microservices with multiple candidate binary names
    std::vector<MonitoredService> monitored_services = {
        {
            "FSM",
            {"FSM", "fsm", "/root/FSM", "/root/fsm", "/usr/local/bin/FSM", "/usr/local/bin/fsm"},
            {},
            -1,
            0,
            {},
            false
        },
        {
            "Sensors",
            {"Sensors", "sensors", "/root/Sensors", "/root/sensors", "/usr/local/bin/Sensors", "/usr/local/bin/sensors"},
            {},
            -1,
            0,
            {},
            false
        },
        {
            "Motors",
            {"Motors", "motors", "/root/Motors", "/root/motors", "/usr/local/bin/Motors", "/usr/local/bin/motors"},
            {},
            -1,
            0,
            {},
            false
        },
        {
            "TelemetryService",
            {"TelemetryService", "telemetry", "/root/TelemetryService", "/root/telemetry", "/usr/local/bin/TelemetryService", "/usr/local/bin/telemetry"},
            {},
            -1,
            0,
            {},
            false
        }
    };

    log_msg(1, "[INIT] MONITORING SERVICES..\n");

    while (true) {
        int status;
        pid_t dead_pid;
        while ((dead_pid = waitpid(-1, &status, WNOHANG)) > 0) {
            int exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
            int term_sig = WIFSIGNALED(status) ? WTERMSIG(status) : -1;

            if (dead_pid == pid_dropbear) {
                log_msg(1, "[SUPERVISOR] Dropbear SSH (PID %d) terminated with code %d (sig %d)\n",
                        dead_pid, exit_code, term_sig);
                pid_dropbear = -1;
                continue;
            }

            if (dead_pid == pid_shell) {
                log_msg(1, "[SUPERVISOR] Console shell (PID %d) terminated with code %d. Restarting shell...\n",
                        dead_pid, exit_code);
                pid_shell = -1;
                if (is_pid_1) {
                    pid_shell = spawn_shell();
                }
                continue;
            }

            bool found_service = false;
            for (auto& svc : monitored_services) {
                if (svc.pid == dead_pid) {
                    log_msg(2, "[SUPERVISOR] Service '%s' (PID %d) died! (status=%d, sig=%d). Re-arming for restart.\n",
                            svc.name.c_str(), dead_pid, exit_code, term_sig);
                    svc.pid = -1;
                    svc.last_attempt = std::chrono::steady_clock::now();
                    svc.restart_count++;
                    found_service = true;
                    break;
                }
            }

            if (!found_service) {
                log_msg(1, "[SUPERVISOR] Process %d exited with status %d\n", dead_pid, exit_code);
            }
        }

        // Spawn or restart dead services with rate limiting
        auto now = std::chrono::steady_clock::now();
        for (auto& svc : monitored_services) {
            if (svc.pid <= 0) {
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - svc.last_attempt).count();
                // 100ms minimum cooldown as specified in README.md
                if (svc.restart_count > 0 && elapsed_ms < 100) {
                    continue;
                }
                // Backoff if failing repeatedly (e.g. 5 failures within 5 seconds)
                if (svc.restart_count >= 5 && elapsed_ms < 2000) {
                    continue;
                }

                std::string exec_path = resolve_executable(svc.candidates);
                svc.last_attempt = now;
                svc.pid = spawn_daemon(svc.name, exec_path, svc.args);
            }
        }

        // Network carrier and Dropbear monitoring
        std::string iface = find_ethernet_interface();
        if (!iface.empty()) {
            interface_admin_up(iface.c_str());
            bool link_is_up = check_carrier(iface);

            if (link_is_up && !eth_configured) {
                log_msg(1, "[NET] Interface %s link UP! Configuring IP: %s\n", iface.c_str(), eth0_ip);
                configure_static_ip(iface.c_str(), eth0_ip, "255.255.255.0");
                eth_configured = true;

                if (pid_dropbear <= 0) {
                    log_msg(1, "[INIT] Launching Dropbear SSH on %s:22...\n", eth0_ip);
                    pid_dropbear = spawn_dropbear();
                }
            } else if (!link_is_up && eth_configured) {
                log_msg(1, "[NET] Interface %s link DOWN (cable unplugged).\n", iface.c_str());
                eth_configured = false;
            }
        }

        usleep(100000); // 100 ms monitoring interval
    }

    return 0;
}
