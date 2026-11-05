#include "ServiceManager.hpp"

#include <vector>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <chrono>
#include <csignal>

static volatile sig_atomic_t g_shutdown_requested = 0;
static void handle_exit_signal(int sig) {
    (void)sig;
    g_shutdown_requested = 1;
}

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

    setenv("PATH", "/root:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:.", 1);

    // Nettoyage préventif complet pour permettre un redémarrage instantané de /root/init
    system("killall -q -9 FSM Sensors Motors TelemetryService PeripheralsService peripherals 2>/dev/null");
    unlink("/tmp/peripherals.sock");
    unlink("/dev/shm/picar_shm");
    system("rm -f /dev/shm/* /tmp/*.sock /tmp/peripherals.sock 2>/dev/null");
    usleep(80000); // 80 ms pour libérer les ports TCP/UDP et sockets

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
        //pid_shell = spawn_shell();
    }

    // Monitored microservices with multiple candidate binary names
    std::vector<MonitoredService> monitored_services = {
        {
            "FSM",
            {"/root/FSM", "/root/fsm", "FSM", "fsm", "/usr/local/bin/FSM", "/usr/local/bin/fsm", "/usr/sbin/FSM"},
            {},
            -1,
            0,
            {},
            false
        },
        {
            "Sensors",
            {"/root/Sensors", "/root/sensors", "Sensors", "sensors", "/usr/local/bin/Sensors", "/usr/local/bin/sensors", "/usr/sbin/Sensors"},
            {},
            -1,
            0,
            {},
            false
        },
        {
            "Motors",
            {"/root/Motors", "/root/motors", "Motors", "motors", "/usr/local/bin/Motors", "/usr/local/bin/motors", "/usr/sbin/Motors"},
            {},
            -1,
            0,
            {},
            false
        },
        {
            "TelemetryService",
            {"/root/TelemetryService", "/root/telemetry", "TelemetryService", "telemetry", "/usr/local/bin/TelemetryService", "/usr/local/bin/telemetry", "/usr/sbin/TelemetryService"},
            {},
            -1,
            0,
            {},
            false
        },
        {
            "PeripheralsService",
            {"/root/PeripheralsService", "/usr/sbin/peripherals"},
            {},
            -1,
            0,
            {},
            false
        }
    };

    log_msg(1, "[INIT] MONITORING SERVICES..\n");

    signal(SIGINT, handle_exit_signal);
    signal(SIGTERM, handle_exit_signal);


    if (pid_dropbear <= 0) {
            log_msg(1, "[INIT] Launching Dropbear SSH on %s:22...\n", eth0_ip);
            pid_dropbear = spawn_dropbear();
    }

    while (!g_shutdown_requested) {
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
            } else if (!link_is_up && eth_configured) {
                log_msg(1, "[NET] Interface %s link DOWN (cable unplugged).\n", iface.c_str());
                eth_configured = false;
            }
        }

        usleep(100000); // 100 ms monitoring interval
    }

    log_msg(1, "\n[INIT] Signal d'arrêt reçu (Ctrl+C). Extinction propre des microservices...\n");
    for (auto& svc : monitored_services) {
        if (svc.pid > 0) {
            kill(svc.pid, SIGINT);
        }
    }

    // Attente que tous les enfants se terminent proprement (max 1.5s)
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
    for (auto& svc : monitored_services) {
        if (svc.pid > 0) {
            while (std::chrono::steady_clock::now() < deadline) {
                int status;
                if (waitpid(svc.pid, &status, WNOHANG) > 0) {
                    svc.pid = -1;
                    break;
                }
                usleep(20000); // 20ms
            }
            if (svc.pid > 0) {
                kill(svc.pid, SIGKILL);
                waitpid(svc.pid, nullptr, 0);
                svc.pid = -1;
            }
        }
    }

    // Nettoyage immédiat des sockets et segments SHM pour réarmement instantané
    unlink("/tmp/peripherals.sock");
    unlink("/dev/shm/picar_shm");
    system("rm -f /tmp/peripherals.sock /tmp/*.sock 2>/dev/null");

    log_msg(1, "[INIT] Tous les services et sockets sont libérés. Vous pouvez relancer /root/init immédiatement.\n");

    return 0;
}
