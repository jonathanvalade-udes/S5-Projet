#include "ServiceManager.hpp"

#include <vector>
#include <string>
#include <sys/mount.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <sys/stat.h>
#include <sys/ioctl.h>
#if defined(__linux__)
#include <sys/sysmacros.h>
#else
#include <sys/types.h>
#ifndef MS_NOSUID
#define MS_NOSUID 0
#endif
#ifndef MS_NODEV
#define MS_NODEV 0
#endif
#ifndef MS_NOEXEC
#define MS_NOEXEC 0
#endif
#ifndef MS_REMOUNT
#define MS_REMOUNT 0
#endif
#endif
#include <termios.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <dirent.h>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>

static int g_console_fd = -1;
static int g_kmsg_fd = -1;

void log_msg(int fd, const char* format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len <= 0) return;

    ssize_t written = -1;
    if (fd >= 0) {
        written = write(fd, buffer, len);
    }

    if (written < 0) {
        int cfd = open("/dev/console", O_WRONLY | O_NOCTTY);
        if (cfd >= 0) {
            ssize_t ret = write(cfd, buffer, len);
            (void)ret;
            close(cfd);
        }
    }

    if (g_kmsg_fd < 0) {
        g_kmsg_fd = open("/dev/kmsg", O_WRONLY | O_NOCTTY);
    }
    if (g_kmsg_fd >= 0) {
        char kmsg_buf[1050];
        const char* level = (fd == 2) ? "<3>" : "<6>";
        int klen = snprintf(kmsg_buf, sizeof(kmsg_buf), "%s%s", level, buffer);
        if (klen > 0) {
            ssize_t ret = write(g_kmsg_fd, kmsg_buf, klen);
            (void)ret;
        }
    }
}

void remount_rootfs_rw() {
#if defined(__linux__)
    if (mount(nullptr, "/", nullptr, MS_REMOUNT, nullptr) == -1) {
        log_msg(2, "[INIT] Failed to remount rootfs as read-write: %s\n", std::strerror(errno));
    } else {
        log_msg(1, "[INIT] Successfully remounted rootfs as read-write.\n");
    }
#else
    log_msg(1, "[INIT] (Host non-Linux mock) Skip remounting rootfs as read-write.\n");
#endif
}

void mount_essential_fs(const char* source, const char* target, const char* type, unsigned long flags) {
#if defined(__linux__)
    if (mount(source, target, type, flags, nullptr) == 0) {
        log_msg(1, "[INIT] Mounted %s onto %s\n", type, target);
    } else {
        if (errno == EBUSY) {
            log_msg(1, "[INIT] Target %s already mounted.\n", target);
        } else {
            log_msg(2, "[INIT] Failed to mount %s: %s\n", target, std::strerror(errno));
        }
    }
#else
    (void)source; (void)target; (void)type; (void)flags;
    log_msg(1, "[INIT] (Host non-Linux mock) Skip mounting %s onto %s\n", type, target);
#endif
}

void interface_admin_up(const char* iface) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return;

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, iface, IFNAMSIZ);

    if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0) {
        if (!(ifr.ifr_flags & IFF_UP)) {
            ifr.ifr_flags |= (IFF_UP | IFF_RUNNING);
            if (ioctl(sock, SIOCSIFFLAGS, &ifr) == 0) {
                log_msg(1, "[NET] Interface %s administratively UP.\n", iface);
            }
        }
    }
    close(sock);
}

bool check_carrier(const std::string& iface) {
    std::string carrier_path = "/sys/class/net/" + iface + "/carrier";
    int fd = open(carrier_path.c_str(), O_RDONLY);
    if (fd >= 0) {
        char carrier_state = '0';
        ssize_t n = read(fd, &carrier_state, 1);
        close(fd);
        if (n > 0) {
            return (carrier_state == '1');
        }
        return false;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock >= 0) {
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ);
        if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0) {
            close(sock);
            return (ifr.ifr_flags & IFF_RUNNING) != 0;
        }
        close(sock);
    }
    return false;
}

void configure_static_ip(const char* iface, const char* ip, const char* netmask) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return;

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, iface, IFNAMSIZ);
    struct sockaddr_in* addr = (struct sockaddr_in*)&ifr.ifr_addr;
    addr->sin_family = AF_INET;

    inet_pton(AF_INET, ip, &addr->sin_addr);
    ioctl(sock, SIOCSIFADDR, &ifr);

    inet_pton(AF_INET, netmask, &addr->sin_addr);
    ioctl(sock, SIOCSIFNETMASK, &ifr);

    log_msg(1, "[NET] Configured %s with IP: %s\n", iface, ip);
    close(sock);
}

std::string find_ethernet_interface() {
    if (access("/sys/class/net/eth0", F_OK) == 0) {
        return "eth0";
    }
    DIR* dir = opendir("/sys/class/net");
    if (!dir) return "";
    struct dirent* entry;
    std::string found = "";
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_name[0] == '.') continue;
        if (std::strcmp(entry->d_name, "lo") == 0) continue;
        found = entry->d_name;
        break;
    }
    closedir(dir);
    return found;
}

void setup_console() {
    mkdir("/dev", 0755);

#if defined(__linux__)
    struct stat st;
    if (stat("/dev/console", &st) != 0) {
        mknod("/dev/console", S_IFCHR | 0600, makedev(5, 1));
    }
    if (stat("/dev/ttyAMA0", &st) != 0) {
        mknod("/dev/ttyAMA0", S_IFCHR | 0660, makedev(204, 64));
    }
    if (stat("/dev/null", &st) != 0) {
        mknod("/dev/null", S_IFCHR | 0666, makedev(1, 3));
    }
#endif

    const char* console_candidates[] = {
        "/dev/ttyAMA0",
        "/dev/console",
        "/dev/ttyS0",
        "/dev/tty0"
    };

    int fd = -1;
    for (const char* dev : console_candidates) {
        fd = open(dev, O_RDWR | O_NOCTTY);
        if (fd < 0) {
            fd = open(dev, O_WRONLY | O_NOCTTY);
        }
        if (fd >= 0) break;
    }

    if (fd >= 0) {
        struct termios tty;
        if (tcgetattr(fd, &tty) == 0) {
            tty.c_cflag |= (CREAD | CLOCAL);
            tty.c_oflag |= (OPOST | ONLCR);
            tty.c_lflag |= (ISIG | ICANON | ECHO | ECHOE | ECHOK);
            tcsetattr(fd, TCSANOW, &tty);
        }

        setsid();
        ioctl(fd, TIOCSCTTY, 1);

        dup2(fd, STDIN_FILENO);
        dup2(fd, STDOUT_FILENO);
        dup2(fd, STDERR_FILENO);

        if (fd > STDERR_FILENO) {
            close(fd);
        }
        g_console_fd = STDOUT_FILENO;
    }
}

std::string resolve_executable(const std::vector<std::string>& candidates) {
    const char* env_path = std::getenv("PATH");
    std::vector<std::string> path_dirs;
    if (env_path) {
        std::string p(env_path);
        size_t start = 0;
        while (start < p.size()) {
            size_t end = p.find(':', start);
            if (end == std::string::npos) end = p.size();
            path_dirs.push_back(p.substr(start, end - start));
            start = end + 1;
        }
    }
    path_dirs.push_back("/usr/local/bin");
    path_dirs.push_back("/usr/local/sbin");
    path_dirs.push_back("/usr/bin");
    path_dirs.push_back("/usr/sbin");
    path_dirs.push_back("/bin");
    path_dirs.push_back("/sbin");
    path_dirs.push_back("/root");
    path_dirs.push_back(".");

    for (const auto& cand : candidates) {
        if (cand.find('/') != std::string::npos) {
            if (access(cand.c_str(), X_OK) == 0) {
                return cand;
            }
        } else {
            for (const auto& dir : path_dirs) {
                std::string full_path = dir.empty() ? cand : (dir + "/" + cand);
                if (access(full_path.c_str(), X_OK) == 0) {
                    return full_path;
                }
            }
        }
    }

    return candidates.empty() ? "" : candidates.front();
}

pid_t spawn_daemon(const std::string& name, const std::string& exec_path, const std::vector<std::string>& args) {
    pid_t pid = fork();
    if (pid < 0) {
        log_msg(2, "[DAEMON] Failed to fork %s: %s\n", name.c_str(), std::strerror(errno));
        return -1;
    }
    if (pid == 0) {
        std::vector<char*> argv;
        argv.push_back(const_cast<char*>(exec_path.c_str()));
        for (const auto& arg : args) {
            argv.push_back(const_cast<char*>(arg.c_str()));
        }
        argv.push_back(nullptr);

        execvp(exec_path.c_str(), argv.data());
        log_msg(2, "[DAEMON] Failed to exec %s (%s): %s\n", name.c_str(), exec_path.c_str(), std::strerror(errno));
        _exit(127);
    }
    log_msg(1, "[DAEMON] Started %s (PID: %d) via %s\n", name.c_str(), pid, exec_path.c_str());
    return pid;
}

pid_t spawn_dropbear() {
    mkdir("/etc/dropbear", 0755);
    mkdir("/var/log", 0755);
    mkdir("/var/run", 0755);
    mkdir("/root", 0700);

    if (access("/etc/key", F_OK) != 0) {
        log_msg(1, "[INIT] Generating Dropbear host key (/etc/key)...\n");
        system("dropbearkey -t ed25519 -f /etc/key >/dev/null 2>&1");
    }

    std::vector<std::string> args = {
        "-F",
        "-E",
        "-r",
        "/etc/key",
        "-B"
    };
    return spawn_daemon("dropbear", "dropbear", args);
}

pid_t spawn_shell() {
    pid_t pid = fork();
    if (pid < 0) {
        log_msg(2, "[INIT] Failed to fork console shell: %s\n", std::strerror(errno));
        return -1;
    }
    if (pid == 0) {
        const char* tty_candidates[] = {
            "/dev/ttyAMA0",
            "/dev/console",
            "/dev/ttyS0",
            "/dev/tty0"
        };

        int console_fd = -1;
        for (const char* dev : tty_candidates) {
            console_fd = open(dev, O_RDWR);
            if (console_fd >= 0) break;
        }

        if (console_fd >= 0) {
            dup2(console_fd, STDIN_FILENO);
            dup2(console_fd, STDOUT_FILENO);
            dup2(console_fd, STDERR_FILENO);
            if (console_fd > STDERR_FILENO) {
                close(console_fd);
            }
        }

        setsid();
        ioctl(STDIN_FILENO, TIOCSCTTY, 1);

        struct termios tty;
        if (tcgetattr(STDIN_FILENO, &tty) == 0) {
            tty.c_cflag |= (CREAD | CLOCAL);
            tty.c_oflag |= (OPOST | ONLCR);
            tty.c_lflag |= (ISIG | ICANON | ECHO | ECHOE | ECHOK);
            tcsetattr(STDIN_FILENO, TCSANOW, &tty);
        }

        setenv("USER", "root", 1);
        setenv("LOGNAME", "root", 1);
        setenv("HOME", "/root", 1);
        setenv("TERM", "vt100", 1);
        setenv("PATH", "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/root:.", 1);

        mkdir("/root", 0700);
        if (chdir("/root") != 0) {
            (void)chdir("/");
        }

        const char* shell_candidates[] = {
            "/bin/sh",
            "/bin/bash",
            "/bin/ash",
            "/sbin/sh",
            "/usr/bin/sh"
        };

        for (const char* candidate : shell_candidates) {
            if (access(candidate, X_OK) == 0) {
                char* const args[] = { const_cast<char*>("-sh"), nullptr };
                execv(candidate, args);
            }
        }

        char* const fallback_args[] = { const_cast<char*>("-sh"), nullptr };
        execvp("sh", fallback_args);

        _exit(1);
    }
    log_msg(1, "[INIT] Started interactive console shell (PID: %d)\n", pid);
    return pid;
}
