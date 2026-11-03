#include <vector>
#include <string>
#include <sys/mount.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/sysmacros.h>
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

void mount_essential_fs(const char* source, const char* target, const char* type, unsigned long flags) {
    if (mount(source, target, type, flags, nullptr) == 0) {
        log_msg(1, "[INIT] Mounted %s onto %s\n", type, target);
    } else {
        if (errno == EBUSY) {
            log_msg(1, "[INIT] Target %s already mounted.\n", target);
        } else {
            log_msg(2, "[INIT] Failed to mount %s: %s\n", target, std::strerror(errno));
        }
    }
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

// Checks whether physical link/carrier is active (cable plugged in)
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

    // Fallback: Check IFF_RUNNING flag via ioctl (RFC 2863 operational state)
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

int main()
{
    umask(0);
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

    setenv("PATH", "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin", 1);

    log_msg(1, "[INIT] STARTING THE SYSTEM...\n");

    configure_static_ip("lo", "127.0.0.1", "255.0.0.0");
    interface_admin_up("lo");

    const char* eth0_ip = "192.168.0.100";
    bool eth_configured = false;
    pid_t pid_dropbear = -1;
    
    log_msg(1, "[INIT] MONITORING SERVICES..\n");
    while(true){
        int status;
        pid_t dead_pid;
        while ((dead_pid = waitpid(-1, &status, WNOHANG)) > 0) {
            if (WIFEXITED(status)) {
                log_msg(1, "[SUPERVISOR] Process %d exited with status %d\n", dead_pid, WEXITSTATUS(status));
            } else if (WIFSIGNALED(status)) {
                log_msg(1, "[SUPERVISOR] Process %d killed by signal %d\n", dead_pid, WTERMSIG(status));
            }
        }

        
        std::string iface = find_ethernet_interface();
        if (!iface.empty()) {
            interface_admin_up(iface.c_str());

            bool link_is_up = check_carrier(iface);

            if (link_is_up && !eth_configured) {
                log_msg(1, "[NET] Interface %s link UP (cable plugged in)! Configuring IP: %s\n", iface.c_str(), eth0_ip);
                configure_static_ip(iface.c_str(), eth0_ip, "255.255.255.0");
                eth_configured = true;

                // Execute dropbear once eth0 is detected and configured
                //if (pid_dropbear <= 0) {
                    //log_msg(1, "[INIT] Executing Dropbear SSH on %s:22...\n", eth0_ip);
                    //pid_dropbear = start_dropbear(eth0_ip);
                //}
            } else if (!link_is_up && eth_configured) {
                log_msg(1, "[NET] Interface %s link DOWN (cable unplugged).\n", iface.c_str());
                eth_configured = false;

                // Stop dropbear when cable is unplugged
                //if (pid_dropbear > 0) {
                    //log_msg(1, "[INIT] Stopping Dropbear SSH (cable unplugged)...\n");
                    //kill(pid_dropbear, SIGTERM);
                    //int drop_status;
                    //waitpid(pid_dropbear, &drop_status, WNOHANG);
                    //pid_dropbear = -1;
                //}
            }
        }

        usleep(100000); 
    }
    
    return 0;
}