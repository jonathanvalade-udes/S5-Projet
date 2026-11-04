#include "PeripheralsService.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cerrno>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include <signal.h>

#include "hal/sensor_driver.hpp"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif
#include <sys/wait.h>

namespace services {


PeripheralsService::PeripheralsService(std::string socket_path)
    : m_socket_path(std::move(socket_path)),
      m_running(false),
      m_server_fd(-1) {}

PeripheralsService::~PeripheralsService() {
    stop();
    int unexport_fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (unexport_fd >= 0) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", buzzer_gpio);
        [[maybe_unused]] auto _1 = write(unexport_fd, buf, strlen(buf));
        snprintf(buf, sizeof(buf), "%d", wifi_gpio);
        [[maybe_unused]] auto _2 = write(unexport_fd, buf, strlen(buf));
        snprintf(buf, sizeof(buf), "%d", start_gpio);
        [[maybe_unused]] auto _3 = write(unexport_fd, buf, strlen(buf));
        close(unexport_fd);
    }
}

bool PeripheralsService::init() {
    std::cout << "[PeripheralsService] Initializing service with POSIX socket: " << m_socket_path << std::endl;
    cleanup_socket();
    start_buzzer_worker();

    m_server_fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (m_server_fd < 0) {
        std::cerr << "[PeripheralsService] Failed to create POSIX UNIX socket: " << std::strerror(errno) << std::endl;
        return false;
    }

    // Set server socket to non-blocking mode
    int flags = fcntl(m_server_fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(m_server_fd, F_SETFL, flags | O_NONBLOCK);
    }

    // Unlink old socket file if it exists
    ::unlink(m_socket_path.c_str());

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (m_socket_path.size() >= sizeof(addr.sun_path)) {
        std::cerr << "[PeripheralsService] Socket path exceeds maximum length (" 
                  << sizeof(addr.sun_path) << " bytes): " << m_socket_path << std::endl;
        cleanup_socket();
        return false;
    }
    std::strncpy(addr.sun_path, m_socket_path.c_str(), sizeof(addr.sun_path) - 1);

    if (::bind(m_server_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[PeripheralsService] Failed to bind POSIX socket to " << m_socket_path 
                  << ": " << std::strerror(errno) << std::endl;
        cleanup_socket();
        return false;
    }

    if (::listen(m_server_fd, 5) < 0) {
        std::cerr << "[PeripheralsService] Failed to listen on POSIX socket: " 
                  << std::strerror(errno) << std::endl;
        cleanup_socket();
        return false;
    }

    // Allow read/write permissions for IPC clients
    ::chmod(m_socket_path.c_str(), 0666);

    std::cout << "[PeripheralsService] POSIX IPC socket listening at: " << m_socket_path 
              << " (fd=" << m_server_fd << ")" << std::endl;

    m_running.store(true);

    int base = PiCarHatsSensorDriver::detect_gpio_base();

    // set gpios
    buzzer_gpio = base+services::BUZZER_PIN;
    wifi_gpio = base+ services::WIFI_BTN_PIN;
    start_gpio = base+services::START_BTN_PIN;

    //Init them
    int export_fd = open("/sys/class/gpio/export", O_WRONLY);
    if (export_fd >= 0) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", buzzer_gpio);
        [[maybe_unused]] auto _e1 = write(export_fd, buf, strlen(buf));
        snprintf(buf, sizeof(buf), "%d", wifi_gpio);
        [[maybe_unused]] auto _e2 = write(export_fd, buf, strlen(buf));
        snprintf(buf, sizeof(buf), "%d", start_gpio);
        [[maybe_unused]] auto _e3 = write(export_fd, buf, strlen(buf));
        close(export_fd);
    }

    usleep(50000); // 50 ms pour la création des nœuds sysfs

    // Configuration explicite des directions
    char dir_path[64];
    snprintf(dir_path, sizeof(dir_path), "/sys/class/gpio/gpio%d/direction", buzzer_gpio);
    int d_fd = open(dir_path, O_WRONLY);
    if (d_fd >= 0) {
        [[maybe_unused]] auto _d1 = write(d_fd, "out", 3);
        close(d_fd);
    } else {
        std::cerr << "[WARN] Impossible de configurer BUZZER GPIO" << buzzer_gpio << " (BCM " << buzzer_gpio << ") en OUT" << std::endl;
    }

    snprintf(dir_path, sizeof(dir_path), "/sys/class/gpio/gpio%d/direction", wifi_gpio);
    d_fd = open(dir_path, O_WRONLY);
    if (d_fd >= 0) {
        [[maybe_unused]] auto _d2 = write(d_fd, "in", 2);
        close(d_fd);
    } else {
        std::cerr << "[WARN] Impossible de configurer WIFI GPIO " << wifi_gpio << " (BCM " << static_cast<int>(WIFI_BTN_PIN) << ") en IN" << std::endl;
    }

    snprintf(dir_path, sizeof(dir_path), "/sys/class/gpio/gpio%d/direction", start_gpio);
    d_fd = open(dir_path, O_WRONLY);
    if (d_fd >= 0) {
        [[maybe_unused]] auto _d3 = write(d_fd, "in", 2);
        close(d_fd);
    } else {
        std::cerr << "[WARN] Impossible de configurer START GPIO " << start_gpio << " (BCM " << static_cast<int>(START_BTN_PIN) << ") en IN" << std::endl;
    }

    // Initialize idle states (active LOW if pin reads 1)
    int w_val = read_gpio_value(wifi_gpio);
    if (w_val >= 0) {
        m_wifi_idle_val = w_val;
    }
    int s_val = read_gpio_value(start_gpio);
    if (s_val >= 0) {
        m_start_idle_val = s_val;
    }

    // Play initial sound sequence on buzzer
    init_buzzer();

    if(system("ifconfig wlan0 up") < 0){
        std::cerr << "WLAN 0 wasn't configured\n";
    }

    return true;
}

void PeripheralsService::run() {
    std::cout << "[PeripheralsService] Service started and running." << std::endl;

    while (m_running.load()) {
        poll_buttons();

        std::vector<struct pollfd> pfds;

        // Add server socket to poll set
        if (m_server_fd >= 0) {
            pfds.push_back({m_server_fd, POLLIN, 0});
        }

        // Add connected clients to poll set
        {
            std::lock_guard<std::mutex> lock(m_clients_mutex);
            for (int cfd : m_client_fds) {
                pfds.push_back({cfd, POLLIN | POLLHUP | POLLERR, 0});
            }
        }

        int poll_timeout_ms = 5; // Fast polling to ensure responsive button sampling (>= 10ms debounce)
        int ret = ::poll(pfds.data(), static_cast<nfds_t>(pfds.size()), poll_timeout_ms);

        poll_buttons();

        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (m_running.load()) {
                std::cerr << "[PeripheralsService] poll error: " << std::strerror(errno) << std::endl;
            }
            break;
        }

        if (ret == 0) {
            // Periodic processing when no events
            continue;
        }

        for (const auto& pfd : pfds) {
            if (pfd.revents & POLLIN) {
                if (pfd.fd == m_server_fd) {
                    // Incoming IPC client connection
                    int client_fd = ::accept(m_server_fd, nullptr, nullptr);
                    if (client_fd >= 0) {
                        int flags = fcntl(client_fd, F_GETFL, 0);
                        if (flags >= 0) {
                            fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);
                        }
                        {
                            std::lock_guard<std::mutex> lock(m_clients_mutex);
                            m_client_fds.push_back(client_fd);
                            m_client_connected.store(true);
                        }
                        std::cout << "[PeripheralsService] New IPC client connected (fd=" << client_fd << ")." << std::endl;
                    }
                } else {
                    // Data from connected IPC client
                    handle_client_data(pfd.fd);
                }
            } else if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
                if (pfd.fd != m_server_fd) {
                    close_client(pfd.fd);
                }
            }
        }
    }

    std::cout << "[PeripheralsService] Service loop terminated." << std::endl;
}

void PeripheralsService::handle_client_data(int client_fd) {
    uint8_t buffer[2048];
    ssize_t bytes = ::recv(client_fd, buffer, sizeof(buffer), 0);

    if (bytes <= 0) {
        if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return;
        }
        // Client disconnected or connection error
        close_client(client_fd);
    } else {
        std::cout << "[PeripheralsService] Received " << bytes 
                  << " bytes on IPC socket from client (fd=" << client_fd << ")" << std::endl;

        if (m_message_callback) {
            std::vector<uint8_t> data(buffer, buffer + bytes);
            m_message_callback(client_fd, data);
        }
    }
}

bool PeripheralsService::broadcast(const void* data, size_t size) {
    if (!data || size == 0) return false;

    std::lock_guard<std::mutex> lock(m_clients_mutex);
    if (m_client_fds.empty()) {
        return false;
    }

    bool any_success = false;
    for (auto it = m_client_fds.begin(); it != m_client_fds.end(); ) {
        ssize_t sent = ::send(*it, data, size, MSG_NOSIGNAL);
        if (sent < 0) {
            if (errno == EPIPE || errno == ECONNRESET) {
                ::close(*it);
                it = m_client_fds.erase(it);
                continue;
            }
        } else {
            any_success = true;
        }
        ++it;
    }

    m_client_connected.store(!m_client_fds.empty());
    return any_success;
}

bool PeripheralsService::broadcast(const std::string& message) {
    return broadcast(message.data(), message.size());
}

size_t PeripheralsService::get_connected_clients_count() const {
    std::lock_guard<std::mutex> lock(m_clients_mutex);
    return m_client_fds.size();
}

void PeripheralsService::close_client(int client_fd) {
    std::lock_guard<std::mutex> lock(m_clients_mutex);
    auto it = std::find(m_client_fds.begin(), m_client_fds.end(), client_fd);
    if (it != m_client_fds.end()) {
        std::cout << "[PeripheralsService] IPC client disconnected (fd=" << client_fd << ")." << std::endl;
        ::close(client_fd);
        m_client_fds.erase(it);
    }
    m_client_connected.store(!m_client_fds.empty());
}

void PeripheralsService::cleanup_socket() {
    {
        std::lock_guard<std::mutex> lock(m_clients_mutex);
        for (int cfd : m_client_fds) {
            ::close(cfd);
        }
        m_client_fds.clear();
        m_client_connected.store(false);
    }

    if (m_server_fd >= 0) {
        ::close(m_server_fd);
        m_server_fd = -1;
    }

    if (!m_socket_path.empty()) {
        ::unlink(m_socket_path.c_str());
    }
}

void PeripheralsService::stop() {
    if (m_running.exchange(false)) {
        std::cout << "[PeripheralsService] Stopping service..." << std::endl;
        stop_buzzer_worker();
        cleanup_socket();
    }
}

void PeripheralsService::trig_wifi() {
    std::cout << "[PeripheralsService] Triggering WiFi..." << std::endl;
    wifi_buzzer();

    if (m_wifi_callback) {
        m_wifi_callback();
    }

    // 1. If WiFi is currently running, toggle it OFF
    if (m_wifi_pid > 0) {
        std::cout << "[PeripheralsService] Stopping WiFi (PID " << m_wifi_pid << ")..." << std::endl;
        kill(m_wifi_pid, SIGTERM);
        
        // Wait for child to exit to prevent zombie process
        int status = 0;
        waitpid(m_wifi_pid, &status, 0);
        m_wifi_pid = -1;

        // Clean up DHCP client and bring down wlan0
        [[maybe_unused]] auto _k = system("killall -q udhcpc");
        [[maybe_unused]] auto _d = system("ip link set wlan0 down 2>/dev/null");
    } 
    // 2. If WiFi is currently stopped, toggle it ON
    else {
        std::cout << "[PeripheralsService] Starting WiFi..." << std::endl;
        
        pid_t pid = fork();
        if (pid < 0) {
            std::cerr << "[PeripheralsService] Failed to fork WiFi process!" << std::endl;
            play_buzzer_pattern({{50, 80}, {50, 80}, {50, 80}, {50, 80}});
            return;
        }

        if (pid == 0) {
            // === CHILD PROCESS ONLY ===
            // Start wpa_supplicant in foreground (-s for syslog/kmsg)
            char* const args[] = {
                (char*)"wpa_supplicant",
                (char*)"-i", (char*)"wlan0",
                (char*)"-c", (char*)"/etc/wpa_supplicant.conf",
                nullptr
            };
            execvp(args[0], args);
            
            // If execvp fails, exit child immediately with non-zero
            _exit(127);
        }

        // === PARENT PROCESS ONLY ===
        m_wifi_pid = pid;

        // Launch DHCP lease acquisition in the background after wpa_supplicant starts
        std::thread([]() {
            // Wait 2 seconds for wpa_supplicant association
            std::this_thread::sleep_for(std::chrono::seconds(2));
            [[maybe_unused]] auto _dhcp = system("udhcpc -i wlan0 -b -R -t 5 >/dev/null 2>&1");
        }).detach();
    }

    broadcast("TRIGGER_WIFI");
}

void PeripheralsService::trig_start() {
    std::cout << "[PeripheralsService] Triggering Start..." << std::endl;
    start_buzzer();
    broadcast("TRIGGER_START");
}

void PeripheralsService::set_buzzer(bool on) {
    if (buzzer_gpio <= 0) return;
    char val_path[64];
    snprintf(val_path, sizeof(val_path), "/sys/class/gpio/gpio%d/value", buzzer_gpio);
    int fd = open(val_path, O_WRONLY);
    if (fd >= 0) {
        const char* val = on ? "1" : "0";
        [[maybe_unused]] auto _bz = write(fd, val, 1);
        close(fd);
    }
}

void PeripheralsService::start_buzzer_worker() {
    stop_buzzer_worker();
    m_buzzer_running.store(true);
    m_buzzer_thread = std::thread([this]() {
        while (m_buzzer_running.load()) {
            std::vector<std::pair<int, int>> pattern;
            {
                std::unique_lock<std::mutex> lock(m_buzzer_mutex);
                m_buzzer_cv.wait(lock, [this]() {
                    return !m_buzzer_running.load() || m_has_pattern;
                });
                if (!m_buzzer_running.load()) break;
                pattern = std::move(m_current_pattern);
                m_has_pattern = false;
            }

            for (const auto& [on_ms, off_ms] : pattern) {
                if (!m_buzzer_running.load()) break;
                set_buzzer(true);
                auto start = std::chrono::steady_clock::now();
                while (m_buzzer_running.load() &&
                       std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < on_ms) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                set_buzzer(false);
                if (off_ms > 0 && m_buzzer_running.load()) {
                    start = std::chrono::steady_clock::now();
                    while (m_buzzer_running.load() &&
                           std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < off_ms) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    }
                }
            }
            set_buzzer(false);
        }
        set_buzzer(false);
    });
}

void PeripheralsService::stop_buzzer_worker() {
    m_buzzer_running.store(false);
    m_buzzer_cv.notify_all();
    if (m_buzzer_thread.joinable()) {
        m_buzzer_thread.join();
    }
    set_buzzer(false);
}

void PeripheralsService::play_buzzer_pattern(const std::vector<std::pair<int, int>>& pattern) {
    if (!m_buzzer_running.load()) {
        start_buzzer_worker();
    }
    {
        std::lock_guard<std::mutex> lock(m_buzzer_mutex);
        m_current_pattern = pattern;
        m_has_pattern = true;
    }
    m_buzzer_cv.notify_one();
}

void PeripheralsService::init_buzzer() {
    m_last_buzzer_sound.store(BuzzerSound::Init);
    std::cout << "[PeripheralsService] Buzzer: Init sound (2 short beeps)" << std::endl;
    play_buzzer_pattern({{80, 50}, {80, 0}});
}

void PeripheralsService::wifi_buzzer() {
    m_last_buzzer_sound.store(BuzzerSound::Wifi);
    std::cout << "[PeripheralsService] Buzzer: WiFi trigger sound (3 fast chirps)" << std::endl;
    play_buzzer_pattern({{50, 40}, {50, 40}, {120, 0}});
}

void PeripheralsService::start_buzzer() {
    m_last_buzzer_sound.store(BuzzerSound::Start);
    std::cout << "[PeripheralsService] Buzzer: Start trigger sound (race countdown fanfare)" << std::endl;
    play_buzzer_pattern({{100, 80}, {100, 80}, {300, 0}});
}

int PeripheralsService::read_gpio_value(int gpio) {
    if (gpio <= 0) return -1;
    char val_path[64];
    snprintf(val_path, sizeof(val_path), "/sys/class/gpio/gpio%d/value", gpio);
    int fd = open(val_path, O_RDONLY);
    if (fd < 0) return -1;
    char ch = 0;
    ssize_t n = read(fd, &ch, 1);
    close(fd);
    if (n <= 0) return -1;
    return (ch == '1') ? 1 : (ch == '0' ? 0 : -1);
}

bool PeripheralsService::is_button_pressed(uint8_t pin) {
    if (m_mock_gpio) {
        if (pin == WIFI_BTN_PIN) return m_mock_wifi_pressed;
        if (pin == START_BTN_PIN) return m_mock_start_pressed;
        return false;
    }

    if (pin == WIFI_BTN_PIN) {
        int val = read_gpio_value(wifi_gpio);
        if (val < 0) return false;
        return val != m_wifi_idle_val;
    }
    if (pin == START_BTN_PIN) {
        int val = read_gpio_value(start_gpio);
        if (val < 0) return false;
        return val != m_start_idle_val;
    }
    return false;
}

void PeripheralsService::set_mock_button_state(uint8_t pin, bool pressed) {
    if (pin == WIFI_BTN_PIN) {
        m_mock_wifi_pressed = pressed;
    } else if (pin == START_BTN_PIN) {
        m_mock_start_pressed = pressed;
    }
}

void PeripheralsService::update_button(uint8_t pin, ButtonTracker& tracker, const std::function<void()>& on_trigger) {
    bool pressed = is_button_pressed(pin);
    auto now = std::chrono::steady_clock::now();

    if (pressed) {
        if (!tracker.is_down) {
            tracker.is_down = true;
            tracker.press_start = now;
            tracker.triggered = false;
        } else if (!tracker.triggered) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - tracker.press_start).count();
            if (elapsed_ms >= DEBOUNCE_MIN_MS) {
                tracker.triggered = true;
                on_trigger();
            }
        }
    } else {
        tracker.is_down = false;
        tracker.triggered = false;
    }
}

void PeripheralsService::poll_buttons() {
    update_button(WIFI_BTN_PIN, m_wifi_btn_tracker, [this]() {
        trig_wifi();
    });
    update_button(START_BTN_PIN, m_start_btn_tracker, [this]() {
        trig_start();
    });
}

} // namespace services
