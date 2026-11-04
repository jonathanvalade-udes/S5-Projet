#pragma once

#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <functional>
#include <thread>
#include <condition_variable>

// Shared headers from libs
#include "header/header.hpp"
#include "services/services_telemetry.hpp"

namespace services {

constexpr inline uint8_t BUZZER_PIN = 5;
constexpr inline uint8_t WIFI_BTN_PIN = 6;
constexpr inline uint8_t START_BTN_PIN = 13;

enum class BuzzerSound {
    None,
    Init,
    Wifi,
    WifiDisconnect,
    Start
};

class PeripheralsService {
public:
    static constexpr const char* DEFAULT_SOCKET_PATH = "/tmp/peripherals.sock";
    static constexpr int DEBOUNCE_MIN_MS = 10;

    explicit PeripheralsService(std::string socket_path = DEFAULT_SOCKET_PATH);
    ~PeripheralsService();

    bool init();
    void run();
    void stop();

    bool is_running() const { return m_running.load(); }
    const std::string& get_socket_path() const { return m_socket_path; }
    int get_server_fd() const { return m_server_fd; }
    bool is_client_connected() const { return m_client_connected.load(); }
    size_t get_connected_clients_count() const;

    // Send data across the POSIX IPC socket to all connected clients
    bool broadcast(const void* data, size_t size);
    bool broadcast(const std::string& message);

    // Optional callback for receiving IPC client data
    using MessageCallback = std::function<void(int client_fd, const std::vector<uint8_t>& data)>;
    void set_message_callback(MessageCallback callback) { m_message_callback = std::move(callback); }

    // Event triggers
    void trig_wifi();
    void trig_start();

    // Buzzer sounds
    void wifi_buzzer();
    void wifi_disconnect_buzzer();
    void start_buzzer();
    void init_buzzer();
    void set_buzzer(bool on);

    BuzzerSound get_last_buzzer_sound() const { return m_last_buzzer_sound.load(); }

    // User callback hooks for wifi and start enablement
    using TriggerCallback = std::function<void()>;
    void set_wifi_callback(TriggerCallback callback) { m_wifi_callback = std::move(callback); }
    void set_start_callback(TriggerCallback callback) { m_start_callback = std::move(callback); }

    // Button polling & debounce
    void poll_buttons();
    bool is_button_pressed(uint8_t pin);

    // Mock / test injection helpers
    void set_mock_gpio_enabled(bool enabled) { m_mock_gpio = enabled; }
    void set_mock_button_state(uint8_t pin, bool pressed);

private:
    struct ButtonTracker {
        bool is_down{false};
        bool triggered{false};
        std::chrono::steady_clock::time_point press_start{};
    };

    void handle_client_data(int client_fd);
    void close_client(int client_fd);
    void cleanup_socket();

    int read_gpio_value(int gpio);
    void update_button(uint8_t pin, ButtonTracker& tracker, const std::function<void()>& on_trigger);
    void play_buzzer_pattern(const std::vector<std::pair<int, int>>& pattern);
    void start_buzzer_worker();
    void stop_buzzer_worker();

    pid_t m_wifi_pid = -1;
    pid_t m_dhcp_pid = -1;

    int buzzer_gpio = 0, wifi_gpio = 0, start_gpio = 0;
    int m_wifi_idle_val{1};
    int m_start_idle_val{1};

    std::string m_socket_path;
    std::atomic<bool> m_running{false};
    int m_server_fd{-1};

    mutable std::mutex m_clients_mutex;
    std::vector<int> m_client_fds;
    std::atomic<bool> m_client_connected{false};

    MessageCallback m_message_callback;
    TriggerCallback m_wifi_callback;
    TriggerCallback m_start_callback;

    ButtonTracker m_wifi_btn_tracker;
    ButtonTracker m_start_btn_tracker;

    std::thread m_buzzer_thread;
    std::mutex m_buzzer_mutex;
    std::condition_variable m_buzzer_cv;
    std::vector<std::pair<int, int>> m_current_pattern;
    bool m_has_pattern{false};
    std::atomic<bool> m_buzzer_running{false};
    std::atomic<BuzzerSound> m_last_buzzer_sound{BuzzerSound::None};

    bool m_mock_gpio{false};
    bool m_mock_wifi_pressed{false};
    bool m_mock_start_pressed{false};
};

} // namespace services
