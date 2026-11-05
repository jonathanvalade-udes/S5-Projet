#include "UsbLogger.hpp"
#include "fsm/machine_states.hpp"
#include <unistd.h>
#include <ctime>
#include <sstream>
#include <vector>

namespace services {

static const char* fsm_state_to_str(uint32_t state) {
    switch (static_cast<shared::fsm::MachineStates>(state)) {
        case shared::fsm::MachineStates::Init: return "Init";
        case shared::fsm::MachineStates::Unarmed: return "Unarmed";
        case shared::fsm::MachineStates::Armed: return "Armed";
        case shared::fsm::MachineStates::LineFollowingForward: return "LineFollowingForward";
        case shared::fsm::MachineStates::CollisionRisk: return "CollisionRisk";
        case shared::fsm::MachineStates::Stopped: return "Stopped";
        case shared::fsm::MachineStates::RightDeviation: return "RightDeviation";
        case shared::fsm::MachineStates::LeftDeviation: return "LeftDeviation";
        case shared::fsm::MachineStates::Abort: return "Abort";
        case shared::fsm::MachineStates::Failure: return "Failure";
        default: return "Undefined";
    }
}

UsbLogger::UsbLogger() : m_active(false), m_running(false) {}

UsbLogger::~UsbLogger() {
    stop();
}

std::string UsbLogger::detect_mount_point() {
    const char* env_path = std::getenv("PICAR_USB_PATH");
    if (env_path && env_path[0] != '\0') {
        return std::string(env_path);
    }

    std::ifstream mounts("/proc/mounts");
    if (mounts.is_open()) {
        std::string line;
        while (std::getline(mounts, line)) {
            std::istringstream iss(line);
            std::string device, mount_point, fs_type;
            if (iss >> device >> mount_point >> fs_type) {
                if (mount_point == "/" || mount_point == "/mnt/wsl" || mount_point.rfind("/mnt/wslg", 0) == 0) {
                    continue;
                }
                if ((mount_point.rfind("/media/", 0) == 0 || mount_point.rfind("/mnt/", 0) == 0 ||
                     device.rfind("/dev/sd", 0) == 0) && mount_point != "/mnt") {
#if !defined(_WIN32)
                    if (access(mount_point.c_str(), W_OK) == 0) {
                        return mount_point;
                    }
#else
                    return mount_point;
#endif
                }
            }
        }
    }

    const std::vector<std::string> standard_paths = {
        "/media/usb", "/mnt/usb", "/media/pi", "/media/picar"
    };
    for (const auto& p : standard_paths) {
#if !defined(_WIN32)
        if (access(p.c_str(), W_OK) == 0) {
            return p;
        }
#else
        return p;
#endif
    }

    return "";
}

bool UsbLogger::init() {
    std::string mount = detect_mount_point();
    if (mount.empty()) {
        m_active = false;
        return false;
    }

    m_active = true;
    m_running.store(true);
    m_thread = std::thread(&UsbLogger::worker_loop, this);
    return true;
}

void UsbLogger::stop() {
    if (m_running.load()) {
        m_running.store(false);
    }
    m_cv.notify_all();
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void UsbLogger::push_sample(const TelemetrySample& sample) {
    if (!m_active) return;

    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_fifo.size() >= MAX_FIFO_SIZE) {
        m_fifo.pop();
    }
    m_fifo.push(sample);
    if (m_fifo.size() % BATCH_NOTIFY_SIZE == 0) {
        m_cv.notify_one();
    }
}

void UsbLogger::worker_loop() {
    const std::string csv_header = "timestamp_us,uptime_s,fsm_state,speed_ms,accel_ms2,steer_deg,steer_offset_deg,dist_us_cm,line_0,line_1,line_2,line_3,line_4,odometry_m,emergency_stop,accel_violations\n";

    char line_buf[256];
    std::string write_chunk;
    write_chunk.reserve(65536);

    while (m_running.load() || !m_fifo.empty()) {
        std::vector<TelemetrySample> batch;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait_for(lock, std::chrono::milliseconds(20), [&]() {
                return !m_fifo.empty() || !m_running.load();
            });

            while (!m_fifo.empty()) {
                batch.push_back(m_fifo.front());
                m_fifo.pop();
            }
        }

        if (batch.empty()) {
            continue;
        }

        if (!m_stream.is_open()) {
            std::string current_mount = detect_mount_point();
            if (!current_mount.empty()) {
                auto now = std::chrono::system_clock::now();
                std::time_t now_c = std::chrono::system_clock::to_time_t(now);
                std::tm now_tm{};
#if defined(_WIN32)
                localtime_s(&now_tm, &now_c);
#else
                localtime_r(&now_c, &now_tm);
#endif
                char time_str[32];
                std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", &now_tm);

                const std::string file_path = current_mount + "/telemetry_run_" + std::string(time_str) + ".csv";
                m_stream.open(file_path, std::ios::out | std::ios::app);
                if (m_stream.is_open()) {
                    m_stream << csv_header;
                    m_stream.flush();
                }
            }
        }

        if (m_stream.is_open()) {
            write_chunk.clear();
            for (const auto& s : batch) {
                int len = snprintf(line_buf, sizeof(line_buf),
                    "%llu,%u,%s,%.3f,%.3f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%u,%.2f,%u,%u\n",
                    static_cast<unsigned long long>(s.timestamp_us),
                    s.uptime_s,
                    fsm_state_to_str(s.fsm_state),
                    s.speed_ms,
                    s.accel_ms2,
                    s.steer_deg,
                    s.steer_offset_deg,
                    s.dist_us_cm,
                    static_cast<unsigned>(s.line_sensors[0]),
                    static_cast<unsigned>(s.line_sensors[1]),
                    static_cast<unsigned>(s.line_sensors[2]),
                    static_cast<unsigned>(s.line_sensors[3]),
                    static_cast<unsigned>(s.line_sensors[4]),
                    s.odometry_m,
                    static_cast<unsigned>(s.emergency_stop),
                    static_cast<unsigned>(s.accel_violations));
                if (len > 0) {
                    write_chunk.append(line_buf, static_cast<size_t>(len));
                }
            }
            m_stream.write(write_chunk.data(), static_cast<std::streamsize>(write_chunk.size()));
            m_stream.flush();
        }
    }

    if (m_stream.is_open()) {
        m_stream.flush();
#if !defined(_WIN32)
        sync();
#endif
        m_stream.close();
    }
}

} // namespace services
