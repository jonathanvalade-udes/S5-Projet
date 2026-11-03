#pragma once

#include <string_view>
#include <string>
#include <cstdint>
#include <atomic>
#include <chrono>
#include <thread>
#include <cstring>
#include <algorithm>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

namespace shared::services::memory {

    constexpr inline std::string_view SHM_ROOT_PATH = "/dev/shm/car/";
    constexpr uint32_t SHM_MAGIC = 0x53484D31; // "SHM1"
    constexpr uint32_t SHM_VERSION = 1;
    constexpr uint32_t DEFAULT_BUFFER_CAPACITY = 65536; // 64 KB
    constexpr int MAX_READ_RETRIES = 16;

    /**
     * @brief Low-overhead CPU relaxation to prevent CPU overwhelming during brief contention
     */
    inline void cpu_relax() noexcept {
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__)
        asm volatile("pause" ::: "memory");
#elif defined(__aarch64__) || defined(__arm__)
        asm volatile("yield" ::: "memory");
#else
        std::this_thread::yield();
#endif
    }

    /**
     * @brief Monotonic clock timestamp in milliseconds
     */
    inline uint32_t getCurrentTimestampMs() noexcept {
        auto now = std::chrono::steady_clock::now().time_since_epoch();
        return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
    }

    inline size_t align64(size_t size) noexcept {
        return (size + 63) & ~size_t(63);
    }

    /**
     * @brief Header for an individual buffer slot (for double-buffering)
     */
    struct alignas(64) BufferSlot {
        std::atomic<uint32_t> seq{0};             // SeqLock counter: odd = write in progress, even = valid
        std::atomic<uint32_t> data_size{0};       // Actual data payload length in bytes
        std::atomic<uint32_t> last_refresh_ms{0}; // Millisecond timestamp of last write
        uint32_t reserved{0};
    };

    /**
     * @brief Master shared memory header placed at offset 0 of the mapped file
     */
    struct alignas(64) ShmHeader {
        uint32_t magic{0};                        // SHM_MAGIC
        uint32_t version{0};                      // SHM_VERSION
        pid_t writer_pid{0};                      // Server process ID
        std::atomic<uint32_t> active_slot{0};     // 0 or 1: currently active slot for readers
        std::atomic<uint32_t> publish_count{0};   // Total published update counter
        uint32_t slot_capacity{0};                // Maximum capacity in bytes for each slot
        pthread_mutex_t notify_mutex;             // Process-shared robust mutex for notifications
        pthread_cond_t  notify_cond;              // Process-shared condition variable for update notifications
    };

    inline BufferSlot* getSlotHeader(void* base, uint32_t index, size_t slot_capacity) noexcept {
        size_t aligned_slot_cap = align64(slot_capacity);
        size_t slot_stride = sizeof(BufferSlot) + aligned_slot_cap;
        uint8_t* ptr = static_cast<uint8_t*>(base) + sizeof(ShmHeader) + index * slot_stride;
        return reinterpret_cast<BufferSlot*>(ptr);
    }

    inline uint8_t* getSlotData(void* base, uint32_t index, size_t slot_capacity) noexcept {
        BufferSlot* slot = getSlotHeader(base, index, slot_capacity);
        return reinterpret_cast<uint8_t*>(slot) + sizeof(BufferSlot);
    }

    inline size_t calculateTotalShmSize(size_t slot_capacity) noexcept {
        size_t aligned_slot_cap = align64(slot_capacity);
        size_t slot_stride = sizeof(BufferSlot) + aligned_slot_cap;
        return sizeof(ShmHeader) + 2 * slot_stride;
    }

} // namespace shared::services::memory
