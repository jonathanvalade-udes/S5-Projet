#pragma once

#include "memory_common.hpp"
#include <string>
#include <vector>
#include <type_traits>

namespace shared::services::memory {

    /**
     * @brief Zero-latency lock-free subscriber for reading shared memory regions.
     *
     * Multiple subscribers can read the same shared memory region concurrently
     * with zero lock contention, guaranteed consistent views via SeqLock,
     * and optional event-driven wakeups via process-shared condition variables.
     */
    class MemoryShareComponentSubscriber {
    private:
        std::string serverName;
        uint32_t lastRefreshed_MS{0};
        int shm_fd{-1};
        void* mmap_ptr{nullptr};
        size_t total_mapped_size{0};
        ShmHeader* header{nullptr};
        bool connected{false};
        uint32_t last_seen_publish_count{0};

        bool tryConnect();
        void cleanup();

    public:
        explicit MemoryShareComponentSubscriber(const std::string& serverName);
        ~MemoryShareComponentSubscriber();

        // Non-copyable
        MemoryShareComponentSubscriber(const MemoryShareComponentSubscriber&) = delete;
        MemoryShareComponentSubscriber& operator=(const MemoryShareComponentSubscriber&) = delete;

        // Movable
        MemoryShareComponentSubscriber(MemoryShareComponentSubscriber&& other) noexcept;
        MemoryShareComponentSubscriber& operator=(MemoryShareComponentSubscriber&& other) noexcept;

        /**
         * @brief Read data into uint32_t vector (legacy/compatibility API)
         */
        bool getData(std::vector<uint32_t>& raw_buffer);

        /**
         * @brief Read data into uint8_t byte vector
         */
        bool getData(std::vector<uint8_t>& raw_buffer);

        /**
         * @brief Read raw bytes directly into destination memory buffer
         */
        bool getData(void* dest, size_t max_size, size_t& actual_size);

        /**
         * @brief Read a raw trivially-copyable struct directly from shared memory
         */
        template <typename T>
        bool read(T& out_data) {
            static_assert(std::is_trivially_copyable_v<T>, "Type must be trivially copyable for raw shared memory");
            size_t actual = 0;
            if (!getData(&out_data, sizeof(T), actual)) {
                return false;
            }
            return actual == sizeof(T);
        }

        /**
         * @brief Wait for the server to publish an update without burning CPU.
         *
         * Uses process-shared condition variable sleeping in the kernel.
         * @param timeout_ms Max milliseconds to wait (0 = wait indefinitely)
         * @return true if updated, false on timeout or error
         */
        bool waitForUpdate(uint32_t timeout_ms = 0);

        /**
         * @brief Get millisecond timestamp of last read data
         */
        uint32_t getLastRefresh() const { return lastRefreshed_MS; }

        bool isConnected() const { return connected && header != nullptr; }
        bool connect() { return tryConnect(); }
        const std::string& getServerName() const { return serverName; }
        size_t getSlotCapacity() const { return header ? header->slot_capacity : 0; }
    };

} // namespace shared::services::memory
