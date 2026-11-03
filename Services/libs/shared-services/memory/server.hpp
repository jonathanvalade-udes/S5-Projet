#pragma once

#include "memory_common.hpp"
#include <string>
#include <vector>
#include <type_traits>

namespace shared::services::memory {

    /**
     * @brief Zero-latency single-writer publisher for shared memory regions.
     *
     * Uses double-buffering and SeqLock synchronization to allow wait-free,
     * non-blocking updates without torn reads and with zero reader locks.
     */
    class MemoryShareComponentServer {
    private:
        std::string name;
        uint32_t lastRefreshed_MS{0};
        size_t capacity{0};
        int shm_fd{-1};
        void* mmap_ptr{nullptr};
        size_t total_mapped_size{0};
        ShmHeader* header{nullptr};
        bool initialized{false};

        bool initShm(size_t max_buffer_size);
        void cleanup();

    public:
        explicit MemoryShareComponentServer(const std::string& name, size_t max_buffer_size = DEFAULT_BUFFER_CAPACITY);
        ~MemoryShareComponentServer();

        // Non-copyable
        MemoryShareComponentServer(const MemoryShareComponentServer&) = delete;
        MemoryShareComponentServer& operator=(const MemoryShareComponentServer&) = delete;

        // Movable
        MemoryShareComponentServer(MemoryShareComponentServer&& other) noexcept;
        MemoryShareComponentServer& operator=(MemoryShareComponentServer&& other) noexcept;

        /**
         * @brief Publish a byte vector to the shared memory region
         */
        bool refreshBuffer(const std::vector<uint8_t>& raw_buffer);

        /**
         * @brief Publish raw memory buffer to the shared memory region (zero-latency)
         */
        bool refreshBuffer(const void* data, size_t size);

        /**
         * @brief Publish a raw trivially-copyable struct directly to shared memory
         */
        template <typename T>
        bool publish(const T& data) {
            static_assert(std::is_trivially_copyable_v<T>, "Type must be trivially copyable for raw shared memory");
            return refreshBuffer(&data, sizeof(T));
        }

        /**
         * @brief Get millisecond timestamp of last refresh
         */
        uint32_t getLastRefresh() const { return lastRefreshed_MS; }

        bool isValid() const { return initialized && header != nullptr; }
        const std::string& getName() const { return name; }
        size_t getCapacity() const { return capacity; }
    };

} // namespace shared::services::memory
