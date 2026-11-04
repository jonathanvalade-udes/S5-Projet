#include "subscriber.hpp"
#include <iostream>
#include <cerrno>

namespace shared::services::memory {

MemoryShareComponentSubscriber::MemoryShareComponentSubscriber(const std::string& serverName)
    : serverName(serverName) {
    tryConnect();
}

MemoryShareComponentSubscriber::~MemoryShareComponentSubscriber() {
    cleanup();
}

MemoryShareComponentSubscriber::MemoryShareComponentSubscriber(MemoryShareComponentSubscriber&& other) noexcept
    : serverName(std::move(other.serverName)),
      lastRefreshed_MS(other.lastRefreshed_MS),
      shm_fd(other.shm_fd),
      mmap_ptr(other.mmap_ptr),
      total_mapped_size(other.total_mapped_size),
      header(other.header),
      connected(other.connected),
      last_seen_publish_count(other.last_seen_publish_count) {
    other.shm_fd = -1;
    other.mmap_ptr = nullptr;
    other.header = nullptr;
    other.connected = false;
    other.total_mapped_size = 0;
}

MemoryShareComponentSubscriber& MemoryShareComponentSubscriber::operator=(MemoryShareComponentSubscriber&& other) noexcept {
    if (this != &other) {
        cleanup();
        serverName = std::move(other.serverName);
        lastRefreshed_MS = other.lastRefreshed_MS;
        shm_fd = other.shm_fd;
        mmap_ptr = other.mmap_ptr;
        total_mapped_size = other.total_mapped_size;
        header = other.header;
        connected = other.connected;
        last_seen_publish_count = other.last_seen_publish_count;

        other.shm_fd = -1;
        other.mmap_ptr = nullptr;
        other.header = nullptr;
        other.connected = false;
        other.total_mapped_size = 0;
    }
    return *this;
}

bool MemoryShareComponentSubscriber::tryConnect() {
    if (connected && header != nullptr) {
        return true;
    }

    std::string full_path = std::string(SHM_ROOT_PATH) + serverName;

    int fd = open(full_path.c_str(), O_RDWR);
    if (fd < 0) {
        connected = false;
        return false;
    }

    struct stat st;
    if (fstat(fd, &st) == -1 || st.st_size < static_cast<off_t>(sizeof(ShmHeader))) {
        close(fd);
        connected = false;
        return false;
    }

    void* ptr = mmap(nullptr, st.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        close(fd);
        connected = false;
        return false;
    }

    ShmHeader* h = reinterpret_cast<ShmHeader*>(ptr);
    if (h->magic != SHM_MAGIC || h->version != SHM_VERSION) {
        munmap(ptr, st.st_size);
        close(fd);
        connected = false;
        return false;
    }

    shm_fd = fd;
    mmap_ptr = ptr;
    total_mapped_size = st.st_size;
    header = h;
    last_seen_publish_count = header->publish_count.load(std::memory_order_relaxed);
    connected = true;

    return true;
}

void MemoryShareComponentSubscriber::cleanup() {
    if (mmap_ptr && mmap_ptr != MAP_FAILED) {
        munmap(mmap_ptr, total_mapped_size);
        mmap_ptr = nullptr;
    }
    if (shm_fd >= 0) {
        close(shm_fd);
        shm_fd = -1;
    }
    header = nullptr;
    connected = false;
}

bool MemoryShareComponentSubscriber::getData(void* dest, size_t max_size, size_t& actual_size) {
    if (!dest || max_size == 0) {
        return false;
    }
    if (!isConnected() && !tryConnect()) {
        return false;
    }

    for (int retry = 0; retry < MAX_READ_RETRIES; ++retry) {
        // 1. Read the index of the currently active slot
        uint32_t active = header->active_slot.load(std::memory_order_acquire) & 1;

        BufferSlot* slot = getSlotHeader(mmap_ptr, active, header->slot_capacity);
        uint8_t* slot_data = getSlotData(mmap_ptr, active, header->slot_capacity);

        // 2. Check SeqLock start counter
        uint32_t seq1 = slot->seq.load(std::memory_order_acquire);
        if (seq1 & 1) {
            // Write currently in progress on this slot
            cpu_relax();
            continue;
        }

        uint32_t data_sz = slot->data_size.load(std::memory_order_acquire);
        uint32_t refresh_ts = slot->last_refresh_ms.load(std::memory_order_relaxed);

        size_t bytes_to_copy = std::min(static_cast<size_t>(data_sz), max_size);
        std::memcpy(dest, slot_data, bytes_to_copy);

        // 3. Memory fence to ensure data copy precedes the second seq check
        std::atomic_thread_fence(std::memory_order_acquire);
        uint32_t seq2 = slot->seq.load(std::memory_order_acquire);

        // 4. If sequence counter matches, the read is valid and uncorrupted
        if (seq1 == seq2) {
            actual_size = bytes_to_copy;
            lastRefreshed_MS = refresh_ts;
            return true;
        }

        // Slot was overwritten during copy, pause/yield CPU and retry
        if (retry > 2) {
            std::this_thread::yield();
        } else {
            cpu_relax();
        }
    }

    return false;
}

bool MemoryShareComponentSubscriber::getData(std::vector<uint8_t>& raw_buffer) {
    if (!isConnected() && !tryConnect()) {
        return false;
    }

    raw_buffer.resize(header->slot_capacity);
    size_t actual_bytes = 0;
    if (!getData(raw_buffer.data(), raw_buffer.size(), actual_bytes)) {
        raw_buffer.clear();
        return false;
    }
    raw_buffer.resize(actual_bytes);
    return true;
}

bool MemoryShareComponentSubscriber::getData(std::vector<uint32_t>& raw_buffer) {
    if (!isConnected() && !tryConnect()) {
        return false;
    }

    size_t max_bytes = header->slot_capacity;
    size_t num_elements = (max_bytes + sizeof(uint32_t) - 1) / sizeof(uint32_t);
    raw_buffer.resize(num_elements);

    size_t actual_bytes = 0;
    if (!getData(raw_buffer.data(), raw_buffer.size() * sizeof(uint32_t), actual_bytes)) {
        raw_buffer.clear();
        return false;
    }

    size_t actual_elements = (actual_bytes + sizeof(uint32_t) - 1) / sizeof(uint32_t);
    raw_buffer.resize(actual_elements);
    return true;
}

bool MemoryShareComponentSubscriber::waitForUpdate(uint32_t timeout_ms) {
    if (!isConnected() && !tryConnect()) {
        return false;
    }

    int rc = pthread_mutex_lock(&header->notify_mutex);
#ifdef EOWNERDEAD
    if (rc == EOWNERDEAD) {
#ifdef PTHREAD_MUTEX_ROBUST
        pthread_mutex_consistent(&header->notify_mutex);
#endif
    } else
#endif
    if (rc != 0) {
        return false;
    }

    uint32_t current_count = header->publish_count.load(std::memory_order_acquire);
    if (current_count != last_seen_publish_count) {
        last_seen_publish_count = current_count;
        pthread_mutex_unlock(&header->notify_mutex);
        return true;
    }

    bool updated = false;
    if (timeout_ms == 0) {
        while (header->publish_count.load(std::memory_order_acquire) == last_seen_publish_count) {
            rc = pthread_cond_wait(&header->notify_cond, &header->notify_mutex);
            if (rc != 0) {
                break;
            }
        }
        if (header->publish_count.load(std::memory_order_acquire) != last_seen_publish_count) {
            last_seen_publish_count = header->publish_count.load(std::memory_order_acquire);
            updated = true;
        }
    } else {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        ts.tv_sec += timeout_ms / 1000;
        ts.tv_nsec += (timeout_ms % 1000) * 1000000L;
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec += ts.tv_nsec / 1000000000L;
            ts.tv_nsec %= 1000000000L;
        }

        while (header->publish_count.load(std::memory_order_acquire) == last_seen_publish_count) {
            rc = pthread_cond_timedwait(&header->notify_cond, &header->notify_mutex, &ts);
            if (rc == ETIMEDOUT || rc != 0) {
                break;
            }
        }
        if (header->publish_count.load(std::memory_order_acquire) != last_seen_publish_count) {
            last_seen_publish_count = header->publish_count.load(std::memory_order_acquire);
            updated = true;
        }
    }

    pthread_mutex_unlock(&header->notify_mutex);
    return updated;
}

} // namespace shared::services::memory
