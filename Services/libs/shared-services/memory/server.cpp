#include "server.hpp"
#include <filesystem>
#include <iostream>
#include <cerrno>

namespace shared::services::memory {

MemoryShareComponentServer::MemoryShareComponentServer(const std::string& name, size_t max_buffer_size)
    : name(name), capacity(max_buffer_size) {
    initShm(max_buffer_size);
}

MemoryShareComponentServer::~MemoryShareComponentServer() {
    cleanup();
}

MemoryShareComponentServer::MemoryShareComponentServer(MemoryShareComponentServer&& other) noexcept
    : name(std::move(other.name)),
      lastRefreshed_MS(other.lastRefreshed_MS),
      capacity(other.capacity),
      shm_fd(other.shm_fd),
      mmap_ptr(other.mmap_ptr),
      total_mapped_size(other.total_mapped_size),
      header(other.header),
      initialized(other.initialized) {
    other.shm_fd = -1;
    other.mmap_ptr = nullptr;
    other.header = nullptr;
    other.initialized = false;
    other.total_mapped_size = 0;
}

MemoryShareComponentServer& MemoryShareComponentServer::operator=(MemoryShareComponentServer&& other) noexcept {
    if (this != &other) {
        cleanup();
        name = std::move(other.name);
        lastRefreshed_MS = other.lastRefreshed_MS;
        capacity = other.capacity;
        shm_fd = other.shm_fd;
        mmap_ptr = other.mmap_ptr;
        total_mapped_size = other.total_mapped_size;
        header = other.header;
        initialized = other.initialized;

        other.shm_fd = -1;
        other.mmap_ptr = nullptr;
        other.header = nullptr;
        other.initialized = false;
        other.total_mapped_size = 0;
    }
    return *this;
}

bool MemoryShareComponentServer::initShm(size_t max_buffer_size) {
    std::error_code ec;
    std::filesystem::create_directories(SHM_ROOT_PATH, ec);
    if (ec) {
        std::cerr << "[MemoryShareComponentServer] Failed to create SHM directory "
                  << SHM_ROOT_PATH << ": " << ec.message() << std::endl;
    }

    std::string full_path = std::string(SHM_ROOT_PATH) + name;

    total_mapped_size = calculateTotalShmSize(max_buffer_size);

    shm_fd = open(full_path.c_str(), O_RDWR | O_CREAT, 0666);
    if (shm_fd < 0) {
        std::cerr << "[MemoryShareComponentServer] open failed for " << full_path
                  << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    // Set file permissions explicitly in case umask restricted them
    fchmod(shm_fd, 0666);

    if (ftruncate(shm_fd, static_cast<off_t>(total_mapped_size)) != 0) {
        std::cerr << "[MemoryShareComponentServer] ftruncate failed: "
                  << std::strerror(errno) << std::endl;
        close(shm_fd);
        shm_fd = -1;
        return false;
    }

    mmap_ptr = mmap(nullptr, total_mapped_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (mmap_ptr == MAP_FAILED) {
        std::cerr << "[MemoryShareComponentServer] mmap failed: "
                  << std::strerror(errno) << std::endl;
        close(shm_fd);
        shm_fd = -1;
        mmap_ptr = nullptr;
        return false;
    }

    header = reinterpret_cast<ShmHeader*>(mmap_ptr);

    // Initialize header fields
    header->magic = SHM_MAGIC;
    header->version = SHM_VERSION;
    header->writer_pid = getpid();
    header->slot_capacity = static_cast<uint32_t>(max_buffer_size);
    header->active_slot.store(0, std::memory_order_relaxed);
    header->publish_count.store(0, std::memory_order_relaxed);

    // Initialize slots
    for (uint32_t i = 0; i < 2; ++i) {
        BufferSlot* slot = getSlotHeader(mmap_ptr, i, max_buffer_size);
        slot->seq.store(0, std::memory_order_relaxed);
        slot->data_size.store(0, std::memory_order_relaxed);
        slot->last_refresh_ms.store(0, std::memory_order_relaxed);
        slot->reserved = 0;
    }

    // Initialize robust process-shared mutex
    pthread_mutexattr_t mattr;
    pthread_mutexattr_init(&mattr);
    pthread_mutexattr_setpshared(&mattr, PTHREAD_PROCESS_SHARED);
#ifdef PTHREAD_MUTEX_ROBUST
    pthread_mutexattr_setrobust(&mattr, PTHREAD_MUTEX_ROBUST);
#endif
    pthread_mutex_init(&header->notify_mutex, &mattr);
    pthread_mutexattr_destroy(&mattr);

    // Initialize process-shared condition variable
    pthread_condattr_t cattr;
    pthread_condattr_init(&cattr);
    pthread_condattr_setpshared(&cattr, PTHREAD_PROCESS_SHARED);
#if defined(__linux__)
    pthread_condattr_setclock(&cattr, CLOCK_MONOTONIC);
#endif
    pthread_cond_init(&header->notify_cond, &cattr);
    pthread_condattr_destroy(&cattr);

    initialized = true;
    return true;
}

void MemoryShareComponentServer::cleanup() {
    if (mmap_ptr && mmap_ptr != MAP_FAILED) {
        munmap(mmap_ptr, total_mapped_size);
        mmap_ptr = nullptr;
    }
    if (shm_fd >= 0) {
        close(shm_fd);
        shm_fd = -1;
    }
    header = nullptr;
    initialized = false;
}

bool MemoryShareComponentServer::refreshBuffer(const std::vector<uint8_t>& raw_buffer) {
    return refreshBuffer(raw_buffer.data(), raw_buffer.size());
}

bool MemoryShareComponentServer::refreshBuffer(const void* data, size_t size) {
    if (!isValid() || !data || size == 0) {
        return false;
    }
    if (size > capacity) {
        return false;
    }

    // 1. Identify the inactive slot for writing (readers read the active slot)
    uint32_t active = header->active_slot.load(std::memory_order_relaxed);
    uint32_t target = 1 - (active & 1);

    BufferSlot* target_slot = getSlotHeader(mmap_ptr, target, header->slot_capacity);
    uint8_t* slot_data = getSlotData(mmap_ptr, target, header->slot_capacity);

    // 2. Mark target slot as in-progress (odd sequence)
    uint32_t seq = target_slot->seq.load(std::memory_order_relaxed);
    target_slot->seq.store(seq + 1, std::memory_order_release);

    // 3. Direct memory copy to inactive slot
    std::memcpy(slot_data, data, size);
    target_slot->data_size.store(static_cast<uint32_t>(size), std::memory_order_relaxed);

    uint32_t now_ms = getCurrentTimestampMs();
    target_slot->last_refresh_ms.store(now_ms, std::memory_order_relaxed);
    lastRefreshed_MS = now_ms;

    // 4. Mark target slot as valid (even sequence)
    target_slot->seq.store(seq + 2, std::memory_order_release);

    // 5. Atomically publish target slot as the new active slot
    header->active_slot.store(target, std::memory_order_release);
    header->publish_count.fetch_add(1, std::memory_order_release);

    // 6. Broadcast notification to any waiting subscribers
    pthread_cond_broadcast(&header->notify_cond);

    return true;
}

} // namespace shared::services::memory
