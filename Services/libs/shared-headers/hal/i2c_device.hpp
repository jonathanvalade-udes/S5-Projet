#pragma once

#include <cstdint>
#include <cstddef>
#include <iostream>
#include <string>

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#endif

namespace picar::hal {

/**
 * @brief Gestionnaire RAII pour un périphérique I2C sous Linux (/dev/i2c-X).
 * Compatible macOS / non-Linux via stubs inoffensifs.
 */
class I2CDevice {
private:
    std::string m_bus_path;
    uint8_t m_active_addr{0};
    int m_fd{-1};
    bool m_connected{false};

public:
    explicit I2CDevice(uint8_t primary_addr, 
                       uint8_t fallback_addr = 0, 
                       const std::string& bus_path = "/dev/i2c-1")
        : m_bus_path(bus_path) {
#ifdef __linux__
        m_fd = open(m_bus_path.c_str(), O_RDWR);
        if (m_fd >= 0) {
            // Tente l'adresse primaire
            if (ioctl(m_fd, I2C_SLAVE, primary_addr) >= 0) {
                m_active_addr = primary_addr;
                m_connected = true;
            } else if (fallback_addr != 0 && ioctl(m_fd, I2C_SLAVE, fallback_addr) >= 0) {
                m_active_addr = fallback_addr;
                m_connected = true;
            } else {
                std::cerr << "[I2CDevice] Échec d'adressage I2C pour 0x" 
                          << std::hex << (int)primary_addr << " et 0x" << (int)fallback_addr 
                          << std::dec << std::endl;
            }
        } else {
            std::cerr << "[I2CDevice] Impossible d'ouvrir le bus " << m_bus_path << std::endl;
        }
#else
        (void)primary_addr;
        (void)fallback_addr;
        m_connected = false;
#endif
    }

    ~I2CDevice() {
#ifdef __linux__
        if (m_fd >= 0) {
            close(m_fd);
            m_fd = -1;
        }
#endif
        m_connected = false;
    }

    // Interdiction des copies
    I2CDevice(const I2CDevice&) = delete;
    I2CDevice& operator=(const I2CDevice&) = delete;

    // Déplacement autorisé
    I2CDevice(I2CDevice&& other) noexcept 
        : m_bus_path(std::move(other.m_bus_path)),
          m_active_addr(other.m_active_addr),
          m_fd(other.m_fd),
          m_connected(other.m_connected) {
        other.m_fd = -1;
        other.m_connected = false;
    }

    I2CDevice& operator=(I2CDevice&& other) noexcept {
        if (this != &other) {
#ifdef __linux__
            if (m_fd >= 0) close(m_fd);
#endif
            m_bus_path = std::move(other.m_bus_path);
            m_active_addr = other.m_active_addr;
            m_fd = other.m_fd;
            m_connected = other.m_connected;
            other.m_fd = -1;
            other.m_connected = false;
        }
        return *this;
    }

    [[nodiscard]] bool is_connected() const noexcept {
        return m_connected;
    }

    [[nodiscard]] uint8_t get_active_address() const noexcept {
        return m_active_addr;
    }

    /**
     * @brief Écrit un octet de données dans un registre spécifique.
     */
    bool write_byte_data(uint8_t reg, uint8_t value) {
#ifdef __linux__
        if (!m_connected || m_fd < 0) return false;
        uint8_t buffer[2] = {reg, value};
        return (write(m_fd, buffer, 2) == 2);
#else
        (void)reg; (void)value;
        return false;
#endif
    }

    /**
     * @brief Écrit un bloc de données à partir d'un registre de départ.
     */
    bool write_block_data(uint8_t reg, const uint8_t* data, size_t length) {
#ifdef __linux__
        if (!m_connected || m_fd < 0 || !data || length > 32) return false;
        uint8_t buffer[33];
        buffer[0] = reg;
        for (size_t i = 0; i < length; ++i) buffer[i + 1] = data[i];
        return (write(m_fd, buffer, length + 1) == static_cast<ssize_t>(length + 1));
#else
        (void)reg; (void)data; (void)length;
        return false;
#endif
    }

    /**
     * @brief Lit un bloc de données consécutives à partir d'un registre de départ.
     */
    bool read_block_data(uint8_t reg, uint8_t* buffer, size_t length) {
#ifdef __linux__
        if (!m_connected || m_fd < 0 || !buffer) return false;

        // Repeated Start via I2C_RDWR (conforme protocole SMBus bloc)
        struct i2c_msg msgs[2];
        msgs[0].addr = m_active_addr;
        msgs[0].flags = 0;
        msgs[0].len = 1;
        msgs[0].buf = &reg;

        msgs[1].addr = m_active_addr;
        msgs[1].flags = I2C_M_RD;
        msgs[1].len = static_cast<__u16>(length);
        msgs[1].buf = buffer;

        struct i2c_rdwr_ioctl_data rdwr;
        rdwr.msgs = msgs;
        rdwr.nmsgs = 2;

        if (ioctl(m_fd, I2C_RDWR, &rdwr) >= 0) {
            return true;
        }

        // Fallback standard write + read
        if (write(m_fd, &reg, 1) != 1) return false;
        return (read(m_fd, buffer, length) == static_cast<ssize_t>(length));
#else
        (void)reg; (void)buffer; (void)length;
        return false;
#endif
    }

    /**
     * @brief Lit directement sans pointer un registre de départ.
     */
    bool read_raw_bytes(uint8_t* buffer, size_t length) {
#ifdef __linux__
        if (!m_connected || m_fd < 0 || !buffer) return false;
        return (read(m_fd, buffer, length) == static_cast<ssize_t>(length));
#else
        (void)buffer; (void)length;
        return false;
#endif
    }
};

} // namespace picar::hal
