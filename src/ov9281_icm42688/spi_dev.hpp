#pragma once

#include <linux/spi/spidev.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>
#include <vector>

constexpr uint8_t SPI_READ_MASK = 0x80;

class SpiDev {
public:
    explicit SpiDev(std::string devPath) : m_devPath(std::move(devPath)) {}

    ~SpiDev()
    {
        if (m_fd >= 0) {
            ::close(m_fd);
        }
    }

    bool Open(uint32_t speedHz, uint8_t mode, uint8_t bitsPerWord)
    {
        m_fd = ::open(m_devPath.c_str(), O_RDWR);
        if (m_fd < 0) {
            std::cerr << "open " << m_devPath << " failed: " << strerror(errno) << "\n";
            return false;
        }
        if (ioctl(m_fd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(m_fd, SPI_IOC_RD_MODE, &mode) < 0) {
            std::cerr << "SPI set mode failed: " << strerror(errno) << "\n";
            return false;
        }
        if (ioctl(m_fd, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) < 0 ||
            ioctl(m_fd, SPI_IOC_RD_BITS_PER_WORD, &bitsPerWord) < 0) {
            std::cerr << "SPI set bits failed: " << strerror(errno) << "\n";
            return false;
        }
        if (ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speedHz) < 0 ||
            ioctl(m_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speedHz) < 0) {
            std::cerr << "SPI set speed failed: " << strerror(errno) << "\n";
            return false;
        }
        m_speedHz = speedHz;
        m_mode = mode;
        m_bitsPerWord = bitsPerWord;
        return true;
    }

    bool WriteReg(uint8_t reg, uint8_t val)
    {
        uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), val};
        uint8_t rx[2] = {0, 0};
        return Transfer(tx, rx, sizeof(tx));
    }

    bool ReadReg(uint8_t reg, uint8_t& val)
    {
        uint8_t tx[2] = {static_cast<uint8_t>(SPI_READ_MASK | (reg & 0x7F)), 0x00};
        uint8_t rx[2] = {0, 0};
        if (!Transfer(tx, rx, sizeof(tx))) {
            return false;
        }
        val = rx[1];
        return true;
    }

    bool ReadRegs(uint8_t startReg, uint8_t* out, size_t len)
    {
        std::vector<uint8_t> tx(len + 1, 0x00);
        std::vector<uint8_t> rx(len + 1, 0x00);
        tx[0] = static_cast<uint8_t>(SPI_READ_MASK | (startReg & 0x7F));
        if (!Transfer(tx.data(), rx.data(), rx.size())) {
            return false;
        }
        std::memcpy(out, rx.data() + 1, len);
        return true;
    }

private:
    bool Transfer(const uint8_t* tx, uint8_t* rx, size_t len)
    {
        spi_ioc_transfer transfer{};
        transfer.tx_buf = reinterpret_cast<unsigned long>(tx);
        transfer.rx_buf = reinterpret_cast<unsigned long>(rx);
        transfer.len = static_cast<uint32_t>(len);
        transfer.speed_hz = m_speedHz;
        transfer.bits_per_word = m_bitsPerWord;
        transfer.delay_usecs = 0;
        if (ioctl(m_fd, SPI_IOC_MESSAGE(1), &transfer) < 0) {
            std::cerr << "SPI transfer failed: " << strerror(errno) << "\n";
            return false;
        }
        return true;
    }

    std::string m_devPath;
    int m_fd{-1};
    uint32_t m_speedHz{8000000};
    uint8_t m_mode{SPI_MODE_0};
    uint8_t m_bitsPerWord{8};
};
