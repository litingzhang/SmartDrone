#pragma once

#include <linux/spi/spidev.h>

#include <cstddef>
#include <cstdint>
#include <string>

constexpr uint8_t SPI_READ_MASK = 0x80;

class SpiDev {
public:
    explicit SpiDev(std::string devPath);
    ~SpiDev();

    bool Open(uint32_t speedHz, uint8_t mode, uint8_t bitsPerWord);
    bool WriteReg(uint8_t reg, uint8_t val);
    bool ReadReg(uint8_t reg, uint8_t& val);
    bool ReadRegs(uint8_t startReg, uint8_t* out, size_t len);

private:
    bool Transfer(const uint8_t* tx, uint8_t* rx, size_t len);

    std::string m_devPath;
    int m_fd{-1};
    uint32_t m_speedHz{8000000};
    uint8_t m_mode{SPI_MODE_0};
    uint8_t m_bitsPerWord{8};
};
