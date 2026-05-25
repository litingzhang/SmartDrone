#include "platform/linux/gpio/drdy_gpio.h"

#include <cerrno>
#include <cstring>
#include <iostream>

DrdyGpio::~DrdyGpio()
{
    CloseGpiodResources();
    if (m_chip) {
        gpiod_chip_close(m_chip);
    }
}

bool DrdyGpio::Open(const std::string &chipPath, unsigned lineOffset)
{
    if (!OpenChip(chipPath)) {
        return false;
    }
    return OpenLine(lineOffset);
}

bool DrdyGpio::ReadReadyTimestamp(int64_t &tsNsOut)
{
    if (!EdgeEventReady()) {
        return false;
    }

    const int64_t latestTsNs = ReadLatestEventTimestamp();
    if (latestTsNs <= 0) {
        return false;
    }

    tsNsOut = latestTsNs;
    return true;
}

bool DrdyGpio::OpenChip(const std::string &chipPath)
{
    m_chip = gpiod_chip_open(chipPath.c_str());
    if (!m_chip) {
        std::cerr << "gpiod_chip_open(" << chipPath
                  << ") failed: " << strerror(errno) << "\n";
        return false;
    }
    return true;
}
