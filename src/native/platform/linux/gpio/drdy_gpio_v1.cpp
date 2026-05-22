#include "platform/linux/gpio/drdy_gpio.h"

#include <cerrno>
#include <cstring>
#include <iostream>

namespace {

gpiod_line *AsLine(void *line)
{
    return static_cast<gpiod_line *>(line);
}

} // namespace

void DrdyGpio::CloseGpiodResources()
{
    if (m_line) {
        gpiod_line_release(AsLine(m_line));
    }
}

bool DrdyGpio::OpenLine(unsigned lineOffset)
{
    gpiod_line *line = gpiod_chip_get_line(m_chip, lineOffset);
    if (!line) {
        std::cerr << "gpiod_chip_get_line(" << lineOffset
                  << ") failed: " << strerror(errno) << "\n";
        return false;
    }

    const int rc = gpiod_line_request_rising_edge_events(line, "icm42688_drdy");
    if (rc < 0) {
        std::cerr << "gpiod_line_request_rising_edge_events failed: "
                  << strerror(errno) << "\n";
        return false;
    }

    m_line = line;
    return true;
}

bool DrdyGpio::WaitForGpiodEvent(int timeoutMs)
{
    timespec timeoutTs{};
    timespec *timeoutPtr = nullptr;
    if (timeoutMs >= 0) {
        timeoutTs.tv_sec = timeoutMs / 1000;
        timeoutTs.tv_nsec = static_cast<long>(timeoutMs % 1000) * 1000000L;
        timeoutPtr = &timeoutTs;
    }
    return gpiod_line_event_wait(AsLine(m_line), timeoutPtr) > 0;
}

int64_t DrdyGpio::ReadLatestEventTimestamp()
{
    int64_t latestTsNs = 0;
    while (true) {
        gpiod_line_event event{};
        if (gpiod_line_event_read(AsLine(m_line), &event) < 0) {
            break;
        }
        latestTsNs = static_cast<int64_t>(event.ts.tv_sec) * 1000000000LL +
                     static_cast<int64_t>(event.ts.tv_nsec);

        timespec pollTs{};
        if (gpiod_line_event_wait(AsLine(m_line), &pollTs) <= 0) {
            break;
        }
    }
    return latestTsNs;
}
