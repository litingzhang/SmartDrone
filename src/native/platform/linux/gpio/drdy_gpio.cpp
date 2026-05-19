#include "platform/linux/gpio/drdy_gpio.h"

#include <cerrno>
#include <cstring>
#include <iostream>

DrdyGpio::~DrdyGpio()
{
#if SMART_DRONE_GPIOD_V2
    if (m_eventBuffer) {
        gpiod_edge_event_buffer_free(m_eventBuffer);
    }
    if (m_request) {
        gpiod_line_request_release(m_request);
    }
#else
    if (m_line) {
        gpiod_line_release(m_line);
    }
#endif
    if (m_chip) {
        gpiod_chip_close(m_chip);
    }
}

bool DrdyGpio::Open(const std::string &chipPath, unsigned lineOffset)
{
    if (!OpenChip(chipPath)) {
        return false;
    }
#if SMART_DRONE_GPIOD_V2
    return OpenLineV2(lineOffset);
#else
    return OpenLineV1(lineOffset);
#endif
}

bool DrdyGpio::WaitTs(int timeoutMs, int64_t &tsNsOut)
{
#if SMART_DRONE_GPIOD_V2
    const int64_t timeoutNs = (timeoutMs < 0) ? -1 : static_cast<int64_t>(timeoutMs) * 1000000LL;
    if (gpiod_line_request_wait_edge_events(m_request, timeoutNs) <= 0) {
        return false;
    }
    const int64_t latestTsNs = ReadLatestEventTimestampV2();
#else
    timespec timeoutTs{};
    timespec *timeoutPtr = nullptr;
    if (timeoutMs >= 0) {
        timeoutTs.tv_sec = timeoutMs / 1000;
        timeoutTs.tv_nsec = static_cast<long>(timeoutMs % 1000) * 1000000L;
        timeoutPtr = &timeoutTs;
    }
    if (gpiod_line_event_wait(m_line, timeoutPtr) <= 0) {
        return false;
    }
    const int64_t latestTsNs = ReadLatestEventTimestampV1();
#endif
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
        std::cerr << "gpiod_chip_open(" << chipPath << ") failed: " << strerror(errno) << "\n";
        return false;
    }
    return true;
}

#if SMART_DRONE_GPIOD_V2
bool DrdyGpio::OpenLineV2(unsigned lineOffset)
{
    gpiod_line_settings *settings = gpiod_line_settings_new();
    if (!settings) {
        return false;
    }
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);
    gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);

    gpiod_line_config *lineConfig = gpiod_line_config_new();
    if (!lineConfig) {
        gpiod_line_settings_free(settings);
        return false;
    }

    unsigned offsets[1] = {lineOffset};
    const int rc = gpiod_line_config_add_line_settings(lineConfig, offsets, 1, settings);
    gpiod_line_settings_free(settings);
    if (rc < 0) {
        gpiod_line_config_free(lineConfig);
        return false;
    }

    return CreateLineRequestV2(lineConfig);
}

bool DrdyGpio::CreateLineRequestV2(gpiod_line_config *lineConfig)
{
    gpiod_request_config *requestConfig = gpiod_request_config_new();
    if (!requestConfig) {
        gpiod_line_config_free(lineConfig);
        return false;
    }
    gpiod_request_config_set_consumer(requestConfig, "icm42688_drdy");

    m_request = gpiod_chip_request_lines(m_chip, requestConfig, lineConfig);
    gpiod_request_config_free(requestConfig);
    gpiod_line_config_free(lineConfig);

    if (!m_request) {
        std::cerr << "gpiod_chip_request_lines failed: " << strerror(errno) << "\n";
        return false;
    }

    m_eventBuffer = gpiod_edge_event_buffer_new(256);
    return m_eventBuffer != nullptr;
}

int64_t DrdyGpio::ReadLatestEventTimestampV2()
{
    int64_t latestTsNs = 0;
    while (true) {
        const int eventCount = gpiod_line_request_read_edge_events(m_request, m_eventBuffer, 32);
        if (eventCount <= 0) {
            break;
        }

        for (int i = 0; i < eventCount; ++i) {
            gpiod_edge_event *event = gpiod_edge_event_buffer_get_event(m_eventBuffer, i);
            if (event) {
                latestTsNs = static_cast<int64_t>(gpiod_edge_event_get_timestamp_ns(event));
            }
        }

        if (gpiod_line_request_wait_edge_events(m_request, 0) <= 0) {
            break;
        }
    }
    return latestTsNs;
}
#else
bool DrdyGpio::OpenLineV1(unsigned lineOffset)
{
    m_line = gpiod_chip_get_line(m_chip, lineOffset);
    if (!m_line) {
        std::cerr << "gpiod_chip_get_line(" << lineOffset << ") failed: " << strerror(errno) << "\n";
        return false;
    }

    const int rc = gpiod_line_request_rising_edge_events(m_line, "icm42688_drdy");
    if (rc < 0) {
        std::cerr << "gpiod_line_request_rising_edge_events failed: " << strerror(errno) << "\n";
        return false;
    }

    return true;
}

int64_t DrdyGpio::ReadLatestEventTimestampV1()
{
    int64_t latestTsNs = 0;
    while (true) {
        gpiod_line_event event{};
        if (gpiod_line_event_read(m_line, &event) < 0) {
            break;
        }
        latestTsNs =
            static_cast<int64_t>(event.ts.tv_sec) * 1000000000LL + static_cast<int64_t>(event.ts.tv_nsec);

        timespec pollTs{};
        if (gpiod_line_event_wait(m_line, &pollTs) <= 0) {
            break;
        }
    }
    return latestTsNs;
}
#endif
