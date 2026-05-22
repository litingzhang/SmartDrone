#include "platform/linux/gpio/drdy_gpio.h"

#include <cerrno>
#include <cstring>
#include <iostream>

namespace {

gpiod_line_request *AsRequest(void *request)
{
    return static_cast<gpiod_line_request *>(request);
}

gpiod_edge_event_buffer *AsEventBuffer(void *eventBuffer)
{
    return static_cast<gpiod_edge_event_buffer *>(eventBuffer);
}

bool ConfigureInputLine(gpiod_line_config *lineConfig,
                        unsigned lineOffset)
{
    gpiod_line_settings *settings = gpiod_line_settings_new();
    if (!settings) {
        return false;
    }
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);
    gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);

    unsigned offsets[1] = {lineOffset};
    const int rc = gpiod_line_config_add_line_settings(lineConfig, offsets, 1,
                                                       settings);
    gpiod_line_settings_free(settings);
    return rc >= 0;
}

} // namespace

void DrdyGpio::CloseGpiodResources()
{
    if (m_eventBuffer) {
        gpiod_edge_event_buffer_free(AsEventBuffer(m_eventBuffer));
    }
    if (m_request) {
        gpiod_line_request_release(AsRequest(m_request));
    }
}

bool DrdyGpio::OpenLine(unsigned lineOffset)
{
    gpiod_line_config *lineConfig = gpiod_line_config_new();
    if (!lineConfig) {
        return false;
    }
    if (!ConfigureInputLine(lineConfig, lineOffset)) {
        gpiod_line_config_free(lineConfig);
        return false;
    }

    gpiod_request_config *requestConfig = gpiod_request_config_new();
    if (!requestConfig) {
        gpiod_line_config_free(lineConfig);
        return false;
    }
    gpiod_request_config_set_consumer(requestConfig, "icm42688_drdy");

    gpiod_line_request *request =
        gpiod_chip_request_lines(m_chip, requestConfig, lineConfig);
    gpiod_request_config_free(requestConfig);
    gpiod_line_config_free(lineConfig);

    if (!request) {
        std::cerr << "gpiod_chip_request_lines failed: " << strerror(errno)
                  << "\n";
        return false;
    }

    m_request = request;
    m_eventBuffer = gpiod_edge_event_buffer_new(256);
    return m_eventBuffer != nullptr;
}

bool DrdyGpio::WaitForGpiodEvent(int timeoutMs)
{
    const int64_t timeoutNs =
        (timeoutMs < 0) ? -1 : static_cast<int64_t>(timeoutMs) * 1000000LL;
    return gpiod_line_request_wait_edge_events(AsRequest(m_request), timeoutNs) >
           0;
}

int64_t DrdyGpio::ReadLatestEventTimestamp()
{
    int64_t latestTsNs = 0;
    while (true) {
        const int eventCount =
            gpiod_line_request_read_edge_events(AsRequest(m_request),
                                                AsEventBuffer(m_eventBuffer),
                                                32);
        if (eventCount <= 0) {
            break;
        }

        for (int i = 0; i < eventCount; ++i) {
            gpiod_edge_event *event =
                gpiod_edge_event_buffer_get_event(AsEventBuffer(m_eventBuffer),
                                                  i);
            if (event) {
                latestTsNs =
                    static_cast<int64_t>(gpiod_edge_event_get_timestamp_ns(event));
            }
        }

        if (gpiod_line_request_wait_edge_events(AsRequest(m_request), 0) <= 0) {
            break;
        }
    }
    return latestTsNs;
}
