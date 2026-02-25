#pragma once

#include <gpiod.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

class DrdyGpio {
public:
    bool Open(const std::string& chipPath, unsigned lineOffset)
    {
        m_chip = gpiod_chip_open(chipPath.c_str());
        if (!m_chip) {
            std::cerr << "gpiod_chip_open(" << chipPath << ") failed: " << strerror(errno)
                      << "\n";
            return false;
        }

        gpiod_line_settings* settings = gpiod_line_settings_new();
        if (!settings) {
            return false;
        }
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
        gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);
        gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);

        gpiod_line_config* lineConfig = gpiod_line_config_new();
        if (!lineConfig) {
            gpiod_line_settings_free(settings);
            return false;
        }

        unsigned offsets[1] = {lineOffset};
        int rc = gpiod_line_config_add_line_settings(lineConfig, offsets, 1, settings);
        gpiod_line_settings_free(settings);
        if (rc < 0) {
            gpiod_line_config_free(lineConfig);
            return false;
        }

        gpiod_request_config* requestConfig = gpiod_request_config_new();
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

    ~DrdyGpio()
    {
        if (m_eventBuffer) {
            gpiod_edge_event_buffer_free(m_eventBuffer);
        }
        if (m_request) {
            gpiod_line_request_release(m_request);
        }
        if (m_chip) {
            gpiod_chip_close(m_chip);
        }
    }

    bool WaitTs(int timeoutMs, int64_t& tsNsOut)
    {
        int64_t timeoutNs = (timeoutMs < 0) ? -1 : static_cast<int64_t>(timeoutMs) * 1000000LL;
        int ret = gpiod_line_request_wait_edge_events(m_request, timeoutNs);
        if (ret <= 0) {
            return false;
        }

        int eventCount = gpiod_line_request_read_edge_events(m_request, m_eventBuffer, 1);
        if (eventCount <= 0) {
            return false;
        }

        gpiod_edge_event* event = gpiod_edge_event_buffer_get_event(m_eventBuffer, 0);
        if (!event) {
            return false;
        }

        tsNsOut = static_cast<int64_t>(gpiod_edge_event_get_timestamp_ns(event));
        return true;
    }

private:
    gpiod_chip* m_chip{nullptr};
    gpiod_line_request* m_request{nullptr};
    gpiod_edge_event_buffer* m_eventBuffer{nullptr};
};
