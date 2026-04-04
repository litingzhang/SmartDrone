#pragma once

#include <gpiod.h>

#include <cstdint>
#include <string>

class DrdyGpio {
public:
    ~DrdyGpio();

    bool Open(const std::string& chipPath, unsigned lineOffset);
    bool WaitTs(int timeoutMs, int64_t& tsNsOut);

private:
    gpiod_chip* m_chip{nullptr};
    gpiod_line_request* m_request{nullptr};
    gpiod_edge_event_buffer* m_eventBuffer{nullptr};
};
