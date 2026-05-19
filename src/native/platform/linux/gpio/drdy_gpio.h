#pragma once

#include <gpiod.h>

#include <cstdint>
#include <string>

class DrdyGpio {
  public:
    ~DrdyGpio();

    bool Open(const std::string &chipPath, unsigned lineOffset);
    bool WaitTs(int timeoutMs, int64_t &tsNsOut);

  private:
    bool OpenChip(const std::string &chipPath);
#if SMART_DRONE_GPIOD_V2
    bool OpenLineV2(unsigned lineOffset);
    bool CreateLineRequestV2(gpiod_line_config *lineConfig);
    int64_t ReadLatestEventTimestampV2();
#else
    bool OpenLineV1(unsigned lineOffset);
    int64_t ReadLatestEventTimestampV1();
#endif

    gpiod_chip *m_chip{nullptr};
#if SMART_DRONE_GPIOD_V2
    gpiod_line_request *m_request{nullptr};
    gpiod_edge_event_buffer *m_eventBuffer{nullptr};
#else
    gpiod_line *m_line{nullptr};
#endif
};
