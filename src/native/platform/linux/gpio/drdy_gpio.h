#pragma once

#include <gpiod.h>

#include <cstdint>
#include <string>

class DrdyGpio {
  public:
    ~DrdyGpio();

    bool Open(const std::string &chipPath, unsigned lineOffset);
    bool ReadReadyTimestamp(int64_t &tsNsOut);

  private:
    bool OpenChip(const std::string &chipPath);
    bool OpenLine(unsigned lineOffset);
    bool EdgeEventReady();
    void CloseGpiodResources();
    int64_t ReadLatestEventTimestamp();

    gpiod_chip *m_chip{nullptr};
    void *m_request{nullptr};
    void *m_eventBuffer{nullptr};
    void *m_line{nullptr};
};
