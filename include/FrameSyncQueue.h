#pragma once

#include <cstdint>
#include <deque>
#include <mutex>

#include "FrameTypes.h"

class FrameSyncQueue {
  public:
    FrameSyncQueue();

    void PushFrame(const FrameItem &item);
    bool PopSyncedPair(FrameItem &left, FrameItem &right, uint64_t maxDtNs);

  private:
    void TrimQueue(std::deque<FrameItem> &q);

  private:
    std::mutex m_mutex;
    std::deque<FrameItem> m_leftQ;
    std::deque<FrameItem> m_rightQ;
    size_t m_maxQueue;
};
