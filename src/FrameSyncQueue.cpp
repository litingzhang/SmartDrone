#include "FrameSyncQueue.h"

#include <limits>

FrameSyncQueue::FrameSyncQueue() : m_mutex(), m_leftQ(), m_rightQ(), m_maxQueue(30) {}

void FrameSyncQueue::TrimQueue(std::deque<FrameItem> &q)
{
    while (q.size() > m_maxQueue)
        q.pop_front();
}

void FrameSyncQueue::PushFrame(const FrameItem &item)
{
    std::lock_guard<std::mutex> lk(m_mutex);

    if (item.m_camIndex == 0) {
        m_leftQ.push_back(item);
        TrimQueue(m_leftQ);
    } else {
        m_rightQ.push_back(item);
        TrimQueue(m_rightQ);
    }
}

bool FrameSyncQueue::PopSyncedPair(FrameItem &left, FrameItem &right, uint64_t maxDtNs)
{
    std::lock_guard<std::mutex> lk(m_mutex);

    if (m_leftQ.empty() || m_rightQ.empty())
        return false;

    for (auto itL = m_leftQ.begin(); itL != m_leftQ.end(); ++itL) {
        uint64_t tL = itL->m_tsNs;

        auto bestItR = m_rightQ.end();
        uint64_t bestAbs = std::numeric_limits<uint64_t>::max();

        for (auto itR = m_rightQ.begin(); itR != m_rightQ.end(); ++itR) {
            uint64_t tR = itR->m_tsNs;
            uint64_t absd = (tR > tL) ? (tR - tL) : (tL - tR);

            if (absd < bestAbs) {
                bestAbs = absd;
                bestItR = itR;
            }
        }

        if (bestItR != m_rightQ.end() && bestAbs <= maxDtNs) {
            left = *itL;
            right = *bestItR;

            m_leftQ.erase(m_leftQ.begin(), std::next(itL));
            m_rightQ.erase(m_rightQ.begin(), std::next(bestItR));
            return true;
        }
    }

    if (m_leftQ.front().m_tsNs < m_rightQ.front().m_tsNs)
        m_leftQ.pop_front();
    else
        m_rightQ.pop_front();

    return false;
}
