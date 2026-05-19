#include "core/application/state/imu_buffer.h"

#include <algorithm>
#include <limits>
#include <mutex>

void ImuBuffer::Push(const ImuSample &sample)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_queue.push_back(sample);

    const int64_t keepNs = static_cast<int64_t>(m_keepSec) * 1000000000LL;
    while (!m_queue.empty() && (m_queue.back().tNs - m_queue.front().tNs) > keepNs) {
        m_queue.pop_front();
        if (m_lastUsedIdx > 0) {
            --m_lastUsedIdx;
        }
    }
}

std::vector<smartdrone::core::ports::ImuReading>
ImuBuffer::PopBetweenNs(const ImuTimeRange &range)
{
    std::vector<smartdrone::core::ports::ImuReading> out;
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_queue.empty() || range.endNs < range.startNs) {
        return out;
    }

    const int64_t rangeStartNs = range.startNs - range.slackBeforeNs;
    const int64_t rangeEndNs = range.endNs + range.slackAfterNs;
    const size_t searchBeginIdx = FindSearchBeginIndex(rangeStartNs);
    out.reserve(std::min(m_queue.size() - searchBeginIdx, static_cast<size_t>(256)));

    size_t startIdx = FindFirstIndexAtOrAfter(range.startNs, searchBeginIdx);
    AppendLeadingSample(range, rangeEndNs, startIdx, out);
    const size_t cursorIdx = AppendSamplesUntil(range.startNs, range.endNs, startIdx, out);
    AppendTrailingSample(range, rangeEndNs, cursorIdx, out);

    if (!out.empty()) {
        UpdateLastUsedIndex(searchBeginIdx, range.endNs);
    }
    PurgeBefore(range.startNs - m_purgeMarginNs);
    return out;
}

std::vector<smartdrone::core::ports::ImuReading>
ImuBuffer::PopBetweenNs(int64_t t0Ns, int64_t t1Ns,
                        int64_t slackBeforeNs, int64_t slackAfterNs)
{
    return PopBetweenNs(ImuTimeRange{t0Ns, t1Ns, slackBeforeNs, slackAfterNs});
}

size_t ImuBuffer::Size() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_queue.size();
}

bool ImuBuffer::PeekFirstLast(int64_t &tFirst, int64_t &tLast) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_queue.empty()) {
        return false;
    }
    tFirst = m_queue.front().tNs;
    tLast = m_queue.back().tNs;
    return true;
}

ImuSample ImuBuffer::InterpolateSample(const ImuSample &a, const ImuSample &b, int64_t targetNs)
{
    if (b.tNs <= a.tNs || targetNs <= a.tNs) {
        ImuSample out = a;
        out.tNs = targetNs;
        return out;
    }
    if (targetNs >= b.tNs) {
        ImuSample out = b;
        out.tNs = targetNs;
        return out;
    }

    const float alpha = static_cast<float>(targetNs - a.tNs) / static_cast<float>(b.tNs - a.tNs);
    ImuSample out{};
    out.tNs = targetNs;
    out.ax = a.ax + (b.ax - a.ax) * alpha;
    out.ay = a.ay + (b.ay - a.ay) * alpha;
    out.az = a.az + (b.az - a.az) * alpha;
    out.gx = a.gx + (b.gx - a.gx) * alpha;
    out.gy = a.gy + (b.gy - a.gy) * alpha;
    out.gz = a.gz + (b.gz - a.gz) * alpha;
    return out;
}

smartdrone::core::ports::ImuReading ImuBuffer::ToReading(const ImuSample &sample)
{
    smartdrone::core::ports::ImuReading reading{};
    reading.timestampNs = sample.tNs;
    reading.ax = sample.ax;
    reading.ay = sample.ay;
    reading.az = sample.az;
    reading.gx = sample.gx;
    reading.gy = sample.gy;
    reading.gz = sample.gz;
    return reading;
}

size_t ImuBuffer::FindFirstIndexAtOrAfter(int64_t timestampNs, size_t beginIdx) const
{
    size_t index = beginIdx;
    while (index < m_queue.size() && m_queue[index].tNs < timestampNs) {
        ++index;
    }
    return index;
}

size_t ImuBuffer::FindSearchBeginIndex(int64_t rangeStartNs) const
{
    const size_t rewindIdx =
        std::min(m_lastUsedIdx > 0 ? (m_lastUsedIdx - 1) : 0, m_queue.size());
    return FindFirstIndexAtOrAfter(rangeStartNs, rewindIdx);
}

void ImuBuffer::AppendLeadingSample(
    const ImuTimeRange &range, int64_t rangeEndNs, size_t &startIdx,
    std::vector<smartdrone::core::ports::ImuReading> &out) const
{
    if (startIdx < m_queue.size() && m_queue[startIdx].tNs == range.startNs) {
        out.push_back(ToReading(m_queue[startIdx]));
        ++startIdx;
        return;
    }
    if (startIdx > 0 && startIdx < m_queue.size() &&
        m_queue[startIdx - 1].tNs < range.startNs &&
        m_queue[startIdx].tNs > range.startNs &&
        m_queue[startIdx].tNs <= rangeEndNs) {
        out.push_back(ToReading(InterpolateSample(m_queue[startIdx - 1],
                                                  m_queue[startIdx], range.startNs)));
        return;
    }
    const int64_t rangeStartNs = range.startNs - range.slackBeforeNs;
    if (startIdx > 0 && m_queue[startIdx - 1].tNs >= rangeStartNs) {
        out.push_back(ToReading(m_queue[startIdx - 1]));
    }
}

size_t ImuBuffer::AppendSamplesUntil(
    int64_t startNs, int64_t endNs, size_t beginIdx,
    std::vector<smartdrone::core::ports::ImuReading> &out) const
{
    int64_t lastAppendedTsNs =
        out.empty() ? std::numeric_limits<int64_t>::min() : out.back().timestampNs;
    size_t index = beginIdx;
    while (index < m_queue.size() && m_queue[index].tNs <= endNs) {
        if (m_queue[index].tNs > startNs && m_queue[index].tNs != lastAppendedTsNs) {
            out.push_back(ToReading(m_queue[index]));
            lastAppendedTsNs = m_queue[index].tNs;
        }
        ++index;
    }
    return index;
}

void ImuBuffer::AppendTrailingSample(
    const ImuTimeRange &range, int64_t rangeEndNs, size_t cursorIdx,
    std::vector<smartdrone::core::ports::ImuReading> &out) const
{
    if (!out.empty() && out.back().timestampNs == range.endNs) {
        return;
    }
    if (cursorIdx < m_queue.size() && m_queue[cursorIdx].tNs == range.endNs) {
        out.push_back(ToReading(m_queue[cursorIdx]));
        return;
    }
    if (cursorIdx > 0 && cursorIdx < m_queue.size() &&
        m_queue[cursorIdx - 1].tNs < range.endNs &&
        m_queue[cursorIdx].tNs > range.endNs &&
        m_queue[cursorIdx].tNs <= rangeEndNs) {
        out.push_back(ToReading(InterpolateSample(m_queue[cursorIdx - 1],
                                                  m_queue[cursorIdx], range.endNs)));
        return;
    }
    if (cursorIdx < m_queue.size() && m_queue[cursorIdx].tNs <= rangeEndNs) {
        out.push_back(ToReading(m_queue[cursorIdx]));
    }
}

void ImuBuffer::UpdateLastUsedIndex(size_t searchBeginIdx, int64_t endNs)
{
    m_lastUsedIdx = FindFirstIndexAtOrAfter(endNs, searchBeginIdx);
}

void ImuBuffer::PurgeBefore(int64_t purgeBeforeNs)
{
    while (!m_queue.empty() && m_queue.front().tNs < purgeBeforeNs) {
        m_queue.pop_front();
        if (m_lastUsedIdx > 0) {
            --m_lastUsedIdx;
        }
    }
}
