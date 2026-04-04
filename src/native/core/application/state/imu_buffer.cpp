#include "core/application/state/imu_buffer.h"

#include <opencv2/core/types.hpp>

#include <algorithm>
#include <limits>
#include <mutex>

void ImuBuffer::Push(const ImuSample& sample)
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

std::vector<ORB_SLAM3::IMU::Point> ImuBuffer::PopBetweenNs(
    int64_t t0Ns,
    int64_t t1Ns,
    int64_t slackBeforeNs,
    int64_t slackAfterNs)
{
    std::vector<ORB_SLAM3::IMU::Point> out;
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_queue.empty() || t1Ns < t0Ns) {
        return out;
    }

    const int64_t rangeStartNs = t0Ns - slackBeforeNs;
    const int64_t rangeEndNs = t1Ns + slackAfterNs;

    size_t i = std::min(m_lastUsedIdx > 0 ? (m_lastUsedIdx - 1) : 0, m_queue.size());
    while (i < m_queue.size() && m_queue[i].tNs < rangeStartNs) {
        ++i;
    }

    auto appendSample = [&out](const ImuSample& sample) {
        const double ts = static_cast<double>(sample.tNs) * 1e-9;
        out.emplace_back(
            cv::Point3f(sample.ax, sample.ay, sample.az),
            cv::Point3f(sample.gx, sample.gy, sample.gz),
            ts);
    };

    auto appendInterpolatedSample = [&appendSample](const ImuSample& a, const ImuSample& b, int64_t targetNs) {
        appendSample(InterpolateSample(a, b, targetNs));
    };

    out.reserve(std::min(m_queue.size() - i, static_cast<size_t>(256)));

    size_t startIdx = i;
    while (startIdx < m_queue.size() && m_queue[startIdx].tNs < t0Ns) {
        ++startIdx;
    }

    int64_t lastAppendedTsNs = std::numeric_limits<int64_t>::min();

    auto appendUnique = [&](const ImuSample& sample) {
        if (sample.tNs != lastAppendedTsNs) {
            appendSample(sample);
            lastAppendedTsNs = sample.tNs;
        }
    };

    if (startIdx < m_queue.size() && m_queue[startIdx].tNs == t0Ns) {
        appendUnique(m_queue[startIdx]);
        ++startIdx;
    } else if (startIdx > 0 && startIdx < m_queue.size() &&
               m_queue[startIdx - 1].tNs < t0Ns && m_queue[startIdx].tNs > t0Ns &&
               m_queue[startIdx].tNs <= rangeEndNs) {
        appendInterpolatedSample(m_queue[startIdx - 1], m_queue[startIdx], t0Ns);
        lastAppendedTsNs = t0Ns;
    } else if (startIdx > 0 && m_queue[startIdx - 1].tNs >= rangeStartNs) {
        appendUnique(m_queue[startIdx - 1]);
    }

    size_t j = startIdx;
    while (j < m_queue.size() && m_queue[j].tNs <= t1Ns) {
        if (m_queue[j].tNs > t0Ns) {
            appendUnique(m_queue[j]);
        }
        ++j;
    }

    const bool haveExactTrailing = !out.empty() && lastAppendedTsNs == t1Ns;
    if (!haveExactTrailing) {
        if (j < m_queue.size() && m_queue[j].tNs == t1Ns) {
            appendUnique(m_queue[j]);
        } else if (j > 0 && j < m_queue.size() &&
                   m_queue[j - 1].tNs < t1Ns && m_queue[j].tNs > t1Ns &&
                   m_queue[j].tNs <= rangeEndNs) {
            appendInterpolatedSample(m_queue[j - 1], m_queue[j], t1Ns);
            lastAppendedTsNs = t1Ns;
        } else if (j < m_queue.size() && m_queue[j].tNs <= rangeEndNs) {
            appendUnique(m_queue[j]);
        }
    }

    if (!out.empty()) {
        size_t nextUsedIdx = i;
        while (nextUsedIdx < m_queue.size() && m_queue[nextUsedIdx].tNs < t1Ns) {
            ++nextUsedIdx;
        }
        m_lastUsedIdx = nextUsedIdx;
    }

    const int64_t purgeBeforeNs = t0Ns - m_purgeMarginNs;
    while (!m_queue.empty() && m_queue.front().tNs < purgeBeforeNs) {
        m_queue.pop_front();
        if (m_lastUsedIdx > 0) {
            --m_lastUsedIdx;
        }
    }

    return out;
}

size_t ImuBuffer::Size() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_queue.size();
}

bool ImuBuffer::PeekFirstLast(int64_t& tFirst, int64_t& tLast) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_queue.empty()) {
        return false;
    }
    tFirst = m_queue.front().tNs;
    tLast = m_queue.back().tNs;
    return true;
}

ImuSample ImuBuffer::InterpolateSample(const ImuSample& a, const ImuSample& b, int64_t targetNs)
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

    const float alpha = static_cast<float>(targetNs - a.tNs) /
                        static_cast<float>(b.tNs - a.tNs);
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
