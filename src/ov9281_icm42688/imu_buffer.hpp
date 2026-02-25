#pragma once

#include <opencv2/core/types.hpp>

#include <algorithm>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "ImuTypes.h"

struct ImuSample {
    int64_t tNs{};
    float ax{}, ay{}, az{};  // m/s^2
    float gx{}, gy{}, gz{};  // rad/s
};

struct ImuScale {
    float accelLsbPerG{2048.0f};     // default for 16g
    float gyroLsbPerDps{16.4f};      // default for 2000 dps
};

class ImuBuffer {
public:
    void Push(const ImuSample& sample)
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

    std::vector<ORB_SLAM3::IMU::Point> PopBetweenNs(
        int64_t t0Ns, int64_t t1Ns, int64_t slackBeforeNs, int64_t slackAfterNs)
    {
        std::vector<ORB_SLAM3::IMU::Point> out;
        std::lock_guard<std::mutex> lock(m_mutex);

        if (m_queue.empty() || t1Ns < t0Ns) {
            return out;
        }

        const int64_t rangeStartNs = t0Ns - slackBeforeNs;
        const int64_t rangeEndNs = t1Ns + slackAfterNs;

        size_t i = std::min(m_lastUsedIdx, m_queue.size());
        while (i < m_queue.size() && m_queue[i].tNs <= rangeStartNs) {
            ++i;
        }

        size_t j = i;
        while (j < m_queue.size() && m_queue[j].tNs <= rangeEndNs) {
            const auto& sample = m_queue[j];
            if (sample.tNs > t0Ns && sample.tNs <= t1Ns) {
                const double ts = static_cast<double>(sample.tNs) * 1e-9;
                out.emplace_back(
                    cv::Point3f(sample.ax, sample.ay, sample.az),
                    cv::Point3f(sample.gx, sample.gy, sample.gz),
                    ts);
            }
            ++j;
        }

        if (!out.empty()) {
            size_t nextUsedIdx = i;
            while (nextUsedIdx < m_queue.size() && m_queue[nextUsedIdx].tNs <= t1Ns) {
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

    size_t Size() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.size();
    }

    bool PeekFirstLast(int64_t& tFirst, int64_t& tLast) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_queue.empty()) {
            return false;
        }
        tFirst = m_queue.front().tNs;
        tLast = m_queue.back().tNs;
        return true;
    }

private:
    mutable std::mutex m_mutex;
    std::deque<ImuSample> m_queue;
    size_t m_lastUsedIdx{0};
    int m_keepSec{5};
    int64_t m_purgeMarginNs{20000000};  // 20ms
};
