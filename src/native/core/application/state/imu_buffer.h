#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "core/ports/imu_provider.h"

struct ImuSample {
    int64_t tNs{};
    float ax{}, ay{}, az{};
    float gx{}, gy{}, gz{};
};

struct ImuScale {
    float accelLsbPerG{2048.0f};
    float gyroLsbPerDps{16.4f};
};

class ImuBuffer {
  public:
    void Push(const ImuSample &sample);
    std::vector<smartdrone::core::ports::ImuReading> PopBetweenNs(int64_t t0Ns, int64_t t1Ns,
                                                                  int64_t slackBeforeNs, int64_t slackAfterNs);
    size_t Size() const;
    bool PeekFirstLast(int64_t &tFirst, int64_t &tLast) const;

  private:
    static ImuSample InterpolateSample(const ImuSample &a, const ImuSample &b, int64_t targetNs);

    mutable std::mutex m_mutex;
    std::deque<ImuSample> m_queue;
    size_t m_lastUsedIdx{0};
    int m_keepSec{5};
    int64_t m_purgeMarginNs{20000000};
};
