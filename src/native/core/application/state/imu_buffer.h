#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "core/domain/imu_sample.h"
#include "core/ports/imu_provider.h"

class ImuBuffer final : public SmartDrone::Core::Ports::IImuWindowSource {
  public:
    void Push(const ImuSample &sample);
    std::vector<SmartDrone::Core::Ports::ImuReading> PopBetweenNs(
        const ImuTimeRange &range) override;
    std::vector<SmartDrone::Core::Ports::ImuReading> PopBetweenNs(int64_t t0Ns, int64_t t1Ns,
                                                                  int64_t slackBeforeNs, int64_t slackAfterNs);
    size_t Size() const override;
    bool PeekFirstLast(int64_t &tFirst, int64_t &tLast) const;

  private:
    static ImuSample InterpolateSample(const ImuSample &a, const ImuSample &b, int64_t targetNs);
    static SmartDrone::Core::Ports::ImuReading ToReading(const ImuSample &sample);
    size_t FindFirstIndexAtOrAfter(int64_t timestampNs, size_t beginIdx) const;
    size_t FindSearchBeginIndex(int64_t rangeStartNs) const;
    void AppendLeadingSample(const ImuTimeRange &range, int64_t rangeEndNs,
                             size_t &startIdx,
                             std::vector<SmartDrone::Core::Ports::ImuReading> &out) const;
    size_t AppendSamplesUntil(int64_t startNs, int64_t endNs, size_t beginIdx,
                              std::vector<SmartDrone::Core::Ports::ImuReading> &out) const;
    void AppendTrailingSample(const ImuTimeRange &range, int64_t rangeEndNs,
                              size_t cursorIdx,
                              std::vector<SmartDrone::Core::Ports::ImuReading> &out) const;
    void UpdateLastUsedIndex(size_t searchBeginIdx, int64_t endNs);
    void PurgeBefore(int64_t purgeBeforeNs);

    mutable std::mutex m_mutex;
    std::deque<ImuSample> m_queue;
    size_t m_lastUsedIdx{0};
    int m_keepSec{5};
    int64_t m_purgeMarginNs{20000000};
};
