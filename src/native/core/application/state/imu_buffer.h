#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "core/domain/imu_sample.h"
#include "core/ports/imu_provider.h"

class ImuBuffer final : public SmartDrone::Core::Ports::IImuWindowSource {
  public:
    ImuBuffer();
    ~ImuBuffer() override;

    void Push(const ImuSample &sample);
    std::vector<SmartDrone::Core::Ports::ImuReading> PopBetweenNs(
        const ImuTimeRange &range) override;
    std::vector<SmartDrone::Core::Ports::ImuReading> PopBetweenNs(
        int64_t t0Ns, int64_t t1Ns, int64_t slackBeforeNs,
        int64_t slackAfterNs);
    size_t Size() const override;
    bool PeekFirstLast(int64_t &tFirst, int64_t &tLast) const;

  private:
    static ImuSample InterpolateSample(const ImuSample &a, const ImuSample &b, int64_t targetNs);
    static SmartDrone::Core::Ports::ImuReading ToReading(const ImuSample &sample);
    static size_t FindFirstIndexAtOrAfter(const std::vector<ImuSample> &samples,
                                          int64_t timestampNs,
                                          size_t beginIdx);
    static void AppendLeadingSample(const std::vector<ImuSample> &samples,
                                    const ImuTimeRange &range,
                                    int64_t rangeEndNs, size_t &startIdx,
                                    std::vector<SmartDrone::Core::Ports::ImuReading> &out);
    static size_t AppendSamplesUntil(const std::vector<ImuSample> &samples,
                                     int64_t startNs, int64_t endNs,
                                     size_t beginIdx,
                                     std::vector<SmartDrone::Core::Ports::ImuReading> &out);
    static void AppendTrailingSample(const std::vector<ImuSample> &samples,
                                     const ImuTimeRange &range,
                                     int64_t rangeEndNs, size_t cursorIdx,
                                     std::vector<SmartDrone::Core::Ports::ImuReading> &out);

    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
