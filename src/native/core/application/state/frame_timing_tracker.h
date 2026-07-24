#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "core/ports/frame_timing.h"

namespace SmartDrone::Core::Application {

using FrameTimingRecord = SmartDrone::Core::Ports::FrameTimingRecord;
using FrameCaptureTiming = SmartDrone::Core::Ports::FrameCaptureTiming;

class FrameTimingTracker final : public SmartDrone::Core::Ports::IFrameTimingTracker {
  public:
    explicit FrameTimingTracker(size_t maxRecords = 4096);
    ~FrameTimingTracker() override;

    void UpsertCapture(uint64_t frameId, const FrameCaptureTiming &timing);
    void UpsertCapture(uint64_t frameId, uint64_t tCamNs,
                       uint64_t tCaptureMonotonicNs, uint64_t tCbNs);
    void MarkSlamIn(uint64_t frameId, uint64_t tSlamInNs);
    void MarkSlamOut(uint64_t frameId, uint64_t tSlamOutNs);
    void MarkMavTx(uint64_t frameId, uint64_t tMavTxNs) override;
    bool Lookup(uint64_t frameId, FrameTimingRecord &out) const override;

  private:
    struct Impl;

    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Core::Application
