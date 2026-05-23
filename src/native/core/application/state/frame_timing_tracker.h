#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <unordered_map>

#include "core/ports/frame_timing.h"

namespace SmartDrone::Core::Application {

using FrameTimingRecord = SmartDrone::Core::Ports::FrameTimingRecord;

class FrameTimingTracker final : public SmartDrone::Core::Ports::IFrameTimingTracker {
  public:
    explicit FrameTimingTracker(size_t maxRecords = 4096);

    void UpsertCapture(uint64_t frameId, uint64_t tCamNs, uint64_t tCbNs);
    void MarkSlamIn(uint64_t frameId, uint64_t tSlamInNs);
    void MarkSlamOut(uint64_t frameId, uint64_t tSlamOutNs);
    void MarkMavTx(uint64_t frameId, uint64_t tMavTxNs) override;
    bool Lookup(uint64_t frameId, FrameTimingRecord &out) const override;

  private:
    FrameTimingRecord &EnsureRecordLocked(uint64_t frameId);
    void TrimLocked();

    mutable std::mutex m_mutex;
    std::unordered_map<uint64_t, FrameTimingRecord> m_records;
    std::deque<uint64_t> m_order;
    size_t m_maxRecords{4096};
};

} // namespace SmartDrone::Core::Application
