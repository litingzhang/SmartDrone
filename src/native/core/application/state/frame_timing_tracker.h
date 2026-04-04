#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <unordered_map>

namespace smartdrone::core::application {

struct FrameTimingRecord {
    uint64_t frameId{0};
    uint64_t tCamNs{0};
    uint64_t tCbNs{0};
    uint64_t tSlamInNs{0};
    uint64_t tSlamOutNs{0};
    uint64_t tMavTxNs{0};
};

class FrameTimingTracker {
  public:
    void UpsertCapture(uint64_t frameId, uint64_t tCamNs, uint64_t tCbNs);
    void MarkSlamIn(uint64_t frameId, uint64_t tSlamInNs);
    void MarkSlamOut(uint64_t frameId, uint64_t tSlamOutNs);
    void MarkMavTx(uint64_t frameId, uint64_t tMavTxNs);
    bool Lookup(uint64_t frameId, FrameTimingRecord &out) const;

  private:
    FrameTimingRecord &EnsureRecordLocked(uint64_t frameId);
    void TrimLocked();

    static constexpr size_t kMaxRecords = 4096;

    mutable std::mutex m_mutex;
    std::unordered_map<uint64_t, FrameTimingRecord> m_records;
    std::deque<uint64_t> m_order;
};

} // namespace smartdrone::core::application
