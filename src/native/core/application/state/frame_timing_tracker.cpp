#include "core/application/state/frame_timing_tracker.h"

namespace smartdrone::core::application {

void FrameTimingTracker::UpsertCapture(uint64_t frameId, uint64_t tCamNs, uint64_t tCbNs)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    FrameTimingRecord& rec = EnsureRecordLocked(frameId);
    rec.tCamNs = tCamNs;
    rec.tCbNs = tCbNs;
}

void FrameTimingTracker::MarkSlamIn(uint64_t frameId, uint64_t tSlamInNs)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    EnsureRecordLocked(frameId).tSlamInNs = tSlamInNs;
}

void FrameTimingTracker::MarkSlamOut(uint64_t frameId, uint64_t tSlamOutNs)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    EnsureRecordLocked(frameId).tSlamOutNs = tSlamOutNs;
}

void FrameTimingTracker::MarkMavTx(uint64_t frameId, uint64_t tMavTxNs)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    EnsureRecordLocked(frameId).tMavTxNs = tMavTxNs;
}

bool FrameTimingTracker::Lookup(uint64_t frameId, FrameTimingRecord& out) const
{
    std::lock_guard<std::mutex> lk(m_mutex);
    const auto it = m_records.find(frameId);
    if (it == m_records.end()) {
        return false;
    }
    out = it->second;
    return true;
}

FrameTimingRecord& FrameTimingTracker::EnsureRecordLocked(uint64_t frameId)
{
    auto [it, inserted] = m_records.try_emplace(frameId);
    FrameTimingRecord& rec = it->second;
    if (inserted) {
        rec.frameId = frameId;
        m_order.push_back(frameId);
        TrimLocked();
    }
    return rec;
}

void FrameTimingTracker::TrimLocked()
{
    while (m_order.size() > kMaxRecords) {
        const uint64_t oldest = m_order.front();
        m_order.pop_front();
        m_records.erase(oldest);
    }
}

}  // namespace smartdrone::core::application
