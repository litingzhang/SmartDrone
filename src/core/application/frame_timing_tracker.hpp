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
    void UpsertCapture(uint64_t frameId, uint64_t tCamNs, uint64_t tCbNs)
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        FrameTimingRecord& rec = EnsureRecordLocked(frameId);
        rec.tCamNs = tCamNs;
        rec.tCbNs = tCbNs;
    }

    void MarkSlamIn(uint64_t frameId, uint64_t tSlamInNs)
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        EnsureRecordLocked(frameId).tSlamInNs = tSlamInNs;
    }

    void MarkSlamOut(uint64_t frameId, uint64_t tSlamOutNs)
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        EnsureRecordLocked(frameId).tSlamOutNs = tSlamOutNs;
    }

    void MarkMavTx(uint64_t frameId, uint64_t tMavTxNs)
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        EnsureRecordLocked(frameId).tMavTxNs = tMavTxNs;
    }

    bool Lookup(uint64_t frameId, FrameTimingRecord& out) const
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        const auto it = m_records.find(frameId);
        if (it == m_records.end()) {
            return false;
        }
        out = it->second;
        return true;
    }

private:
    FrameTimingRecord& EnsureRecordLocked(uint64_t frameId)
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

    void TrimLocked()
    {
        while (m_order.size() > kMaxRecords) {
            const uint64_t oldest = m_order.front();
            m_order.pop_front();
            m_records.erase(oldest);
        }
    }

    static constexpr size_t kMaxRecords = 4096;

    mutable std::mutex m_mutex;
    std::unordered_map<uint64_t, FrameTimingRecord> m_records;
    std::deque<uint64_t> m_order;
};

}  // namespace smartdrone::core::application
