#include "core/application/state/frame_timing_tracker.h"

#include <algorithm>
#include <atomic>
#include <vector>

namespace SmartDrone::Core::Application {

namespace {

constexpr std::memory_order FRAME_TIMING_WRITE_ORDER =
    std::memory_order_release;
constexpr std::memory_order FRAME_TIMING_READ_ORDER =
    std::memory_order_acquire;
constexpr std::memory_order FRAME_TIMING_RELAXED_ORDER =
    std::memory_order_relaxed;

struct FrameTimingSlot {
    std::atomic<std::uint64_t> sequence{0};
    std::atomic<std::uint64_t> frameId{0};
    std::atomic<std::uint64_t> tCamNs{0};
    std::atomic<std::uint64_t> tCaptureMonotonicNs{0};
    std::atomic<std::uint64_t> tLeftArrivalNs{0};
    std::atomic<std::uint64_t> tRightArrivalNs{0};
    std::atomic<std::uint64_t> tPairReadyNs{0};
    std::atomic<std::uint64_t> tCbNs{0};
    std::atomic<std::uint64_t> tSlamInFrameId{0};
    std::atomic<std::uint64_t> tSlamInNs{0};
    std::atomic<std::uint64_t> tSlamOutFrameId{0};
    std::atomic<std::uint64_t> tSlamOutNs{0};
    std::atomic<std::uint64_t> tMavTxFrameId{0};
    std::atomic<std::uint64_t> tMavTxNs{0};
};

using FrameTimingTimestampField =
    std::atomic<std::uint64_t> FrameTimingSlot::*;

std::uint64_t NextWriteSequence(const FrameTimingSlot &slot)
{
    return slot.sequence.load(FRAME_TIMING_RELAXED_ORDER) + 1U;
}

bool SlotMatchesFrame(const FrameTimingSlot &slot, std::uint64_t frameId)
{
    return slot.frameId.load(FRAME_TIMING_RELAXED_ORDER) == frameId;
}

std::uint64_t StageTimestamp(
    const FrameTimingSlot &slot,
    std::uint64_t frameId,
    FrameTimingTimestampField timestampField,
    FrameTimingTimestampField frameIdField)
{
    if ((slot.*frameIdField).load(FRAME_TIMING_READ_ORDER) != frameId) {
        return 0;
    }
    return (slot.*timestampField).load(FRAME_TIMING_RELAXED_ORDER);
}

void StoreStageTimestamp(
    FrameTimingSlot &slot,
    std::uint64_t frameId,
    std::uint64_t timestampNs,
    FrameTimingTimestampField timestampField,
    FrameTimingTimestampField frameIdField)
{
    if (!SlotMatchesFrame(slot, frameId)) {
        return;
    }
    (slot.*timestampField).store(timestampNs, FRAME_TIMING_RELAXED_ORDER);
    (slot.*frameIdField).store(frameId, FRAME_TIMING_WRITE_ORDER);
}

} // namespace

struct FrameTimingTracker::Impl {
    explicit Impl(std::size_t maxRecords)
        : slots(maxRecords > 0 ? maxRecords : 1)
    {
    }

    FrameTimingSlot &SlotFor(std::uint64_t frameId)
    {
        return slots[frameId % slots.size()];
    }

    const FrameTimingSlot &SlotFor(std::uint64_t frameId) const
    {
        return slots[frameId % slots.size()];
    }

    std::vector<FrameTimingSlot> slots;
};

FrameTimingTracker::FrameTimingTracker(size_t maxRecords)
    : m_impl(std::make_unique<Impl>(maxRecords))
{
}

FrameTimingTracker::~FrameTimingTracker() = default;

void FrameTimingTracker::UpsertCapture(
    uint64_t frameId, const FrameCaptureTiming &timing)
{
    FrameTimingSlot &slot = m_impl->SlotFor(frameId);
    const std::uint64_t writeSequence = NextWriteSequence(slot);
    const std::uint64_t pairReadyNs =
        std::max(timing.tLeftArrivalNs, timing.tRightArrivalNs);
    slot.sequence.store(writeSequence, FRAME_TIMING_WRITE_ORDER);
    slot.tCamNs.store(timing.tCamNs, FRAME_TIMING_RELAXED_ORDER);
    slot.tCaptureMonotonicNs.store(timing.tCaptureMonotonicNs,
                                   FRAME_TIMING_RELAXED_ORDER);
    slot.tLeftArrivalNs.store(timing.tLeftArrivalNs,
                              FRAME_TIMING_RELAXED_ORDER);
    slot.tRightArrivalNs.store(timing.tRightArrivalNs,
                               FRAME_TIMING_RELAXED_ORDER);
    slot.tPairReadyNs.store(pairReadyNs, FRAME_TIMING_RELAXED_ORDER);
    slot.tCbNs.store(pairReadyNs, FRAME_TIMING_RELAXED_ORDER);
    slot.tSlamInFrameId.store(frameId, FRAME_TIMING_RELAXED_ORDER);
    slot.tSlamInNs.store(0, FRAME_TIMING_RELAXED_ORDER);
    slot.tSlamOutFrameId.store(frameId, FRAME_TIMING_RELAXED_ORDER);
    slot.tSlamOutNs.store(0, FRAME_TIMING_RELAXED_ORDER);
    slot.tMavTxFrameId.store(frameId, FRAME_TIMING_RELAXED_ORDER);
    slot.tMavTxNs.store(0, FRAME_TIMING_RELAXED_ORDER);
    slot.frameId.store(frameId, FRAME_TIMING_RELAXED_ORDER);
    slot.sequence.store(writeSequence + 1U, FRAME_TIMING_WRITE_ORDER);
}

void FrameTimingTracker::UpsertCapture(uint64_t frameId,
                                       uint64_t tCamNs,
                                       uint64_t tCaptureMonotonicNs,
                                       uint64_t tCbNs)
{
    UpsertCapture(frameId, FrameCaptureTiming{
                               tCamNs, tCaptureMonotonicNs, tCbNs, tCbNs});
}

void FrameTimingTracker::MarkSlamIn(uint64_t frameId,
                                    uint64_t tSlamInNs)
{
    StoreStageTimestamp(m_impl->SlotFor(frameId), frameId, tSlamInNs,
                        &FrameTimingSlot::tSlamInNs,
                        &FrameTimingSlot::tSlamInFrameId);
}

void FrameTimingTracker::MarkSlamOut(uint64_t frameId,
                                     uint64_t tSlamOutNs)
{
    StoreStageTimestamp(m_impl->SlotFor(frameId), frameId, tSlamOutNs,
                        &FrameTimingSlot::tSlamOutNs,
                        &FrameTimingSlot::tSlamOutFrameId);
}

void FrameTimingTracker::MarkMavTx(uint64_t frameId,
                                   uint64_t tMavTxNs)
{
    StoreStageTimestamp(m_impl->SlotFor(frameId), frameId, tMavTxNs,
                        &FrameTimingSlot::tMavTxNs,
                        &FrameTimingSlot::tMavTxFrameId);
}

bool FrameTimingTracker::Lookup(uint64_t frameId,
                                FrameTimingRecord &out) const
{
    const FrameTimingSlot &slot = m_impl->SlotFor(frameId);
    const std::uint64_t beforeSequence =
        slot.sequence.load(FRAME_TIMING_READ_ORDER);
    if ((beforeSequence & 1U) != 0U || beforeSequence == 0 ||
        !SlotMatchesFrame(slot, frameId)) {
        return false;
    }

    out.frameId = frameId;
    out.tCamNs = slot.tCamNs.load(FRAME_TIMING_RELAXED_ORDER);
    out.tCaptureMonotonicNs =
        slot.tCaptureMonotonicNs.load(FRAME_TIMING_RELAXED_ORDER);
    out.tLeftArrivalNs =
        slot.tLeftArrivalNs.load(FRAME_TIMING_RELAXED_ORDER);
    out.tRightArrivalNs =
        slot.tRightArrivalNs.load(FRAME_TIMING_RELAXED_ORDER);
    out.tPairReadyNs =
        slot.tPairReadyNs.load(FRAME_TIMING_RELAXED_ORDER);
    out.tCbNs = slot.tCbNs.load(FRAME_TIMING_RELAXED_ORDER);
    out.tSlamInNs = StageTimestamp(
        slot, frameId, &FrameTimingSlot::tSlamInNs,
        &FrameTimingSlot::tSlamInFrameId);
    out.tSlamOutNs = StageTimestamp(
        slot, frameId, &FrameTimingSlot::tSlamOutNs,
        &FrameTimingSlot::tSlamOutFrameId);
    out.tMavTxNs = StageTimestamp(
        slot, frameId, &FrameTimingSlot::tMavTxNs,
        &FrameTimingSlot::tMavTxFrameId);
    return slot.sequence.load(FRAME_TIMING_READ_ORDER) == beforeSequence &&
           SlotMatchesFrame(slot, frameId);
}

} // namespace SmartDrone::Core::Application
