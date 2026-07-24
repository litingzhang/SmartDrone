#include "adapters/camera/gazebo_stereo_frame_assembler.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <utility>

#include "adapters/camera/bounded_gazebo_image_queue.h"
#include "adapters/camera/gazebo_image_processor.h"
#include "adapters/camera/gazebo_stereo_config.h"
#include "common/time_utils.h"

namespace SmartDrone::Adapters::Camera {
namespace {

constexpr std::uint64_t REWIND_THRESHOLD_NS = 1000000000ULL;

std::size_t PixelChannels(GazeboImagePixelFormat format)
{
    if (format == GazeboImagePixelFormat::Mono8) {
        return 1;
    }
    if (format == GazeboImagePixelFormat::Rgb8 ||
        format == GazeboImagePixelFormat::Bgr8) {
        return 3;
    }
    return 0;
}

std::int64_t TimestampDelta(std::uint64_t left, std::uint64_t right)
{
    return left >= right ? static_cast<std::int64_t>(left - right)
                         : -static_cast<std::int64_t>(right - left);
}

std::uint64_t PairTimestamp(const GazeboRawImage &left,
                            const GazeboRawImage &right)
{
    const std::uint64_t earlier =
        std::min(left.measurementTimestampNs, right.measurementTimestampNs);
    const std::uint64_t later =
        std::max(left.measurementTimestampNs, right.measurementTimestampNs);
    return earlier + (later - earlier) / 2ULL;
}

std::int64_t NowMonotonicNs()
{
    return static_cast<std::int64_t>(MonoTimeUs() * 1000ULL);
}

} // namespace

struct GazeboStereoFrameAssembler::Impl {
    struct RawPair {
        GazeboRawImage left;
        GazeboRawImage right;
    };

    explicit Impl(GazeboStereoAssemblerConfig openConfig)
        : config(std::move(openConfig)),
          leftQueue(config.simulation.queueDepth),
          rightQueue(config.simulation.queueDepth),
          imageProcessor(config.simulation.fault)
    {
    }

    bool Validate(const GazeboRawImage &image) const;
    bool Push(GazeboStereoEye eye, GazeboRawImage image);
    void ObserveTimestamp(GazeboStereoEye eye, GazeboRawImage &image);
    bool EnqueueLatest(GazeboStereoEye eye, const GazeboRawImage &image);
    void DrainQueues();
    void DrainQueue(GazeboStereoEye eye);
    void AcceptPending(GazeboStereoEye eye, GazeboRawImage image);
    void ResetPending(std::uint32_t resetCounter);
    void SortAndTrimPending();
    void PairPending();
    void RejectEarlierFrame(std::int64_t deltaNs);
    bool PairReady(const RawPair &pair, std::int64_t nowNs) const;
    bool SelectPair(RawPair &pair, bool preferLatest,
                    std::uint64_t minTimestampNs);
    bool ConvertPair(RawPair pair,
                     SmartDrone::Core::Ports::StereoFrame &out);
    bool Grab(SmartDrone::Core::Ports::StereoFrame &out, bool preferLatest,
              std::uint64_t minTimestampNs);
    void MaybeReloadFaultState(std::int64_t nowNs);
    void PublishQueueSizes();
    SmartDrone::Core::Ports::CameraDiagnostics Diagnostics() const;

    GazeboStereoAssemblerConfig config;
    BoundedGazeboImageQueue leftQueue;
    BoundedGazeboImageQueue rightQueue;
    GazeboImageProcessor imageProcessor;
    std::deque<GazeboRawImage> pendingLeft;
    std::deque<GazeboRawImage> pendingRight;
    std::deque<RawPair> paired;
    std::atomic<std::uint32_t> activeResetCounter{0};
    bool resetCounterInitialized{false};
    std::atomic<bool> acceptFrames{false};
    std::atomic<bool> healthy{true};
    std::atomic<std::uint64_t> lastMeasurementLNs{0};
    std::atomic<std::uint64_t> lastMeasurementRNs{0};
    std::atomic<std::uint32_t> observedResetL{0};
    std::atomic<std::uint32_t> observedResetR{0};
    std::atomic<std::uint32_t> effectiveResetCounter{0};
    std::atomic<std::uint32_t> lastRawSeqL{0};
    std::atomic<std::uint32_t> lastRawSeqR{0};
    std::atomic<std::uint64_t> rawCountL{0};
    std::atomic<std::uint64_t> rawCountR{0};
    std::atomic<std::uint64_t> droppedPairs{0};
    std::atomic<std::uint64_t> droppedUnpairedL{0};
    std::atomic<std::uint64_t> droppedUnpairedR{0};
    std::atomic<std::uint64_t> queueOverflowL{0};
    std::atomic<std::uint64_t> queueOverflowR{0};
    std::atomic<std::uint64_t> timestampRewinds{0};
    std::atomic<std::size_t> pendingCountL{0};
    std::atomic<std::size_t> pendingCountR{0};
    std::atomic<std::size_t> pairedCount{0};
    std::atomic<std::int64_t> lastPairDeltaNs{0};
    std::atomic<std::int64_t> lastRejectDeltaNs{0};
    std::atomic<std::int64_t> lastArrivalL{0};
    std::atomic<std::int64_t> lastArrivalR{0};
    std::atomic<std::int64_t> lastPairArrival{0};
    std::int64_t lastFaultStateCheckNs{0};
};

bool GazeboStereoFrameAssembler::Impl::Validate(
    const GazeboRawImage &image) const
{
    const std::size_t channels = PixelChannels(image.pixelFormat);
    if (channels == 0 || image.width != config.width ||
        image.height != config.height || image.width <= 0 ||
        image.height <= 0 || image.measurementTimestampNs == 0) {
        return false;
    }
    const std::size_t minimumStep =
        static_cast<std::size_t>(image.width) * channels;
    if (!image.payload || image.step < minimumStep) {
        return false;
    }
    const std::size_t height = static_cast<std::size_t>(image.height);
    return image.step <= image.payload->size() / height;
}

void GazeboStereoFrameAssembler::Impl::ObserveTimestamp(
    GazeboStereoEye eye, GazeboRawImage &image)
{
    auto &observedReset = eye == GazeboStereoEye::Left ? observedResetL
                                                        : observedResetR;
    std::uint32_t eyeReset = observedReset.load(std::memory_order_acquire);
    const bool sharedClockAdvanced = image.clockResetCounter > eyeReset;
    std::uint32_t current = effectiveResetCounter.load(std::memory_order_acquire);
    while (image.clockResetCounter > current &&
           !effectiveResetCounter.compare_exchange_weak(
               current, image.clockResetCounter, std::memory_order_acq_rel)) {
    }
    auto &lastMeasurement = eye == GazeboStereoEye::Left
                                ? lastMeasurementLNs
                                : lastMeasurementRNs;
    const std::uint64_t previous = lastMeasurement.exchange(
        image.measurementTimestampNs, std::memory_order_acq_rel);
    if (!sharedClockAdvanced &&
        previous > image.measurementTimestampNs + REWIND_THRESHOLD_NS) {
        current = effectiveResetCounter.load(std::memory_order_acquire);
        if (current <= eyeReset && effectiveResetCounter.compare_exchange_strong(
                                       current, current + 1,
                                       std::memory_order_acq_rel)) {
            current += 1;
            timestampRewinds.fetch_add(1, std::memory_order_relaxed);
        }
        eyeReset = current;
    } else if (sharedClockAdvanced) {
        eyeReset = image.clockResetCounter;
    }
    eyeReset = std::max(
        eyeReset, effectiveResetCounter.load(std::memory_order_acquire));
    observedReset.store(eyeReset, std::memory_order_release);
    image.clockResetCounter = eyeReset;
}

bool GazeboStereoFrameAssembler::Impl::EnqueueLatest(
    GazeboStereoEye eye, const GazeboRawImage &image)
{
    auto &queue = eye == GazeboStereoEye::Left ? leftQueue : rightQueue;
    if (queue.TryEnqueue(image)) {
        return true;
    }
    GazeboRawImage discarded;
    queue.TryDequeue(discarded);
    auto &overflow = eye == GazeboStereoEye::Left ? queueOverflowL
                                                   : queueOverflowR;
    overflow.fetch_add(1, std::memory_order_relaxed);
    droppedPairs.fetch_add(1, std::memory_order_relaxed);
    return queue.TryEnqueue(image);
}

bool GazeboStereoFrameAssembler::Impl::Push(GazeboStereoEye eye,
                                             GazeboRawImage image)
{
    if (!acceptFrames.load(std::memory_order_acquire)) {
        return false;
    }
    if (!Validate(image)) {
        healthy.store(false, std::memory_order_release);
        droppedPairs.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    ObserveTimestamp(eye, image);
    auto &sequence = eye == GazeboStereoEye::Left ? lastRawSeqL : lastRawSeqR;
    auto &count = eye == GazeboStereoEye::Left ? rawCountL : rawCountR;
    auto &arrival = eye == GazeboStereoEye::Left ? lastArrivalL : lastArrivalR;
    sequence.store(image.sequence, std::memory_order_release);
    count.fetch_add(1, std::memory_order_relaxed);
    arrival.store(image.arrivalMonotonicNs, std::memory_order_release);
    healthy.store(true, std::memory_order_release);
    return EnqueueLatest(eye, image);
}

void GazeboStereoFrameAssembler::Impl::ResetPending(
    std::uint32_t resetCounter)
{
    droppedPairs.fetch_add(paired.size(), std::memory_order_relaxed);
    droppedUnpairedL.fetch_add(pendingLeft.size(), std::memory_order_relaxed);
    droppedUnpairedR.fetch_add(pendingRight.size(), std::memory_order_relaxed);
    pendingLeft.clear();
    pendingRight.clear();
    paired.clear();
    activeResetCounter.store(resetCounter, std::memory_order_release);
    resetCounterInitialized = true;
}

void GazeboStereoFrameAssembler::Impl::AcceptPending(
    GazeboStereoEye eye, GazeboRawImage image)
{
    if (!resetCounterInitialized) {
        activeResetCounter.store(image.clockResetCounter,
                                 std::memory_order_release);
        resetCounterInitialized = true;
    }
    if (image.clockResetCounter >
        activeResetCounter.load(std::memory_order_acquire)) {
        ResetPending(image.clockResetCounter);
    }
    if (image.clockResetCounter <
        activeResetCounter.load(std::memory_order_acquire)) {
        auto &dropped = eye == GazeboStereoEye::Left ? droppedUnpairedL
                                                      : droppedUnpairedR;
        dropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    auto &pending = eye == GazeboStereoEye::Left ? pendingLeft : pendingRight;
    pending.push_back(std::move(image));
}

void GazeboStereoFrameAssembler::Impl::DrainQueue(GazeboStereoEye eye)
{
    auto &queue = eye == GazeboStereoEye::Left ? leftQueue : rightQueue;
    GazeboRawImage image;
    while (queue.TryDequeue(image)) {
        AcceptPending(eye, std::move(image));
    }
}

void GazeboStereoFrameAssembler::Impl::DrainQueues()
{
    DrainQueue(GazeboStereoEye::Left);
    DrainQueue(GazeboStereoEye::Right);
    SortAndTrimPending();
}

void GazeboStereoFrameAssembler::Impl::SortAndTrimPending()
{
    const auto compare = [](const GazeboRawImage &left,
                            const GazeboRawImage &right) {
        return left.measurementTimestampNs < right.measurementTimestampNs;
    };
    std::stable_sort(pendingLeft.begin(), pendingLeft.end(), compare);
    std::stable_sort(pendingRight.begin(), pendingRight.end(), compare);
    while (pendingLeft.size() > config.simulation.queueDepth) {
        pendingLeft.pop_front();
        droppedUnpairedL.fetch_add(1, std::memory_order_relaxed);
    }
    while (pendingRight.size() > config.simulation.queueDepth) {
        pendingRight.pop_front();
        droppedUnpairedR.fetch_add(1, std::memory_order_relaxed);
    }
}

void GazeboStereoFrameAssembler::Impl::RejectEarlierFrame(
    std::int64_t deltaNs)
{
    lastRejectDeltaNs.store(std::llabs(deltaNs), std::memory_order_release);
    if (deltaNs < 0) {
        pendingLeft.pop_front();
        droppedUnpairedL.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    pendingRight.pop_front();
    droppedUnpairedR.fetch_add(1, std::memory_order_relaxed);
}

void GazeboStereoFrameAssembler::Impl::PairPending()
{
    while (!pendingLeft.empty() && !pendingRight.empty()) {
        const std::int64_t delta = TimestampDelta(
            pendingLeft.front().measurementTimestampNs,
            pendingRight.front().measurementTimestampNs);
        if (std::llabs(delta) > config.simulation.pairToleranceNs) {
            RejectEarlierFrame(delta);
            continue;
        }
        lastPairDeltaNs.store(delta, std::memory_order_release);
        paired.push_back({std::move(pendingLeft.front()),
                          std::move(pendingRight.front())});
        pendingLeft.pop_front();
        pendingRight.pop_front();
        if (paired.size() > config.simulation.queueDepth) {
            paired.pop_front();
            droppedPairs.fetch_add(1, std::memory_order_relaxed);
        }
    }
    PublishQueueSizes();
}

bool GazeboStereoFrameAssembler::Impl::PairReady(
    const RawPair &pair, std::int64_t nowNs) const
{
    const std::int64_t delayNs =
        static_cast<std::int64_t>(config.simulation.fault.delayMs) * 1000000LL;
    const std::int64_t arrival = std::max(pair.left.arrivalMonotonicNs,
                                          pair.right.arrivalMonotonicNs);
    return arrival + delayNs <= nowNs;
}

bool GazeboStereoFrameAssembler::Impl::SelectPair(
    RawPair &pair, bool preferLatest, std::uint64_t minTimestampNs)
{
    while (!paired.empty() &&
           PairTimestamp(paired.front().left, paired.front().right) <
               minTimestampNs) {
        paired.pop_front();
        droppedPairs.fetch_add(1, std::memory_order_relaxed);
    }
    const std::int64_t nowNs = NowMonotonicNs();
    auto selected = paired.end();
    for (auto it = paired.begin(); it != paired.end(); ++it) {
        if (PairTimestamp(it->left, it->right) < minTimestampNs ||
            !PairReady(*it, nowNs)) {
            continue;
        }
        selected = it;
        if (!preferLatest) {
            break;
        }
    }
    if (selected == paired.end()) {
        return false;
    }
    const std::size_t discarded =
        static_cast<std::size_t>(std::distance(paired.begin(), selected));
    pair = std::move(*selected);
    paired.erase(paired.begin(), std::next(selected));
    droppedPairs.fetch_add(discarded, std::memory_order_relaxed);
    PublishQueueSizes();
    return true;
}

bool GazeboStereoFrameAssembler::Impl::ConvertPair(
    RawPair pair, SmartDrone::Core::Ports::StereoFrame &out)
{
    if (imageProcessor.ShouldDrop(PairTimestamp(pair.left, pair.right))) {
        droppedPairs.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    SmartDrone::Core::Ports::StereoFrame converted;
    if (!imageProcessor.Convert(pair.left, GazeboStereoEye::Left,
                                converted.left) ||
        !imageProcessor.Convert(pair.right, GazeboStereoEye::Right,
                                converted.right)) {
        droppedPairs.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    lastPairArrival.store(
        std::max(pair.left.arrivalMonotonicNs,
                 pair.right.arrivalMonotonicNs),
        std::memory_order_release);
    out = std::move(converted);
    return true;
}

bool GazeboStereoFrameAssembler::Impl::Grab(
    SmartDrone::Core::Ports::StereoFrame &out, bool preferLatest,
    std::uint64_t minTimestampNs)
{
    MaybeReloadFaultState(NowMonotonicNs());
    DrainQueues();
    PairPending();
    RawPair pair;
    while (SelectPair(pair, preferLatest, minTimestampNs)) {
        if (ConvertPair(std::move(pair), out)) {
            return true;
        }
    }
    return false;
}

void GazeboStereoFrameAssembler::Impl::MaybeReloadFaultState(
    std::int64_t nowNs)
{
    constexpr std::int64_t faultStatePollNs = 100000000LL;
    if (config.simulation.faultStatePath.empty() ||
        nowNs - lastFaultStateCheckNs < faultStatePollNs) {
        return;
    }
    lastFaultStateCheckNs = nowNs;
    GazeboImageFaultConfig updated;
    if (!TryLoadGazeboImageFaultState(config.simulation.faultStatePath,
                                      imageProcessor.Generation(), updated)) {
        return;
    }
    config.simulation.fault = updated;
    imageProcessor.UpdateConfig(std::move(updated));
}

void GazeboStereoFrameAssembler::Impl::PublishQueueSizes()
{
    pendingCountL.store(pendingLeft.size(), std::memory_order_release);
    pendingCountR.store(pendingRight.size(), std::memory_order_release);
    pairedCount.store(paired.size(), std::memory_order_release);
}

SmartDrone::Core::Ports::CameraDiagnostics
GazeboStereoFrameAssembler::Impl::Diagnostics() const
{
    SmartDrone::Core::Ports::CameraDiagnostics out;
    out.healthy = healthy.load(std::memory_order_acquire);
    out.acceptFrames = acceptFrames.load(std::memory_order_acquire);
    out.lastRawSeqL = lastRawSeqL.load(std::memory_order_acquire);
    out.lastRawSeqR = lastRawSeqR.load(std::memory_order_acquire);
    out.rawCountL = rawCountL.load(std::memory_order_acquire);
    out.rawCountR = rawCountR.load(std::memory_order_acquire);
    out.droppedPairs = droppedPairs.load(std::memory_order_acquire);
    out.droppedUnpairedL = droppedUnpairedL.load(std::memory_order_acquire);
    out.droppedUnpairedR = droppedUnpairedR.load(std::memory_order_acquire);
    out.queueOverflowL = queueOverflowL.load(std::memory_order_acquire);
    out.queueOverflowR = queueOverflowR.load(std::memory_order_acquire);
    out.timestampRewinds = timestampRewinds.load(std::memory_order_acquire);
    out.clockResetCounter =
        activeResetCounter.load(std::memory_order_acquire);
    out.pendingL = pendingCountL.load(std::memory_order_acquire);
    out.pendingR = pendingCountR.load(std::memory_order_acquire);
    out.pairedQueue = pairedCount.load(std::memory_order_acquire);
    out.pairTolNs = config.simulation.pairToleranceNs;
    out.lastPairDtMs = lastPairDeltaNs.load(std::memory_order_acquire) / 1000000LL;
    out.lastRejectDtUs = lastRejectDeltaNs.load(std::memory_order_acquire) / 1000LL;
    const std::int64_t nowNs = NowMonotonicNs();
    const auto ageMs = [nowNs](std::int64_t timestampNs) {
        return timestampNs > 0 ? std::max<std::int64_t>(0, nowNs - timestampNs) /
                                     1000000LL
                               : -1LL;
    };
    out.lastFrameAgeMsL = ageMs(lastArrivalL.load(std::memory_order_acquire));
    out.lastFrameAgeMsR = ageMs(lastArrivalR.load(std::memory_order_acquire));
    out.lastPairAgeMs = ageMs(lastPairArrival.load(std::memory_order_acquire));
    return out;
}

GazeboStereoFrameAssembler::GazeboStereoFrameAssembler(
    GazeboStereoAssemblerConfig config)
    : m_impl(std::make_unique<Impl>(std::move(config)))
{
}

GazeboStereoFrameAssembler::~GazeboStereoFrameAssembler() = default;

void GazeboStereoFrameAssembler::SetAcceptFrames(bool acceptFrames)
{
    m_impl->acceptFrames.store(acceptFrames, std::memory_order_release);
}

bool GazeboStereoFrameAssembler::PushImage(GazeboStereoEye eye,
                                            GazeboRawImage image)
{
    return m_impl->Push(eye, std::move(image));
}

bool GazeboStereoFrameAssembler::GrabStereo(
    SmartDrone::Core::Ports::StereoFrame &out, bool preferLatest,
    std::uint64_t minTimestampNs)
{
    return m_impl->Grab(out, preferLatest, minTimestampNs);
}

SmartDrone::Core::Ports::CameraHealth
GazeboStereoFrameAssembler::GetHealth() const
{
    return {m_impl->healthy.load(std::memory_order_acquire),
            m_impl->droppedPairs.load(std::memory_order_acquire)};
}

SmartDrone::Core::Ports::CameraDiagnostics
GazeboStereoFrameAssembler::GetDiagnostics() const
{
    return m_impl->Diagnostics();
}

void GazeboStereoFrameAssembler::Clear()
{
    GazeboRawImage discarded;
    while (m_impl->leftQueue.TryDequeue(discarded)) {
    }
    while (m_impl->rightQueue.TryDequeue(discarded)) {
    }
    m_impl->ResetPending(
        m_impl->activeResetCounter.load(std::memory_order_acquire));
    m_impl->PublishQueueSizes();
}

} // namespace SmartDrone::Adapters::Camera
