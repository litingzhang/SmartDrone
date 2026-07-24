#include "adapters/simulation/gazebo_measurement_clock.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>

namespace SmartDrone::Adapters::Simulation {
namespace {

constexpr std::uint64_t CLOCK_REORDER_TOLERANCE_NS = 50000000ULL;
constexpr std::uint64_t RATE_SAMPLE_INTERVAL_NS = 50000000ULL;
constexpr std::size_t RATE_SAMPLE_CAPACITY = 5;
constexpr double MIN_MONOTONIC_NS_PER_MEASUREMENT_NS = 0.001;
constexpr double MAX_MONOTONIC_NS_PER_MEASUREMENT_NS = 1000.0;

double MedianRate(
    std::array<double, RATE_SAMPLE_CAPACITY> samples, std::size_t count)
{
    std::sort(samples.begin(), samples.begin() + count);
    const std::size_t middle = count / 2U;
    if (count % 2U != 0U) {
        return samples[middle];
    }
    return (samples[middle - 1U] + samples[middle]) / 2.0;
}

std::int64_t ApplyRate(std::int64_t referenceMonotonicNs,
                       std::uint64_t measurementDeltaNs,
                       double monotonicNsPerMeasurementNs, bool forward)
{
    const long double scaledDelta = std::round(
        static_cast<long double>(measurementDeltaNs) *
        monotonicNsPerMeasurementNs);
    const long double limit = forward
                                  ? std::numeric_limits<std::int64_t>::max() -
                                        referenceMonotonicNs
                                  : referenceMonotonicNs;
    const auto boundedDelta = static_cast<std::int64_t>(
        std::min(scaledDelta, std::max(0.0L, limit)));
    return forward ? referenceMonotonicNs + boundedDelta
                   : referenceMonotonicNs - boundedDelta;
}

} // namespace

struct GazeboMeasurementClock::Snapshot {
    std::uint64_t measurementNs{0};
    std::int64_t arrivalMonotonicNs{0};
    std::uint64_t rateReferenceMeasurementNs{0};
    std::uint64_t rateReferenceRealNs{0};
    std::int64_t rateReferenceMonotonicNs{0};
    std::array<double, RATE_SAMPLE_CAPACITY> rateSamples{};
    std::size_t rateSampleCount{0};
    std::size_t rateSampleCursor{0};
    double monotonicNsPerMeasurementNs{1.0};
    std::uint32_t resetCounter{0};
    bool valid{false};
    bool rateValid{false};
    bool rateUsesSimulationReal{false};
    bool stalled{false};
};

std::uint64_t GazeboMeasurementClock::NowNs() const
{
    const auto snapshot = std::atomic_load_explicit(
        &m_snapshot, std::memory_order_acquire);
    return snapshot ? snapshot->measurementNs : 0;
}

std::uint32_t GazeboMeasurementClock::ResetCounter() const
{
    const auto snapshot = std::atomic_load_explicit(
        &m_snapshot, std::memory_order_acquire);
    return snapshot ? snapshot->resetCounter : 0;
}

bool GazeboMeasurementClock::Valid() const
{
    const auto snapshot = std::atomic_load_explicit(
        &m_snapshot, std::memory_order_acquire);
    return snapshot && snapshot->valid;
}

bool GazeboMeasurementClock::ShouldIgnoreObservation(
    const Snapshot &snapshot, std::uint64_t measurementNs,
    std::int64_t arrivalMonotonicNs)
{
    if (!snapshot.valid) {
        return false;
    }
    if (measurementNs == snapshot.measurementNs) {
        return true;
    }
    if (measurementNs < snapshot.measurementNs) {
        return snapshot.measurementNs - measurementNs <=
               CLOCK_REORDER_TOLERANCE_NS;
    }
    return arrivalMonotonicNs <= snapshot.arrivalMonotonicNs;
}

void GazeboMeasurementClock::StartRateSegment(
    Snapshot &snapshot, std::uint64_t measurementNs,
    std::uint64_t simulationRealNs,
    std::int64_t arrivalMonotonicNs)
{
    snapshot.measurementNs = measurementNs;
    snapshot.arrivalMonotonicNs = arrivalMonotonicNs;
    snapshot.rateReferenceMeasurementNs = measurementNs;
    snapshot.rateReferenceRealNs = simulationRealNs;
    snapshot.rateReferenceMonotonicNs = arrivalMonotonicNs;
    snapshot.rateSamples = {};
    snapshot.rateSampleCount = 0;
    snapshot.rateSampleCursor = 0;
    snapshot.monotonicNsPerMeasurementNs = 1.0;
    snapshot.valid = true;
    snapshot.rateValid = false;
    snapshot.rateUsesSimulationReal = simulationRealNs > 0;
    snapshot.stalled = false;
}

void GazeboMeasurementClock::AdvanceRateSegment(
    Snapshot &snapshot, const Snapshot &baseline,
    std::uint64_t measurementNs, std::uint64_t simulationRealNs,
    std::int64_t arrivalMonotonicNs)
{
    snapshot.measurementNs = measurementNs;
    snapshot.arrivalMonotonicNs = arrivalMonotonicNs;
    snapshot.stalled = false;
    const bool usesSimulationReal = simulationRealNs > 0;
    if (usesSimulationReal != baseline.rateUsesSimulationReal ||
        (usesSimulationReal &&
         simulationRealNs <= baseline.rateReferenceRealNs)) {
        StartRateSegment(snapshot, measurementNs, simulationRealNs,
                         arrivalMonotonicNs);
        return;
    }
    const std::uint64_t measurementDelta =
        measurementNs - baseline.rateReferenceMeasurementNs;
    if (measurementDelta < RATE_SAMPLE_INTERVAL_NS) {
        return;
    }
    const std::int64_t rateDelta = usesSimulationReal
                                       ? static_cast<std::int64_t>(
                                             simulationRealNs -
                                             baseline.rateReferenceRealNs)
                                       : arrivalMonotonicNs -
                                             baseline.rateReferenceMonotonicNs;
    snapshot.rateReferenceMeasurementNs = measurementNs;
    snapshot.rateReferenceRealNs = simulationRealNs;
    snapshot.rateReferenceMonotonicNs = arrivalMonotonicNs;
    const double rate = static_cast<double>(rateDelta) /
                        static_cast<double>(measurementDelta);
    if (!std::isfinite(rate) ||
        rate < MIN_MONOTONIC_NS_PER_MEASUREMENT_NS ||
        rate > MAX_MONOTONIC_NS_PER_MEASUREMENT_NS) {
        return;
    }
    snapshot.rateSamples[snapshot.rateSampleCursor] = rate;
    snapshot.rateSampleCursor =
        (snapshot.rateSampleCursor + 1U) % RATE_SAMPLE_CAPACITY;
    snapshot.rateSampleCount = std::min(
        snapshot.rateSampleCount + 1U, RATE_SAMPLE_CAPACITY);
    snapshot.monotonicNsPerMeasurementNs =
        MedianRate(snapshot.rateSamples, snapshot.rateSampleCount);
    snapshot.rateValid = true;
}

void GazeboMeasurementClock::Observe(std::uint64_t measurementNs,
                                     std::int64_t arrivalMonotonicNs)
{
    Observe(measurementNs, 0, arrivalMonotonicNs);
}

void GazeboMeasurementClock::Observe(std::uint64_t measurementNs,
                                     std::uint64_t simulationRealNs,
                                     std::int64_t arrivalMonotonicNs)
{
    auto current = std::atomic_load_explicit(
        &m_snapshot, std::memory_order_acquire);
    while (true) {
        const Snapshot baseline = current ? *current : Snapshot{};
        if (ShouldIgnoreObservation(baseline, measurementNs,
                                    arrivalMonotonicNs)) {
            return;
        }
        auto next = std::make_shared<Snapshot>(baseline);
        const bool rewound =
            baseline.valid && measurementNs < baseline.measurementNs;
        if (rewound) {
            ++next->resetCounter;
        }
        if (!baseline.valid || baseline.stalled || rewound) {
            StartRateSegment(*next, measurementNs, simulationRealNs,
                             arrivalMonotonicNs);
        } else {
            AdvanceRateSegment(*next, baseline, measurementNs,
                               simulationRealNs,
                               arrivalMonotonicNs);
        }
        std::shared_ptr<const Snapshot> desired = std::move(next);
        if (std::atomic_compare_exchange_weak_explicit(
                &m_snapshot, &current, desired,
                std::memory_order_release, std::memory_order_acquire)) {
            return;
        }
    }
}

bool GazeboMeasurementClock::DetectStall(std::int64_t nowMonotonicNs,
                                         std::int64_t timeoutNs)
{
    auto current = std::atomic_load_explicit(
        &m_snapshot, std::memory_order_acquire);
    while (current && current->valid && !current->stalled && timeoutNs > 0 &&
           nowMonotonicNs > current->arrivalMonotonicNs &&
           nowMonotonicNs - current->arrivalMonotonicNs > timeoutNs) {
        auto next = std::make_shared<Snapshot>(*current);
        next->stalled = true;
        ++next->resetCounter;
        std::shared_ptr<const Snapshot> desired = std::move(next);
        if (std::atomic_compare_exchange_weak_explicit(
                &m_snapshot, &current, desired,
                std::memory_order_release, std::memory_order_acquire)) {
            return true;
        }
    }
    return false;
}

std::int64_t GazeboMeasurementClock::EstimateCaptureMonotonicNs(
    std::uint64_t measurementNs, std::int64_t fallbackMonotonicNs) const
{
    return ResolveMeasurementStamp(measurementNs, fallbackMonotonicNs)
        .captureMonotonicNs;
}

GazeboMeasurementStamp GazeboMeasurementClock::ResolveMeasurementStamp(
    std::uint64_t measurementNs, std::int64_t fallbackMonotonicNs) const
{
    const auto snapshot = std::atomic_load_explicit(
        &m_snapshot, std::memory_order_acquire);
    GazeboMeasurementStamp stamp{};
    stamp.measurementNs = measurementNs;
    stamp.captureMonotonicNs = fallbackMonotonicNs;
    if (!snapshot || !snapshot->valid) {
        return stamp;
    }
    stamp.clockValid = true;
    stamp.resetCounter = snapshot->resetCounter;
    const std::uint64_t referenceNs = snapshot->measurementNs;
    const std::int64_t arrivalNs = snapshot->arrivalMonotonicNs;
    stamp.measurementNs = measurementNs > 0 ? measurementNs : referenceNs;
    if (arrivalNs < 0) {
        return stamp;
    }
    const double rate = snapshot->rateValid
                            ? snapshot->monotonicNsPerMeasurementNs
                            : 1.0;
    if (stamp.measurementNs >= referenceNs) {
        const std::uint64_t delta = stamp.measurementNs - referenceNs;
        stamp.captureMonotonicNs = ApplyRate(arrivalNs, delta, rate, true);
        return stamp;
    }
    const std::uint64_t delta = referenceNs - stamp.measurementNs;
    stamp.captureMonotonicNs = ApplyRate(arrivalNs, delta, rate, false);
    return stamp;
}

std::int64_t GazeboMeasurementClock::LastArrivalMonotonicNs() const
{
    const auto snapshot = std::atomic_load_explicit(
        &m_snapshot, std::memory_order_acquire);
    return snapshot ? snapshot->arrivalMonotonicNs : 0;
}

void GazeboMeasurementClock::Reset()
{
    auto current = std::atomic_load_explicit(
        &m_snapshot, std::memory_order_acquire);
    while (true) {
        const Snapshot baseline = current ? *current : Snapshot{};
        auto next = std::make_shared<Snapshot>();
        next->resetCounter = baseline.resetCounter + 1U;
        std::shared_ptr<const Snapshot> desired = std::move(next);
        if (std::atomic_compare_exchange_weak_explicit(
                &m_snapshot, &current, desired,
                std::memory_order_release, std::memory_order_acquire)) {
            return;
        }
    }
}

} // namespace SmartDrone::Adapters::Simulation
