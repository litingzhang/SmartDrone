#pragma once

#include <cstdint>
#include <memory>

#include "core/ports/measurement_clock.h"

namespace SmartDrone::Adapters::Simulation {

struct GazeboMeasurementStamp {
    std::uint64_t measurementNs{0};
    std::int64_t captureMonotonicNs{0};
    std::uint32_t resetCounter{0};
    bool clockValid{false};
};

class GazeboMeasurementClock final
    : public SmartDrone::Core::Ports::IMeasurementClock {
  public:
    std::uint64_t NowNs() const override;
    std::uint32_t ResetCounter() const override;
    bool Valid() const override;

    void Observe(std::uint64_t measurementNs,
                 std::int64_t arrivalMonotonicNs);
    void Observe(std::uint64_t measurementNs,
                 std::uint64_t simulationRealNs,
                 std::int64_t arrivalMonotonicNs);
    bool DetectStall(std::int64_t nowMonotonicNs,
                     std::int64_t timeoutNs);
    GazeboMeasurementStamp ResolveMeasurementStamp(
        std::uint64_t measurementNs,
        std::int64_t fallbackMonotonicNs) const;
    std::int64_t EstimateCaptureMonotonicNs(
        std::uint64_t measurementNs, std::int64_t fallbackMonotonicNs) const;
    std::int64_t LastArrivalMonotonicNs() const;
    void Reset();

  private:
    struct Snapshot;
    static bool ShouldIgnoreObservation(const Snapshot &snapshot,
                                        std::uint64_t measurementNs,
                                        std::int64_t arrivalMonotonicNs);
    static void StartRateSegment(Snapshot &snapshot,
                                 std::uint64_t measurementNs,
                                 std::uint64_t simulationRealNs,
                                 std::int64_t arrivalMonotonicNs);
    static void AdvanceRateSegment(Snapshot &snapshot,
                                   const Snapshot &baseline,
                                   std::uint64_t measurementNs,
                                   std::uint64_t simulationRealNs,
                                   std::int64_t arrivalMonotonicNs);
    std::shared_ptr<const Snapshot> m_snapshot;
};

} // namespace SmartDrone::Adapters::Simulation
