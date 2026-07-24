#pragma once

#include <cstdint>

namespace SmartDrone::Core::Ports {

struct FrameCaptureTiming {
    std::uint64_t tCamNs{0};
    std::uint64_t tCaptureMonotonicNs{0};
    std::uint64_t tLeftArrivalNs{0};
    std::uint64_t tRightArrivalNs{0};
};

struct FrameTimingRecord {
    std::uint64_t frameId{0};
    std::uint64_t tCamNs{0};
    std::uint64_t tCaptureMonotonicNs{0};
    std::uint64_t tLeftArrivalNs{0};
    std::uint64_t tRightArrivalNs{0};
    std::uint64_t tPairReadyNs{0};
    std::uint64_t tCbNs{0};
    std::uint64_t tSlamInNs{0};
    std::uint64_t tSlamOutNs{0};
    std::uint64_t tMavTxNs{0};
};

class IFrameTimingTracker {
  public:
    virtual ~IFrameTimingTracker() = default;

    virtual void MarkMavTx(std::uint64_t frameId,
                           std::uint64_t tMavTxNs) = 0;
    virtual bool Lookup(std::uint64_t frameId,
                        FrameTimingRecord &out) const = 0;
};

} // namespace SmartDrone::Core::Ports
