#pragma once

#include <cstdint>
#include <memory>

#include <opencv2/core/mat.hpp>

namespace smartdrone::core::ports {

struct ImageFrame {
    int cameraId{-1};
    uint64_t timestampNs{0};
    int64_t arriveNs{0};
    uint32_t sequence{0};
    cv::Mat gray;
    std::shared_ptr<void> owner;
};

struct StereoFrame {
    ImageFrame left;
    ImageFrame right;
};

struct CameraHealth {
    bool healthy{true};
    uint64_t droppedPairs{0};
};

struct CameraDiagnostics {
    bool healthy{true};
    bool acceptFrames{false};
    uint32_t lastRawSeqL{0};
    uint32_t lastRawSeqR{0};
    uint64_t rawCountL{0};
    uint64_t rawCountR{0};
    uint64_t droppedPairs{0};
    uint64_t droppedUnpairedL{0};
    uint64_t droppedUnpairedR{0};
    size_t pendingL{0};
    size_t pendingR{0};
    size_t pairedQueue{0};
    int64_t pairTolNs{0};
    int64_t lastPairDtMs{0};
    int64_t lastRejectDtUs{0};
    int64_t lastFrameAgeMsL{-1};
    int64_t lastFrameAgeMsR{-1};
    int64_t lastPairAgeMs{-1};
};

class ICameraProvider {
  public:
    virtual ~ICameraProvider() = default;

    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual bool GrabStereo(StereoFrame &out, int timeoutMs, bool preferLatest, uint64_t minTimestampNs = 0) = 0;
    virtual CameraHealth GetHealth() const = 0;
    virtual CameraDiagnostics GetDiagnostics() const = 0;
};

} // namespace smartdrone::core::ports
