#pragma once

#include <cstdint>
#include <memory>

#include <opencv2/core/mat.hpp>

namespace smartdrone::core::ports {

struct ImageFrame {
    int cameraId{-1};
    uint64_t timestampNs{0};
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

class ICameraProvider {
public:
    virtual ~ICameraProvider() = default;

    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual bool GrabStereo(StereoFrame& out, int timeoutMs, bool preferLatest, uint64_t minTimestampNs = 0) = 0;
    virtual CameraHealth GetHealth() const = 0;
};

}  // namespace smartdrone::core::ports
