#pragma once

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "core/ports/visual_feature_data.h"

namespace SmartDrone::core::ports {

struct VisualFrameObservationLoadOptions {
    float baselineFx{0.0f};
    float closeDepthThreshold{0.0f};
    float injectedStereoDepthScale{1.0f};
    int injectedKeypointOctave{0};
    int maxScaleLevel{0};
};

struct VisualFrameObservationData {
    std::vector<cv::KeyPoint> leftKeypoints;
    std::vector<cv::KeyPoint> rightKeypoints;
    cv::Mat leftDescriptors;
    cv::Mat rightDescriptors;
    std::vector<float> rightU;
    std::vector<float> depth;
    int featureCount{0};
    int closePointCount{0};
};

struct MonoFrameObservationLoadRequest {
    const VisualKeypointFeatureSet *features{nullptr};
};

struct StereoFrameObservationLoadRequest {
    const StereoFeatureObservationPacket *features{nullptr};
    VisualFrameObservationLoadOptions options;
};

class IVisualFrameObservationLoader {
  public:
    virtual ~IVisualFrameObservationLoader() = default;

    virtual bool LoadMonoObservation(
        const MonoFrameObservationLoadRequest &request,
        VisualFrameObservationData &outData) const = 0;
    virtual bool LoadStereoObservation(
        const StereoFrameObservationLoadRequest &request,
        VisualFrameObservationData &outData) const = 0;
};

} // namespace SmartDrone::core::ports
