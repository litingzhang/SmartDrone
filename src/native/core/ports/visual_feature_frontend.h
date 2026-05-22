#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "core/ports/visual_descriptor.h"

namespace SmartDrone::core::ports {

struct VisualFeatureFrontendStats {
    double prepareMs{0.0};
    double inputMs{0.0};
    double forwardMs{0.0};
    double totalMs{0.0};
    int rawLeftCount{0};
    int rawRightCount{0};
    int stereoLeftCount{0};
    int stereoRightCount{0};
    bool lightGlueUsed{false};
    bool descriptorFallbackUsed{false};
    uint32_t imageCount{0};
    uint32_t payloadBytes{0};
};

struct VisualFeatureDetectRequest {
    const cv::Mat *gray{nullptr};
};

struct VisualFeatureDetectResult {
    std::vector<cv::Point2f> points;
    std::string error;
};

struct VisualFeatureComputeRequest {
    const cv::Mat *gray{nullptr};
};

struct VisualFeatureComputeResult {
    VisualFeatureSet features;
    std::string error;
};

struct StereoVisualFeatureComputeRequest {
    const cv::Mat *leftGray{nullptr};
    const cv::Mat *rightGray{nullptr};
};

struct StereoVisualFeatureComputeResult {
    VisualFeatureSet leftFeatures;
    VisualFeatureSet rightFeatures;
    std::string error;
};

class IVisualFeatureFrontend {
  public:
    using Stats = VisualFeatureFrontendStats;

    virtual ~IVisualFeatureFrontend() = default;

    virtual bool Running() const = 0;
    virtual bool Detect(const VisualFeatureDetectRequest &request,
                        VisualFeatureDetectResult &result) = 0;
    virtual bool DetectAndCompute(const VisualFeatureComputeRequest &request,
                                  VisualFeatureComputeResult &result) = 0;
    virtual bool
    DetectAndComputeStereo(const StereoVisualFeatureComputeRequest &request,
                           StereoVisualFeatureComputeResult &result) = 0;
    virtual void SetLightGlueEveryNOverride(int everyN) = 0;
    virtual Stats LastStats() const = 0;
};

} // namespace SmartDrone::core::ports
