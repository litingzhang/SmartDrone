#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/external_feature_types.h"

namespace smartdrone::adapters::slam {

struct ExternalFeatureFrontendStats {
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

class ExternalFeatureFrontendClient {
  public:
    using Stats = ExternalFeatureFrontendStats;

    virtual ~ExternalFeatureFrontendClient() = default;

    virtual bool Running() const = 0;
    virtual bool Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err) = 0;
    virtual bool DetectAndCompute(const cv::Mat &gray, ExternalFeatureSet &outFeatures, std::string *err) = 0;
    virtual bool DetectAndComputeStereo(const cv::Mat &leftGray, const cv::Mat &rightGray,
                                        ExternalFeatureSet &leftFeatures, ExternalFeatureSet &rightFeatures,
                                        std::string *err) = 0;
    virtual void SetLightGlueEveryNOverride(int everyN) = 0;
    virtual Stats LastStats() const = 0;
};

} // namespace smartdrone::adapters::slam
