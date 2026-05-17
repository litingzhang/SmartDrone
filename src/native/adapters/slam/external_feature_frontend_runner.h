#pragma once

#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

#include "adapters/slam/external_feature_frontend_client.h"
#include "adapters/slam/external_feature_types.h"

namespace smartdrone::adapters::slam {

struct ExternalStereoFeatureFrontendRunInput {
    ExternalFeatureFrontendClient *client{nullptr};
    const cv::Mat *leftPrepared{nullptr};
    const cv::Mat *rightPrepared{nullptr};
    int inputMaxWidth{0};
    int inputMaxHeight{0};
};

struct ExternalStereoFeatureFrontendRunResult {
    ExternalFeatureSet leftFeatures;
    ExternalFeatureSet rightFeatures;
    ExternalFeatureFrontendStats stats;
    double inputBuildMs{0.0};
    double frontendCallMs{0.0};
    float leftScaleX{1.0f};
    float leftScaleY{1.0f};
    float rightScaleX{1.0f};
    float rightScaleY{1.0f};
    std::string error;
};

bool RunExternalStereoFeatureFrontend(const ExternalStereoFeatureFrontendRunInput &input,
                                      ExternalStereoFeatureFrontendRunResult &result);

} // namespace smartdrone::adapters::slam
