#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "core/ports/visual_feature_frontend.h"

namespace SmartDrone::adapters::slam {

struct StereoFeatureFrontendRunInput {
    core::ports::IVisualFeatureFrontend *client{nullptr};
    const cv::Mat *leftPrepared{nullptr};
    const cv::Mat *rightPrepared{nullptr};
    int inputMaxWidth{0};
    int inputMaxHeight{0};
};

struct StereoFeatureFrontendRunResult {
    core::ports::VisualFeatureSet leftFeatures;
    core::ports::VisualFeatureSet rightFeatures;
    core::ports::VisualFeatureFrontendStats stats;
    double inputBuildMs{0.0};
    double frontendCallMs{0.0};
    float leftScaleX{1.0f};
    float leftScaleY{1.0f};
    float rightScaleX{1.0f};
    float rightScaleY{1.0f};
    std::string error;
};

bool RunStereoFeatureFrontend(const StereoFeatureFrontendRunInput &input,
                              StereoFeatureFrontendRunResult &result);

} // namespace SmartDrone::adapters::slam
