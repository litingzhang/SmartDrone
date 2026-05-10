#pragma once

#include <chrono>
#include <vector>

#include <opencv2/core.hpp>

#include "Frame.h"
#include "core/ports/slam_engine.h"

namespace smartdrone::adapters::slam {

class SlamEngineAdapter;

struct ExternalStereoTrackRequest {
    bool enabled{false};
    bool recordTotalMs{false};
    std::chrono::steady_clock::time_point totalStartTp{};
    cv::Mat leftPrepared;
    cv::Mat rightPrepared;
    ORB_SLAM3::ExternalStereoFrameData externalData;
    std::vector<cv::Point2f> leftFeaturePoints;
    std::vector<cv::Point2f> rightFeaturePoints;
    uint64_t externalHash{0};
    double inputPrepareMs{0.0};
    double frontendMs{0.0};
    double stereoPairMs{0.0};
    double externalPackMs{0.0};
    double monoAugmentMs{0.0};
};

core::ports::SlamOutput RunSlamTrackingBackend(SlamEngineAdapter &engine,
                                                  const core::ports::SlamInputBatch &input,
                                                  bool extractFeatures, bool extractPointCloud,
                                                  const ExternalStereoTrackRequest *externalRequest);

} // namespace smartdrone::adapters::slam
