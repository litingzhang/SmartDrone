#pragma once

#include <chrono>
#include <vector>

#include <opencv2/core.hpp>

#include "core/ports/slam_engine.h"
#include "core/ports/visual_feature_data.h"

namespace SmartDrone::Adapters::Slam {

class SlamEngineAdapter;

struct StereoFeatureTrackRequest {
    bool enabled{false};
    bool recordTotalMs{false};
    std::chrono::steady_clock::time_point totalStartTp{};
    cv::Mat leftPrepared;
    cv::Mat rightPrepared;
    Core::Ports::StereoFeatureObservationPacket observations;
    std::vector<cv::Point2f> leftFeaturePoints;
    std::vector<cv::Point2f> rightFeaturePoints;
    uint64_t observationHash{0};
    double inputPrepareMs{0.0};
    double frontendMs{0.0};
    double stereoPairMs{0.0};
    double featurePackMs{0.0};
    double monoAugmentMs{0.0};
};

Core::Ports::SlamOutput
RunSlamTrackingBackend(SlamEngineAdapter &engine,
                       const Core::Ports::SlamInputBatch &input,
                       bool extractFeatures, bool extractPointCloud,
                       const StereoFeatureTrackRequest *stereoFeatureRequest);

} // namespace SmartDrone::Adapters::Slam
