#include "adapters/slam/slam_mode_strategy.h"

#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "adapters/slam/slam_engine_access.h"
#include "adapters/slam/slam_mode_common.h"
#include "adapters/slam/slam_tracking_backend.h"

namespace smartdrone::adapters::slam {

FeatureFrontend SuperPointLightGlueModeStrategy::Frontend() const { return FeatureFrontend::SuperPointLightGlue; }

core::ports::SlamOutput SuperPointLightGlueModeStrategy::Process(SlamEngineAdapter &engine,
                                                                 const core::ports::SlamInputBatch &input,
                                                                 bool extractFeatures, bool extractPointCloud)
{
    ORB_SLAM3::System *system = SlamEngineAccess::System(engine);
    if (system == nullptr) {
        return {};
    }

    SlamModeSharedState &state = SlamEngineAccess::ModeState(engine);
    const bool monoMode = SlamEngineAccess::InputMode(engine) != SlamInputMode::Stereo;
    if (monoMode || state.m_externalFeatureFrontendClient == nullptr || !state.m_externalFeatureFrontendClient->Running()) {
        return RunSlamTrackingBackend(engine, input, extractFeatures, extractPointCloud, nullptr);
    }

    state.ResetExternalFeatureStats();

    const auto externalStartTp = std::chrono::steady_clock::now();
    const auto prepareStartTp = externalStartTp;
    cv::Mat leftPrepared = EnsureGray8(input.stereo.left.gray);
    cv::Mat rightPrepared = EnsureGray8(input.stereo.right.gray);
    if (leftPrepared.empty() || rightPrepared.empty()) {
        return RunSlamTrackingBackend(engine, input, extractFeatures, extractPointCloud, nullptr);
    }

    if (state.m_lkCalibrationLoaded) {
        state.EnsureStereoRectifier(leftPrepared.size());
        if (!state.m_lkMap1x.empty() && !state.m_lkMap2x.empty()) {
            cv::Mat leftRect;
            cv::Mat rightRect;
            cv::remap(leftPrepared, leftRect, state.m_lkMap1x, state.m_lkMap1y, cv::INTER_LINEAR);
            cv::remap(rightPrepared, rightRect, state.m_lkMap2x, state.m_lkMap2y, cv::INTER_LINEAR);
            leftPrepared = std::move(leftRect);
            rightPrepared = std::move(rightRect);
        }
    }

    float leftScaleX = 1.0f;
    float leftScaleY = 1.0f;
    float rightScaleX = 1.0f;
    float rightScaleY = 1.0f;
    const cv::Mat leftInput = BuildSuperPointInputImage(leftPrepared, state.m_externalFeatureInputMaxWidth,
                                                        state.m_externalFeatureInputMaxHeight, leftScaleX, leftScaleY);
    const cv::Mat rightInput = BuildSuperPointInputImage(rightPrepared, state.m_externalFeatureInputMaxWidth,
                                                         state.m_externalFeatureInputMaxHeight, rightScaleX, rightScaleY);
    const auto inputEndTp = std::chrono::steady_clock::now();

    SuperPointFeatureSet leftFeatures;
    SuperPointFeatureSet rightFeatures;
    std::string featureErr;
    const auto frontendStartTp = std::chrono::steady_clock::now();
    if (!state.m_externalFeatureFrontendClient->DetectAndComputeStereo(leftInput, rightInput, leftFeatures, rightFeatures,
                                                                       &featureErr)) {
        return RunSlamTrackingBackend(engine, input, extractFeatures, extractPointCloud, nullptr);
    }

    const auto frontendEndTp = std::chrono::steady_clock::now();
    const ExternalFeatureFrontendClient::Stats stats = state.m_externalFeatureFrontendClient->LastStats();
    state.m_lastSuperPointPrepareMs = stats.prepareMs;
    state.m_lastSuperPointInputMs = stats.inputMs;
    state.m_lastSuperPointForwardMs = stats.forwardMs;
    state.m_lastSuperPointFrontendMs = stats.totalMs;
    state.m_lastSuperPointImageCount = stats.imageCount;
    state.m_lastSuperPointPayloadBytes = stats.payloadBytes;
    state.m_lastSuperPointRawLeftCount = static_cast<int>(leftFeatures.keypoints.size());
    state.m_lastSuperPointRawRightCount = static_cast<int>(rightFeatures.keypoints.size());

    RemapKeypointsToSource(leftFeatures.keypoints, leftScaleX, leftScaleY);
    RemapKeypointsToSource(rightFeatures.keypoints, rightScaleX, rightScaleY);

    const auto matchStartTp = std::chrono::steady_clock::now();
    const std::vector<StereoMatchPair> rawMatches =
        BuildAlignedStereoPairs(leftFeatures, rightFeatures, leftPrepared, rightPrepared);
    const std::vector<StereoMatchPair> matches = FilterStereoPairsByDisparityConsistency(rawMatches);
    const double stereoPairMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - matchStartTp).count();
    state.m_lastSuperPointStereoMatchMs = stereoPairMs;
    state.m_lastSuperPointMatchedStereoCount = static_cast<int>(matches.size());

    std::vector<cv::Point2f> matchedLeftPoints;
    std::vector<cv::Point2f> matchedRightPoints;
    matchedLeftPoints.reserve(matches.size());
    matchedRightPoints.reserve(matches.size());
    for (const StereoMatchPair &match : matches) {
        if (match.leftIndex < 0 || match.rightIndex < 0 ||
            static_cast<size_t>(match.leftIndex) >= leftFeatures.keypoints.size() ||
            static_cast<size_t>(match.rightIndex) >= rightFeatures.keypoints.size()) {
            continue;
        }
        matchedLeftPoints.push_back(leftFeatures.keypoints[static_cast<size_t>(match.leftIndex)]);
        matchedRightPoints.push_back(rightFeatures.keypoints[static_cast<size_t>(match.rightIndex)]);
    }

    ORB_SLAM3::ExternalStereoFrameData externalData;
    ORB_SLAM3::Tracking *tracker = system->GetTracker();
    const auto packStartTp = std::chrono::steady_clock::now();
    if (!FinalizeStereoExternalFromPairs(tracker != nullptr ? tracker->GetLeftORBExtractor() : nullptr,
                                         tracker != nullptr ? tracker->GetRightORBExtractor() : nullptr, leftPrepared,
                                         rightPrepared, matchedLeftPoints, matchedRightPoints, externalData)) {
        return RunSlamTrackingBackend(engine, input, extractFeatures, extractPointCloud, nullptr);
    }

    const auto packEndTp = std::chrono::steady_clock::now();
    double monoAugmentMs = 0.0;
    const bool initializedForMonoAugmentation =
        tracker != nullptr && tracker->mState != ORB_SLAM3::Tracking::NO_IMAGES_YET &&
        tracker->mState != ORB_SLAM3::Tracking::NOT_INITIALIZED;
    if (initializedForMonoAugmentation) {
        const auto augmentStartTp = std::chrono::steady_clock::now();
        const size_t maxLeftFeatures =
            EnvSizeValueClamped("SMART_DRONE_EXTERNAL_STEREO_MAX_LEFT_FEATURES", kExternalStereoMaxLeftFeatures,
                                kExternalStereoMaxLeftFeatures, kExternalStereoMaxLeftFeaturesLimit);
        AppendOrbLeftOnlyFeatures(tracker->GetLeftORBExtractor(), leftPrepared, externalData, maxLeftFeatures);
        monoAugmentMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - augmentStartTp).count();
    }

    state.m_lastSuperPointInjectedLeftCount = static_cast<int>(externalData.leftKeypoints.size());
    state.m_lastSuperPointInjectedRightCount = static_cast<int>(externalData.rightKeypoints.size());

    ExternalStereoTrackRequest request;
    request.enabled = true;
    request.recordTotalMs = true;
    request.totalStartTp = externalStartTp;
    request.leftPrepared = std::move(leftPrepared);
    request.rightPrepared = std::move(rightPrepared);
    request.externalData = std::move(externalData);
    request.inputPrepareMs = std::chrono::duration<double, std::milli>(inputEndTp - prepareStartTp).count();
    request.frontendMs = std::chrono::duration<double, std::milli>(frontendEndTp - frontendStartTp).count();
    request.stereoPairMs = stereoPairMs;
    request.externalPackMs = std::chrono::duration<double, std::milli>(packEndTp - packStartTp).count();
    request.monoAugmentMs = monoAugmentMs;
    request.leftFeaturePoints.reserve(request.externalData.leftKeypoints.size());
    request.rightFeaturePoints.reserve(request.externalData.rightKeypoints.size());
    for (const cv::KeyPoint &kp : request.externalData.leftKeypoints) {
        request.leftFeaturePoints.push_back(kp.pt);
    }
    for (const cv::KeyPoint &kp : request.externalData.rightKeypoints) {
        request.rightFeaturePoints.push_back(kp.pt);
    }

    return RunSlamTrackingBackend(engine, input, extractFeatures, extractPointCloud, &request);
}

std::unique_ptr<SlamModeStrategy> CreateSuperPointLightGlueModeStrategy()
{
    return std::make_unique<SuperPointLightGlueModeStrategy>();
}

} // namespace smartdrone::adapters::slam
