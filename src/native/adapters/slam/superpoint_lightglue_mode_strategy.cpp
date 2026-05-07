#include "adapters/slam/slam_mode_strategy.h"

#include <chrono>
#include <algorithm>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "adapters/slam/slam_engine_access.h"
#include "adapters/slam/slam_mode_common.h"
#include "adapters/slam/slam_tracking_backend.h"

namespace smartdrone::adapters::slam {

namespace {

core::ports::SlamOutput MakeExternalFeatureFailureOutput(SlamModeSharedState &state,
                                                         const core::ports::SlamInputBatch &input)
{
    core::ports::SlamOutput out{};
    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    state.CopyExternalFeatureStatsToOutput(out);
    return out;
}

int ChooseLightGlueCadence(const SlamModeSharedState &state, ORB_SLAM3::System &system)
{
    const int baseEveryN = EnvIntValueClamped("SMART_DRONE_LIGHTGLUE_EVERY_N", 4, 1, 120);
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_ADAPTIVE_CADENCE", false)) {
        return baseEveryN;
    }

    const int stableEveryN =
        EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_LIGHTGLUE_EVERY_N", std::min(baseEveryN + 1, 120), baseEveryN, 120);
    if (stableEveryN <= baseEveryN) {
        return baseEveryN;
    }

    const int trackingState = system.GetTrackingState();
    const int trackedMapPoints = system.GetTrackedMapPointCount();
    const int okStreakMin = EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_OK_STREAK", 120, 1, 100000);
    const int trackedMapPointMin = EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_TRACKED_MPS", 96, 1, 100000);
    if (trackingState == ORB_SLAM3::Tracking::OK && state.m_superPointLightGlueOkStreak >= okStreakMin &&
        trackedMapPoints >= trackedMapPointMin) {
        return stableEveryN;
    }
    return baseEveryN;
}

void UpdateLightGlueCadenceState(SlamModeSharedState &state, const core::ports::SlamOutput &out)
{
    const int trackedMapPointMin = EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_TRACKED_MPS", 96, 1, 100000);
    if (out.trackingState == ORB_SLAM3::Tracking::OK &&
        static_cast<int>(out.trackedMapPointCount) >= trackedMapPointMin) {
        ++state.m_superPointLightGlueOkStreak;
    } else {
        state.m_superPointLightGlueOkStreak = 0;
    }
}

} // namespace

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
        return MakeExternalFeatureFailureOutput(state, input);
    }

    const int lightGlueEveryN = ChooseLightGlueCadence(state, *system);
    state.m_externalFeatureFrontendClient->SetLightGlueEveryNOverride(lightGlueEveryN);
    state.m_superPointLightGlueLastEveryN = lightGlueEveryN;
    state.ResetExternalFeatureStats();

    const auto externalStartTp = std::chrono::steady_clock::now();
    const auto prepareStartTp = externalStartTp;
    cv::Mat leftPrepared = EnsureGray8(input.stereo.left.gray);
    cv::Mat rightPrepared = EnsureGray8(input.stereo.right.gray);
    if (leftPrepared.empty() || rightPrepared.empty()) {
        return MakeExternalFeatureFailureOutput(state, input);
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
        std::cerr << "[superpoint_lightglue] frontend_failed frame_id=" << input.frameId
                  << " err=" << featureErr << "\n";
        return MakeExternalFeatureFailureOutput(state, input);
    }

    const auto frontendEndTp = std::chrono::steady_clock::now();
    const ExternalFeatureFrontendClient::Stats stats = state.m_externalFeatureFrontendClient->LastStats();
    state.m_lastSuperPointPrepareMs = stats.prepareMs;
    state.m_lastSuperPointInputMs = stats.inputMs;
    state.m_lastSuperPointForwardMs = stats.forwardMs;
    state.m_lastSuperPointFrontendMs = stats.totalMs;
    state.m_lastSuperPointImageCount = stats.imageCount;
    state.m_lastSuperPointPayloadBytes = stats.payloadBytes;
    state.m_lastSuperPointRawLeftCount = stats.rawLeftCount;
    state.m_lastSuperPointRawRightCount = stats.rawRightCount;

    RemapKeypointsToSource(leftFeatures.keypoints, leftScaleX, leftScaleY);
    RemapKeypointsToSource(rightFeatures.keypoints, rightScaleX, rightScaleY);

    ORB_SLAM3::Tracking *tracker = system->GetTracker();
    const auto matchStartTp = std::chrono::steady_clock::now();
    const size_t pairedFeatureCount = std::min(leftFeatures.keypoints.size(), rightFeatures.keypoints.size());
    const bool initializingExternalStereo =
        tracker != nullptr && (tracker->mState == ORB_SLAM3::Tracking::NO_IMAGES_YET ||
                               tracker->mState == ORB_SLAM3::Tracking::NOT_INITIALIZED);
    const bool recoveringExternalStereo =
        tracker != nullptr && (tracker->mState == ORB_SLAM3::Tracking::RECENTLY_LOST ||
                               tracker->mState == ORB_SLAM3::Tracking::LOST);
    const bool trustFrontendInitPairs =
        initializingExternalStereo &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_INIT_TRUST_FRONTEND_PAIRS", false);
    const bool trustFrontendRecoveryPairs =
        recoveringExternalStereo &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_RECOVERY_TRUST_FRONTEND_PAIRS", false);
    const int trustFrontendOkStreak =
        EnvIntValueClamped("SMART_DRONE_SP_LG_TRUST_FRONTEND_PAIRS_OK_STREAK", 0, 0, 100000);
    const bool trustFrontendBootstrapPairs =
        trustFrontendOkStreak > 0 &&
        state.m_superPointLightGlueOkStreak < trustFrontendOkStreak &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_FRONTEND_PAIRS", false);
    const bool trustFrontendPairs =
        pairedFeatureCount > 0 &&
        (trustFrontendInitPairs || trustFrontendRecoveryPairs || trustFrontendBootstrapPairs);
    std::vector<StereoMatchPair> rawMatches;
    if (!trustFrontendPairs) {
        rawMatches = BuildAlignedStereoPairs(leftFeatures, rightFeatures, leftPrepared, rightPrepared);
    }
    const bool initializationStereoBias =
        EnvFlagEnabled("SMART_DRONE_SP_LG_INIT_STEREO_BIAS", false) &&
        initializingExternalStereo;
    if (initializationStereoBias && !rawMatches.empty()) {
        const float closeDisparity =
            EnvFloatValue("SMART_DRONE_SP_LG_INIT_CLOSE_DISPARITY", 4.0f);
        std::stable_sort(rawMatches.begin(), rawMatches.end(),
                         [closeDisparity](const StereoMatchPair &lhs, const StereoMatchPair &rhs) {
                             const bool lhsClose = lhs.disparity > closeDisparity;
                             const bool rhsClose = rhs.disparity > closeDisparity;
                             if (lhsClose != rhsClose) {
                                 return lhsClose;
                             }
                             if (std::abs(lhs.quality - rhs.quality) > 1.0e-6f) {
                                 return lhs.quality > rhs.quality;
                             }
                             if (std::abs(lhs.zncc - rhs.zncc) > 1.0e-6f) {
                                 return lhs.zncc > rhs.zncc;
                             }
                             return lhs.disparity > rhs.disparity;
                         });
    }
    const std::vector<StereoMatchPair> matches = FilterStereoPairsByDisparityConsistency(rawMatches);
    std::vector<cv::Point2f> matchedLeftPoints;
    std::vector<cv::Point2f> matchedRightPoints;
    auto appendMatchedPairs = [&](const std::vector<StereoMatchPair> &sourceMatches) {
        matchedLeftPoints.clear();
        matchedRightPoints.clear();
        matchedLeftPoints.reserve(sourceMatches.size());
        matchedRightPoints.reserve(sourceMatches.size());
        for (const StereoMatchPair &match : sourceMatches) {
            if (match.leftIndex < 0 || match.rightIndex < 0 ||
                static_cast<size_t>(match.leftIndex) >= leftFeatures.keypoints.size() ||
                static_cast<size_t>(match.rightIndex) >= rightFeatures.keypoints.size()) {
                continue;
            }
            matchedLeftPoints.push_back(leftFeatures.keypoints[static_cast<size_t>(match.leftIndex)]);
            matchedRightPoints.push_back(rightFeatures.keypoints[static_cast<size_t>(match.rightIndex)]);
        }
    };
    if (trustFrontendPairs) {
        matchedLeftPoints.reserve(pairedFeatureCount);
        matchedRightPoints.reserve(pairedFeatureCount);
        for (size_t i = 0; i < pairedFeatureCount; ++i) {
            matchedLeftPoints.push_back(leftFeatures.keypoints[i]);
            matchedRightPoints.push_back(rightFeatures.keypoints[i]);
        }
    } else if (initializationStereoBias && !rawMatches.empty()) {
        appendMatchedPairs(rawMatches);
    } else if (EnvFlagEnabled("SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT", true) && !matches.empty()) {
        appendMatchedPairs(matches);
    } else {
        matchedLeftPoints.reserve(pairedFeatureCount);
        matchedRightPoints.reserve(pairedFeatureCount);
        for (size_t i = 0; i < pairedFeatureCount; ++i) {
            matchedLeftPoints.push_back(leftFeatures.keypoints[i]);
            matchedRightPoints.push_back(rightFeatures.keypoints[i]);
        }
    }
    const double stereoPairMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - matchStartTp).count();
    state.m_lastSuperPointStereoMatchMs = stereoPairMs;
    state.m_lastSuperPointMatchedStereoCount = static_cast<int>(matchedLeftPoints.size());

    ORB_SLAM3::ExternalStereoFrameData externalData;
    const auto packStartTp = std::chrono::steady_clock::now();
    bool packedExternalData = false;
    if (EnvFlagEnabled("SMART_DRONE_SP_LG_NATIVE_DESCRIPTOR_INJECT", false)) {
        std::vector<StereoMatchPair> descriptorMatches;
        if (EnvFlagEnabled("SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT", true) && !matches.empty()) {
            descriptorMatches = matches;
        } else {
            descriptorMatches.reserve(pairedFeatureCount);
            for (size_t i = 0; i < pairedFeatureCount; ++i) {
                descriptorMatches.push_back(StereoMatchPair{static_cast<int>(i), static_cast<int>(i),
                                                            1.0f, 1.0f, 0.0f, 1.0f});
            }
        }
        packedExternalData = BuildExternalStereoFromFeatureMatches(leftFeatures, rightFeatures, descriptorMatches,
                                                                   externalData);
    }
    const std::vector<StereoMatchPair> &depthMatches =
        EnvFlagEnabled("SMART_DRONE_SP_LG_DEPTH_DISPARITY_CONSISTENCY_FILTER", false) ? matches : rawMatches;
    if (!packedExternalData && EnvFlagEnabled("SMART_DRONE_SP_LG_ALL_LEFT_GEOMETRIC_DEPTH", false) &&
        !depthMatches.empty()) {
        packedExternalData = FinalizeStereoExternalFromPairsWithAllLeft(
            tracker != nullptr ? tracker->GetLeftORBExtractor() : nullptr,
            tracker != nullptr ? tracker->GetRightORBExtractor() : nullptr, leftPrepared, rightPrepared,
            matchedLeftPoints, depthMatches, leftFeatures, rightFeatures, externalData);
    }
    if (!packedExternalData &&
        !FinalizeStereoExternalFromPairs(tracker != nullptr ? tracker->GetLeftORBExtractor() : nullptr,
                                         tracker != nullptr ? tracker->GetRightORBExtractor() : nullptr, leftPrepared,
                                         rightPrepared, matchedLeftPoints, matchedRightPoints, externalData)) {
        appendMatchedPairs(matches);
        state.m_lastSuperPointMatchedStereoCount = static_cast<int>(matchedLeftPoints.size());
        if (!FinalizeStereoExternalFromPairs(tracker != nullptr ? tracker->GetLeftORBExtractor() : nullptr,
                                             tracker != nullptr ? tracker->GetRightORBExtractor() : nullptr,
                                             leftPrepared, rightPrepared, matchedLeftPoints, matchedRightPoints,
                                             externalData)) {
            return MakeExternalFeatureFailureOutput(state, input);
        }
    }

    const auto packEndTp = std::chrono::steady_clock::now();
    double monoAugmentMs = 0.0;
    const bool initializedForMonoAugmentation =
        tracker != nullptr && tracker->mState != ORB_SLAM3::Tracking::NO_IMAGES_YET &&
        tracker->mState != ORB_SLAM3::Tracking::NOT_INITIALIZED;
    if (initializedForMonoAugmentation &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_ORB_LEFT_AUGMENT", false)) {
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
    if (EnvFlagEnabled("SMART_DRONE_SP_LG_INJECT_DFX", false)) {
        std::cerr << "[sp_lg_inject_dfx] frame_id=" << input.frameId
                  << " initializing=" << (initializingExternalStereo ? "Y" : "N")
                  << " recovering=" << (recoveringExternalStereo ? "Y" : "N")
                  << " trust_frontend_pairs=" << (trustFrontendPairs ? "Y" : "N")
                  << " ok_streak=" << state.m_superPointLightGlueOkStreak
                  << " frontend_pairs=" << pairedFeatureCount
                  << " raw_matches=" << rawMatches.size()
                  << " filtered_matches=" << matches.size()
                  << " selected_pairs=" << matchedLeftPoints.size()
                  << " injected=" << externalData.leftKeypoints.size()
                  << "/" << externalData.rightKeypoints.size()
                  << " scale=" << leftScaleX << "x" << leftScaleY
                  << "\n";
    }

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

    core::ports::SlamOutput out = RunSlamTrackingBackend(engine, input, extractFeatures, extractPointCloud, &request);
    UpdateLightGlueCadenceState(state, out);
    return out;
}

std::unique_ptr<SlamModeStrategy> CreateSuperPointLightGlueModeStrategy()
{
    return std::make_unique<SuperPointLightGlueModeStrategy>();
}

} // namespace smartdrone::adapters::slam
