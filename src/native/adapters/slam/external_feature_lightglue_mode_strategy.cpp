#include "adapters/slam/slam_mode_strategy.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "adapters/slam/external_feature_frontend_runner.h"
#include "adapters/slam/external_stereo_feature_packet.h"
#include "adapters/slam/external_stereo_matching.h"
#include "adapters/slam/external_temporal_stereo.h"
#include "adapters/slam/orb_slam3_backend.h"
#include "adapters/slam/slam_engine_access.h"
#include "adapters/slam/slam_env.h"
#include "adapters/slam/slam_image_utils.h"
#include "adapters/slam/slam_output_utils.h"
#include "adapters/slam/slam_tracking_backend.h"
#include "adapters/slam/stereo_geometry.h"
#include "core/ports/slam_tracking_state.h"

namespace smartdrone::adapters::slam {

namespace {

core::ports::SlamOutput MakeExternalFeatureFailureOutput(SlamEngineAdapter &engine, SlamModeSharedState &state,
                                                         const core::ports::SlamInputBatch &input)
{
    core::ports::SlamOutput out =
        MakePoseLostSlamOutput(&engine, input, core::ports::kSlamTrackingRecentlyLost,
                               true, true);
    state.CopyExternalFeatureStatsToOutput(out);
    return out;
}

int ChooseLightGlueCadence(const SlamModeSharedState &state, const OrbSlam3Backend &backend)
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

    const int trackingState = backend.TrackingState();
    const int trackedMapPoints = backend.TrackedMapPointCount();
    const int okStreakMin = EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_OK_STREAK", 120, 1, 100000);
    const int trackedMapPointMin = EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_TRACKED_MPS", 96, 1, 100000);
    if (trackingState == core::ports::kSlamTrackingOk && state.m_superPointLightGlueOkStreak >= okStreakMin &&
        trackedMapPoints >= trackedMapPointMin) {
        return stableEveryN;
    }
    return baseEveryN;
}

void UpdateLightGlueCadenceState(SlamModeSharedState &state, const core::ports::SlamOutput &out)
{
    const int trackedMapPointMin = EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_TRACKED_MPS", 96, 1, 100000);
    const int trustFrontendOkStreak =
        EnvIntValueClamped("SMART_DRONE_SP_LG_TRUST_FRONTEND_PAIRS_OK_STREAK", 0, 0, 100000);
    const int bootstrapTrustHoldTrackedMapMin =
        EnvIntValueClamped("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_HOLD_TRACKED_MPS",
                           trackedMapPointMin, 1, 100000);
    const int trackedMapPoints = static_cast<int>(out.trackedMapPointCount);
    const bool bootstrapTrustAlreadyMature =
        trustFrontendOkStreak > 0 && state.m_superPointLightGlueOkStreak >= trustFrontendOkStreak;
    const bool holdMatureBootstrapTrust =
        bootstrapTrustAlreadyMature && trackedMapPoints >= bootstrapTrustHoldTrackedMapMin;
    if (out.trackingState == core::ports::kSlamTrackingOk &&
        (trackedMapPoints >= trackedMapPointMin || holdMatureBootstrapTrust)) {
        ++state.m_superPointLightGlueOkStreak;
    } else {
        state.m_superPointLightGlueOkStreak = 0;
    }
}

void UpdateBootstrapTrustState(SlamModeSharedState &state, bool usedBootstrapTrust, int trustFrontendOkStreak)
{
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_ONCE", false)) {
        return;
    }
    if (trustFrontendOkStreak <= 0) {
        state.m_superPointLightGlueBootstrapTrustClosed = true;
        return;
    }
    if (usedBootstrapTrust) {
        ++state.m_superPointLightGlueBootstrapTrustFrames;
    }
    if (state.m_superPointLightGlueOkStreak >= trustFrontendOkStreak ||
        state.m_superPointLightGlueBootstrapTrustFrames >= trustFrontendOkStreak) {
        state.m_superPointLightGlueBootstrapTrustClosed = true;
    }
}

bool PreviousFrameWasWeak(const SlamModeSharedState &state)
{
    const int minInliers = EnvIntValueClamped("SMART_DRONE_SP_LG_WEAK_FRAME_MIN_INLIERS", 90, 1, 100000);
    const int minTrackedMap = EnvIntValueClamped("SMART_DRONE_SP_LG_WEAK_FRAME_MIN_TRACKED_MAP", 120, 1, 100000);
    return state.m_lastSlamMatchesInliers > 0 &&
           (state.m_lastSlamMatchesInliers < minInliers || state.m_lastSlamTrackedMapPoints < minTrackedMap);
}

size_t RefineSelectedRightPointsByZncc(const cv::Mat &leftPrepared, const cv::Mat &rightPrepared,
                                       std::vector<cv::Point2f> &leftPoints,
                                       std::vector<cv::Point2f> &rightPoints)
{
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_REFINE_RIGHT_ZNCC", false) || leftPrepared.empty() ||
        rightPrepared.empty() || leftPoints.empty() || leftPoints.size() != rightPoints.size()) {
        return 0;
    }

    cv::Mat left32f;
    cv::Mat right32f;
    leftPrepared.convertTo(left32f, CV_32F);
    rightPrepared.convertTo(right32f, CV_32F);

    const float maxShiftPx =
        EnvFloatValueClamped("SMART_DRONE_SP_LG_REFINE_RIGHT_ZNCC_MAX_SHIFT_PX", 4.0f, 0.25f, 16.0f);
    size_t refinedCount = 0;
    for (size_t i = 0; i < leftPoints.size(); ++i) {
        cv::Point2f refinedRight = rightPoints[i];
        float zncc = -1.0f;
        if (!RefineRightPointByStereoZncc(left32f, leftPoints[i], right32f, rightPoints[i], refinedRight, zncc)) {
            continue;
        }
        if (std::fabs(refinedRight.x - rightPoints[i].x) > maxShiftPx ||
            std::fabs(refinedRight.y - rightPoints[i].y) > 1.0e-3f) {
            continue;
        }
        if (!IsStereoPairGeometricallyValid(leftPoints[i], refinedRight)) {
            continue;
        }
        rightPoints[i] = refinedRight;
        ++refinedCount;
    }
    return refinedCount;
}

void StoreTemporalCarrySource(SlamModeSharedState &state, const cv::Mat &leftPrepared, const cv::Mat &rightPrepared,
                              const ExternalStereoObservationPacket &externalData)
{
    cv::Mat prevLeft;
    cv::Mat prevRight;
    std::vector<cv::Point2f> leftPoints;
    std::vector<cv::Point2f> rightPoints;
    if (!ExtractExternalTemporalStereoSource(leftPrepared, rightPrepared, externalData, prevLeft, prevRight,
                                             leftPoints, rightPoints)) {
        state.m_spLgHavePrevStereo = false;
        state.m_spLgPrevLeftPoints.clear();
        state.m_spLgPrevRightPoints.clear();
        return;
    }

    state.m_spLgPrevLeft = std::move(prevLeft);
    state.m_spLgPrevRight = std::move(prevRight);
    state.m_spLgPrevLeftPoints = std::move(leftPoints);
    state.m_spLgPrevRightPoints = std::move(rightPoints);
    state.m_spLgHavePrevStereo = true;
}

} // namespace

ExternalFeatureLightGlueModeStrategy::ExternalFeatureLightGlueModeStrategy(FeatureFrontend frontend)
    : m_frontend(frontend)
{
}

FeatureFrontend ExternalFeatureLightGlueModeStrategy::Frontend() const { return m_frontend; }

core::ports::SlamOutput ExternalFeatureLightGlueModeStrategy::Process(SlamEngineAdapter &engine,
                                                                      const core::ports::SlamInputBatch &input,
                                                                      bool extractFeatures, bool extractPointCloud)
{
    OrbSlam3Backend *backend = SlamEngineAccess::OrbBackend(engine);
    if (backend == nullptr || !backend->Available()) {
        return {};
    }

    SlamModeSharedState &state = SlamEngineAccess::ModeState(engine);
    const bool monoMode = SlamEngineAccess::InputMode(engine) != SlamInputMode::Stereo;
    if (monoMode || state.m_externalFeatureFrontendClient == nullptr || !state.m_externalFeatureFrontendClient->Running()) {
        return MakeExternalFeatureFailureOutput(engine, state, input);
    }

    const int lightGlueEveryN = ChooseLightGlueCadence(state, *backend);
    state.m_externalFeatureFrontendClient->SetLightGlueEveryNOverride(lightGlueEveryN);
    state.m_superPointLightGlueLastEveryN = lightGlueEveryN;
    state.ResetExternalFeatureStats();

    const auto externalStartTp = std::chrono::steady_clock::now();
    const auto prepareStartTp = externalStartTp;
    cv::Mat leftPrepared = EnsureGray8(input.stereo.left.gray);
    cv::Mat rightPrepared = EnsureGray8(input.stereo.right.gray);
    if (leftPrepared.empty() || rightPrepared.empty()) {
        return MakeExternalFeatureFailureOutput(engine, state, input);
    }

    if (EnvFlagEnabled("SMART_DRONE_SP_LG_USE_ORB_PREPARED_IMAGES", false)) {
        if (!backend->PrepareStereoImagesForTracking(input.stereo.left.gray, input.stereo.right.gray, leftPrepared,
                                                     rightPrepared)) {
            return MakeExternalFeatureFailureOutput(engine, state, input);
        }
        leftPrepared = EnsureGray8(leftPrepared);
        rightPrepared = EnsureGray8(rightPrepared);
        if (leftPrepared.empty() || rightPrepared.empty()) {
            return MakeExternalFeatureFailureOutput(engine, state, input);
        }
    } else if (state.m_lkCalibrationLoaded) {
        cv::Mat leftRect;
        cv::Mat rightRect;
        if (state.PrepareRectifiedStereoCpu(leftPrepared, rightPrepared, leftRect, rightRect)) {
            leftPrepared = std::move(leftRect);
            rightPrepared = std::move(rightRect);
        }
    }

    const auto prepareEndTp = std::chrono::steady_clock::now();
    ExternalStereoFeatureFrontendRunInput frontendInput;
    frontendInput.client = state.m_externalFeatureFrontendClient;
    frontendInput.leftPrepared = &leftPrepared;
    frontendInput.rightPrepared = &rightPrepared;
    frontendInput.inputMaxWidth = state.m_externalFeatureInputMaxWidth;
    frontendInput.inputMaxHeight = state.m_externalFeatureInputMaxHeight;
    ExternalStereoFeatureFrontendRunResult frontendResult;
    if (!RunExternalStereoFeatureFrontend(frontendInput, frontendResult)) {
        std::cerr << "[" << ToFeatureFrontendText(m_frontend) << "] frontend_failed frame_id=" << input.frameId
                  << " err=" << frontendResult.error << "\n";
        return MakeExternalFeatureFailureOutput(engine, state, input);
    }

    ExternalFeatureSet leftFeatures = std::move(frontendResult.leftFeatures);
    ExternalFeatureSet rightFeatures = std::move(frontendResult.rightFeatures);
    const ExternalFeatureFrontendClient::Stats &stats = frontendResult.stats;
    state.m_lastSuperPointPrepareMs = stats.prepareMs;
    state.m_lastSuperPointInputMs = stats.inputMs;
    state.m_lastSuperPointForwardMs = stats.forwardMs;
    state.m_lastSuperPointFrontendMs = stats.totalMs;
    state.m_lastSuperPointImageCount = stats.imageCount;
    state.m_lastSuperPointPayloadBytes = stats.payloadBytes;
    state.m_lastSuperPointRawLeftCount = stats.rawLeftCount;
    state.m_lastSuperPointRawRightCount = stats.rawRightCount;

    const auto matchStartTp = std::chrono::steady_clock::now();
    const bool initializingExternalStereo = backend->IsTrackingInitializing();
    const bool recoveringExternalStereo = backend->IsTrackingRecovering();
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
        (!EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_ONCE", false) ||
         !state.m_superPointLightGlueBootstrapTrustClosed) &&
        state.m_superPointLightGlueOkStreak < trustFrontendOkStreak &&
        (!EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_ONCE", false) ||
         state.m_superPointLightGlueBootstrapTrustFrames < trustFrontendOkStreak) &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_FRONTEND_PAIRS", false);

    ExternalStereoMatchSelectionInput matchInput;
    matchInput.leftFeatures = &leftFeatures;
    matchInput.rightFeatures = &rightFeatures;
    matchInput.leftPrepared = &leftPrepared;
    matchInput.rightPrepared = &rightPrepared;
    matchInput.initializing = initializingExternalStereo;
    matchInput.recovering = recoveringExternalStereo;
    matchInput.trustFrontendInitPairs = trustFrontendInitPairs;
    matchInput.trustFrontendRecoveryPairs = trustFrontendRecoveryPairs;
    matchInput.trustFrontendBootstrapPairs = trustFrontendBootstrapPairs;
    matchInput.previousFrameWeak = PreviousFrameWasWeak(state);

    ExternalStereoMatchSelection matchSelection;
    if (!SelectExternalStereoMatches(matchInput, matchSelection)) {
        return MakeExternalFeatureFailureOutput(engine, state, input);
    }
    std::vector<cv::Point2f> matchedLeftPoints = std::move(matchSelection.matchedLeftPoints);
    std::vector<cv::Point2f> matchedRightPoints = std::move(matchSelection.matchedRightPoints);
    const size_t znccRefinedPairs =
        RefineSelectedRightPointsByZncc(leftPrepared, rightPrepared, matchedLeftPoints, matchedRightPoints);
    ExternalTemporalStereoStateView temporalState;
    temporalState.havePrevStereo = state.m_spLgHavePrevStereo;
    temporalState.prevLeft = &state.m_spLgPrevLeft;
    temporalState.prevRight = &state.m_spLgPrevRight;
    temporalState.prevLeftPoints = &state.m_spLgPrevLeftPoints;
    temporalState.prevRightPoints = &state.m_spLgPrevRightPoints;
    temporalState.previousFrameWeak = matchInput.previousFrameWeak;
    ExternalTemporalStereoCarryInput temporalInput;
    temporalInput.state = &temporalState;
    temporalInput.leftPrepared = &leftPrepared;
    temporalInput.rightPrepared = &rightPrepared;
    temporalInput.initializing = initializingExternalStereo;
    temporalInput.recovering = recoveringExternalStereo;
    const size_t temporalCarryPairs =
        AppendExternalTemporalStereoCarry(temporalInput, matchedLeftPoints, matchedRightPoints);
    const double stereoPairMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - matchStartTp).count();
    state.m_lastSuperPointStereoMatchMs = stereoPairMs;
    state.m_lastSuperPointMatchedStereoCount = static_cast<int>(matchedLeftPoints.size());

    const auto packStartTp = std::chrono::steady_clock::now();
    ExternalStereoFeaturePacketBuildInput packetInput;
    packetInput.leftPrepared = &leftPrepared;
    packetInput.rightPrepared = &rightPrepared;
    packetInput.matchedLeftPoints = &matchedLeftPoints;
    packetInput.matchedRightPoints = &matchedRightPoints;
    packetInput.filteredMatches = &matchSelection.filteredMatches;
    packetInput.rawMatches = &matchSelection.rawMatches;
    packetInput.leftFeatures = &leftFeatures;
    packetInput.rightFeatures = &rightFeatures;
    packetInput.leftDescriptorProvider = backend->LeftDescriptorProvider();
    packetInput.rightDescriptorProvider = backend->RightDescriptorProvider();
    packetInput.initializedForMonoAugmentation = backend->HasTrackingInitialized();
    packetInput.stableOkStreak = state.m_superPointLightGlueOkStreak;

    ExternalStereoFeaturePacket packet;
    if (!BuildExternalStereoFeaturePacket(packetInput, packet)) {
        CopyMatchedStereoPointsFromPairs(leftFeatures, rightFeatures, matchSelection.filteredMatches,
                                         matchedLeftPoints, matchedRightPoints);
        state.m_lastSuperPointMatchedStereoCount = static_cast<int>(matchedLeftPoints.size());
        packetInput.matchedLeftPoints = &matchedLeftPoints;
        packetInput.matchedRightPoints = &matchedRightPoints;
        packetInput.allowNativeDescriptorInject = false;
        packetInput.allowAllLeftGeometricDepth = false;
        if (!BuildExternalStereoFeaturePacket(packetInput, packet)) {
            return MakeExternalFeatureFailureOutput(engine, state, input);
        }
    }

    const auto packEndTp = std::chrono::steady_clock::now();
    state.m_lastSuperPointInjectedLeftCount = static_cast<int>(packet.observations.leftKeypoints.size());
    state.m_lastSuperPointInjectedRightCount = static_cast<int>(packet.observations.rightKeypoints.size());
    state.m_lastSuperPointExternalHash = packet.hash;
    if (EnvFlagEnabled("SMART_DRONE_SP_LG_INJECT_DFX", false)) {
        std::cerr << "[sp_lg_inject_dfx] frame_id=" << input.frameId
                  << " initializing=" << (initializingExternalStereo ? "Y" : "N")
                  << " recovering=" << (recoveringExternalStereo ? "Y" : "N")
                  << " trust_frontend_pairs=" << (matchSelection.trustFrontendPairs ? "Y" : "N")
                  << " init_trust_selected=" << (matchSelection.initializationTrustedPairSelection ? "Y" : "N")
                  << " bootstrap_trust=" << (trustFrontendBootstrapPairs ? "Y" : "N")
                  << " bootstrap_closed=" << (state.m_superPointLightGlueBootstrapTrustClosed ? "Y" : "N")
                  << " bootstrap_frames=" << state.m_superPointLightGlueBootstrapTrustFrames
                  << " ok_streak=" << state.m_superPointLightGlueOkStreak
                  << " prev_inliers=" << state.m_lastSlamMatchesInliers
                  << " prev_tracked=" << state.m_lastSlamTrackedMapPoints
                  << " frontend_pairs=" << matchSelection.pairedFeatureCount
                  << " init_trust_matches=" << matchSelection.initializationTrustedMatches.size()
                  << " raw_matches=" << matchSelection.rawMatches.size()
                  << " filtered_matches=" << matchSelection.filteredMatches.size()
                  << " selected_pairs=" << matchedLeftPoints.size()
                  << " zncc_refined=" << znccRefinedPairs
                  << " temporal_carry=" << temporalCarryPairs
                  << " orb_stereo_aug=" << packet.orbStereoAugmentPairs
                  << " injected=" << packet.observations.leftKeypoints.size()
                  << "/" << packet.observations.rightKeypoints.size()
                  << " hash=" << state.m_lastSuperPointExternalHash
                  << " scale=" << frontendResult.leftScaleX << "x" << frontendResult.leftScaleY
                  << "\n";
    }

    ExternalStereoTrackRequest request;
    request.enabled = true;
    request.recordTotalMs = true;
    request.totalStartTp = externalStartTp;
    request.leftPrepared = std::move(leftPrepared);
    request.rightPrepared = std::move(rightPrepared);
    request.observations = std::move(packet.observations);
    request.inputPrepareMs = std::chrono::duration<double, std::milli>(prepareEndTp - prepareStartTp).count() +
                             frontendResult.inputBuildMs;
    request.frontendMs = frontendResult.frontendCallMs;
    request.stereoPairMs = stereoPairMs;
    request.externalPackMs = std::chrono::duration<double, std::milli>(packEndTp - packStartTp).count();
    request.monoAugmentMs = packet.monoAugmentMs;
    request.externalHash = state.m_lastSuperPointExternalHash;
    request.leftFeaturePoints = std::move(packet.leftFeaturePoints);
    request.rightFeaturePoints = std::move(packet.rightFeaturePoints);
    StoreTemporalCarrySource(state, request.leftPrepared, request.rightPrepared, request.observations);

    core::ports::SlamOutput out = RunSlamTrackingBackend(engine, input, extractFeatures, extractPointCloud, &request);
    UpdateLightGlueCadenceState(state, out);
    UpdateBootstrapTrustState(state, trustFrontendBootstrapPairs && matchSelection.trustFrontendPairs,
                              trustFrontendOkStreak);
    state.m_lastSlamMatchesInliers = out.matchesInliers;
    state.m_lastSlamTrackedMapPoints = static_cast<int>(out.trackedMapPointCount);
    return out;
}

std::unique_ptr<SlamModeStrategy> CreateSuperPointLightGlueModeStrategy()
{
    return std::make_unique<ExternalFeatureLightGlueModeStrategy>(FeatureFrontend::SuperPointLightGlue);
}

std::unique_ptr<SlamModeStrategy> CreateXFeatLightGlueModeStrategy()
{
    return std::make_unique<ExternalFeatureLightGlueModeStrategy>(FeatureFrontend::XFeatLightGlue);
}

} // namespace smartdrone::adapters::slam
