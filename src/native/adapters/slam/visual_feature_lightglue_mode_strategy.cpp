#include "adapters/slam/slam_mode_strategy.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "adapters/slam/slam_engine_access.h"
#include "adapters/slam/slam_env.h"
#include "adapters/slam/slam_image_utils.h"
#include "adapters/slam/slam_output_utils.h"
#include "adapters/slam/slam_tracking_backend.h"
#include "adapters/slam/stereo_feature_frontend_runner.h"
#include "adapters/slam/stereo_feature_packet.h"
#include "adapters/slam/stereo_geometry.h"
#include "adapters/slam/stereo_matching.h"
#include "adapters/slam/temporal_stereo.h"
#include "core/ports/slam_tracking_state.h"

namespace SmartDrone::Adapters::Slam {

namespace {

std::unique_ptr<SlamModeStrategy>
CreateRegisteredSuperPointLightGlueModeStrategy()
{
    return CreateSuperPointLightGlueModeStrategy();
}

std::unique_ptr<SlamModeStrategy> CreateRegisteredXFeatLightGlueModeStrategy()
{
    return CreateXFeatLightGlueModeStrategy();
}

const SlamModeStrategyRegistrar kSuperPointLightGlueModeStrategyRegistration(
    FeatureFrontend::SuperPointLightGlue,
    &CreateRegisteredSuperPointLightGlueModeStrategy);

const SlamModeStrategyRegistrar kXFeatLightGlueModeStrategyRegistration(
    FeatureFrontend::XFeatLightGlue,
    &CreateRegisteredXFeatLightGlueModeStrategy);

Core::Ports::SlamOutput
MakeVisualFeatureFailureOutput(SlamEngineAdapter &engine,
                               SlamModeSharedState &state,
                               const Core::Ports::SlamInputBatch &input)
{
    Core::Ports::SlamOutput out = MakePoseLostSlamOutput(
        &engine, input, Core::Ports::kSlamTrackingRecentlyLost, true, true);
    state.CopyVisualFeatureStatsToOutput(out);
    return out;
}

int ChooseLightGlueCadence(
    const SlamModeSharedState &state,
    const Core::Ports::ISlamTrackingStatusProvider &trackingStatus)
{
    const int baseEveryN =
        EnvIntValueClamped("SMART_DRONE_LIGHTGLUE_EVERY_N", 4, 1, 120);
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_ADAPTIVE_CADENCE", false)) {
        return baseEveryN;
    }

    const int stableEveryN =
        EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_LIGHTGLUE_EVERY_N",
                           std::min(baseEveryN + 1, 120), baseEveryN, 120);
    if (stableEveryN <= baseEveryN) {
        return baseEveryN;
    }

    const int trackingState = trackingStatus.TrackingState();
    const int trackedMapPoints = trackingStatus.TrackedMapPointCount();
    const int okStreakMin =
        EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_OK_STREAK", 120, 1, 100000);
    const int trackedMapPointMin =
        EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_TRACKED_MPS", 96, 1, 100000);
    if (trackingState == Core::Ports::kSlamTrackingOk &&
        state.m_visualFeatureLightGlueOkStreak >= okStreakMin &&
        trackedMapPoints >= trackedMapPointMin) {
        return stableEveryN;
    }
    return baseEveryN;
}

void UpdateLightGlueCadenceState(SlamModeSharedState &state,
                                 const Core::Ports::SlamOutput &out)
{
    const int trackedMapPointMin =
        EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_TRACKED_MPS", 96, 1, 100000);
    const int trustFrontendOkStreak = EnvIntValueClamped(
        "SMART_DRONE_SP_LG_TRUST_FRONTEND_PAIRS_OK_STREAK", 0, 0, 100000);
    const int bootstrapTrustHoldTrackedMapMin =
        EnvIntValueClamped("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_HOLD_TRACKED_MPS",
                           trackedMapPointMin, 1, 100000);
    const int trackedMapPoints = static_cast<int>(out.trackedMapPointCount);
    const bool bootstrapTrustAlreadyMature =
        trustFrontendOkStreak > 0 &&
        state.m_visualFeatureLightGlueOkStreak >= trustFrontendOkStreak;
    const bool holdMatureBootstrapTrust =
        bootstrapTrustAlreadyMature &&
        trackedMapPoints >= bootstrapTrustHoldTrackedMapMin;
    if (out.trackingState == Core::Ports::kSlamTrackingOk &&
        (trackedMapPoints >= trackedMapPointMin || holdMatureBootstrapTrust)) {
        ++state.m_visualFeatureLightGlueOkStreak;
    } else {
        state.m_visualFeatureLightGlueOkStreak = 0;
    }
}

void UpdateBootstrapTrustState(SlamModeSharedState &state,
                               bool usedBootstrapTrust,
                               int trustFrontendOkStreak)
{
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_ONCE", false)) {
        return;
    }
    if (trustFrontendOkStreak <= 0) {
        state.m_visualFeatureLightGlueBootstrapTrustClosed = true;
        return;
    }
    if (usedBootstrapTrust) {
        ++state.m_visualFeatureLightGlueBootstrapTrustFrames;
    }
    if (state.m_visualFeatureLightGlueOkStreak >= trustFrontendOkStreak ||
        state.m_visualFeatureLightGlueBootstrapTrustFrames >=
            trustFrontendOkStreak) {
        state.m_visualFeatureLightGlueBootstrapTrustClosed = true;
    }
}

bool PreviousFrameWasWeak(const SlamModeSharedState &state)
{
    const int minInliers = EnvIntValueClamped(
        "SMART_DRONE_SP_LG_WEAK_FRAME_MIN_INLIERS", 90, 1, 100000);
    const int minTrackedMap = EnvIntValueClamped(
        "SMART_DRONE_SP_LG_WEAK_FRAME_MIN_TRACKED_MAP", 120, 1, 100000);
    return state.m_lastSlamMatchesInliers > 0 &&
           (state.m_lastSlamMatchesInliers < minInliers ||
            state.m_lastSlamTrackedMapPoints < minTrackedMap);
}

size_t RefineSelectedRightPointsByZncc(const cv::Mat &leftPrepared,
                                       const cv::Mat &rightPrepared,
                                       std::vector<cv::Point2f> &leftPoints,
                                       std::vector<cv::Point2f> &rightPoints)
{
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_REFINE_RIGHT_ZNCC", false) ||
        leftPrepared.empty() || rightPrepared.empty() || leftPoints.empty() ||
        leftPoints.size() != rightPoints.size()) {
        return 0;
    }

    cv::Mat left32f;
    cv::Mat right32f;
    leftPrepared.convertTo(left32f, CV_32F);
    rightPrepared.convertTo(right32f, CV_32F);

    const float maxShiftPx = EnvFloatValueClamped(
        "SMART_DRONE_SP_LG_REFINE_RIGHT_ZNCC_MAX_SHIFT_PX", 4.0f, 0.25f, 16.0f);
    size_t refinedCount = 0;
    for (size_t i = 0; i < leftPoints.size(); ++i) {
        cv::Point2f refinedRight = rightPoints[i];
        float zncc = -1.0f;
        if (!RefineRightPointByStereoZncc(left32f, leftPoints[i], right32f,
                                          rightPoints[i], refinedRight, zncc)) {
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

void StoreTemporalCarrySource(
    SlamModeSharedState &state, const cv::Mat &leftPrepared,
    const cv::Mat &rightPrepared,
    const Core::Ports::StereoFeatureObservationPacket &stereoData)
{
    Core::Ports::TemporalStereoSourceInput sourceInput;
    sourceInput.leftPrepared = &leftPrepared;
    sourceInput.rightPrepared = &rightPrepared;
    sourceInput.observations = &stereoData;
    Core::Ports::TemporalStereoSource source;
    if (!state.TemporalStereoProcessor().ExtractSource(sourceInput, source)) {
        state.m_visualFeatureTemporalHavePrevStereo = false;
        state.m_visualFeatureTemporalPrevLeftPoints.clear();
        state.m_visualFeatureTemporalPrevRightPoints.clear();
        return;
    }

    state.m_visualFeatureTemporalPrevLeft = std::move(source.prevLeft);
    state.m_visualFeatureTemporalPrevRight = std::move(source.prevRight);
    state.m_visualFeatureTemporalPrevLeftPoints =
        std::move(source.prevLeftPoints);
    state.m_visualFeatureTemporalPrevRightPoints =
        std::move(source.prevRightPoints);
    state.m_visualFeatureTemporalHavePrevStereo = true;
}

} // namespace

VisualFeatureLightGlueModeStrategy::VisualFeatureLightGlueModeStrategy(
    FeatureFrontend frontend)
    : m_frontend(frontend)
{
}

FeatureFrontend VisualFeatureLightGlueModeStrategy::Frontend() const
{
    return m_frontend;
}

Core::Ports::SlamOutput VisualFeatureLightGlueModeStrategy::Process(
    SlamEngineAdapter &engine, const Core::Ports::SlamInputBatch &input,
    bool extractFeatures, bool extractPointCloud)
{
    Core::Ports::ISlamTrackingBackend *backend =
        SlamEngineAccess::TrackingBackend(engine);
    if (backend == nullptr || !backend->Available()) {
        return {};
    }
    Core::Ports::ISlamTrackingStatusProvider *trackingStatus =
        SlamEngineAccess::TrackingStatus(engine);
    Core::Ports::ISlamDescriptorProviderSource *descriptorProviders =
        SlamEngineAccess::DescriptorProviders(engine);
    if (trackingStatus == nullptr || descriptorProviders == nullptr) {
        return {};
    }

    SlamModeSharedState &state = SlamEngineAccess::ModeState(engine);
    const bool monoMode =
        SlamEngineAccess::InputMode(engine) != SlamInputMode::Stereo;
    if (monoMode || state.m_visualFeatureFrontend == nullptr ||
        !state.m_visualFeatureFrontend->Running()) {
        return MakeVisualFeatureFailureOutput(engine, state, input);
    }

    const int lightGlueEveryN = ChooseLightGlueCadence(state, *trackingStatus);
    state.m_visualFeatureFrontend->SetLightGlueEveryNOverride(lightGlueEveryN);
    state.m_visualFeatureLightGlueLastEveryN = lightGlueEveryN;
    state.ResetVisualFeatureStats();

    const auto externalStartTp = std::chrono::steady_clock::now();
    const auto prepareStartTp = externalStartTp;
    cv::Mat leftPrepared = EnsureGray8(input.stereo.left.gray);
    cv::Mat rightPrepared = EnsureGray8(input.stereo.right.gray);
    if (leftPrepared.empty() || rightPrepared.empty()) {
        return MakeVisualFeatureFailureOutput(engine, state, input);
    }

    if (EnvFlagEnabled("SMART_DRONE_SP_LG_USE_ORB_PREPARED_IMAGES", false)) {
        Core::Ports::StereoPreprocessRequest preprocessRequest;
        preprocessRequest.left = &input.stereo.left.gray;
        preprocessRequest.right = &input.stereo.right.gray;
        Core::Ports::StereoPreprocessResult preprocessResult;
        if (!backend->PrepareStereoImagesForTracking(preprocessRequest,
                                                     preprocessResult)) {
            return MakeVisualFeatureFailureOutput(engine, state, input);
        }
        leftPrepared = std::move(preprocessResult.leftPrepared);
        rightPrepared = std::move(preprocessResult.rightPrepared);
        leftPrepared = EnsureGray8(leftPrepared);
        rightPrepared = EnsureGray8(rightPrepared);
        if (leftPrepared.empty() || rightPrepared.empty()) {
            return MakeVisualFeatureFailureOutput(engine, state, input);
        }
    } else if (state.m_lkCalibrationLoaded) {
        cv::Mat leftRect;
        cv::Mat rightRect;
        if (state.PrepareRectifiedStereoCpu(leftPrepared, rightPrepared, leftRect,
                                            rightRect)) {
            leftPrepared = std::move(leftRect);
            rightPrepared = std::move(rightRect);
        }
    }

    const auto prepareEndTp = std::chrono::steady_clock::now();
    StereoFeatureFrontendRunInput frontendInput;
    frontendInput.client = state.m_visualFeatureFrontend;
    frontendInput.leftPrepared = &leftPrepared;
    frontendInput.rightPrepared = &rightPrepared;
    frontendInput.inputMaxWidth = state.m_visualFeatureInputMaxWidth;
    frontendInput.inputMaxHeight = state.m_visualFeatureInputMaxHeight;
    StereoFeatureFrontendRunResult frontendResult;
    if (!RunStereoFeatureFrontend(frontendInput, frontendResult)) {
        std::cerr << "[" << ToFeatureFrontendText(m_frontend)
                  << "] frontend_failed frame_id=" << input.frameId
                  << " err=" << frontendResult.error << "\n";
        return MakeVisualFeatureFailureOutput(engine, state, input);
    }

    Core::Ports::VisualFeatureSet leftFeatures =
        std::move(frontendResult.leftFeatures);
    Core::Ports::VisualFeatureSet rightFeatures =
        std::move(frontendResult.rightFeatures);
    const Core::Ports::IVisualFeatureFrontend::Stats &stats =
        frontendResult.stats;
    state.m_lastVisualFeaturePrepareMs = stats.prepareMs;
    state.m_lastVisualFeatureInputMs = stats.inputMs;
    state.m_lastVisualFeatureForwardMs = stats.forwardMs;
    state.m_lastVisualFeatureFrontendMs = stats.totalMs;
    state.m_lastVisualFeatureImageCount = stats.imageCount;
    state.m_lastVisualFeaturePayloadBytes = stats.payloadBytes;
    state.m_lastVisualFeatureRawLeftCount = stats.rawLeftCount;
    state.m_lastVisualFeatureRawRightCount = stats.rawRightCount;

    const auto matchStartTp = std::chrono::steady_clock::now();
    const bool initializingStereoFeature =
        trackingStatus->IsTrackingInitializing();
    const bool recoveringStereoFeature = trackingStatus->IsTrackingRecovering();
    const bool trustFrontendInitPairs =
        initializingStereoFeature &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_INIT_TRUST_FRONTEND_PAIRS", false);
    const bool trustFrontendRecoveryPairs =
        recoveringStereoFeature &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_RECOVERY_TRUST_FRONTEND_PAIRS", false);
    const int trustFrontendOkStreak = EnvIntValueClamped(
        "SMART_DRONE_SP_LG_TRUST_FRONTEND_PAIRS_OK_STREAK", 0, 0, 100000);
    const bool trustFrontendBootstrapPairs =
        trustFrontendOkStreak > 0 &&
        (!EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_ONCE", false) ||
         !state.m_visualFeatureLightGlueBootstrapTrustClosed) &&
        state.m_visualFeatureLightGlueOkStreak < trustFrontendOkStreak &&
        (!EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_ONCE", false) ||
         state.m_visualFeatureLightGlueBootstrapTrustFrames <
             trustFrontendOkStreak) &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_FRONTEND_PAIRS", false);

    Core::Ports::StereoMatchSelectionInput matchInput;
    matchInput.leftFeatures = &leftFeatures;
    matchInput.rightFeatures = &rightFeatures;
    matchInput.leftPrepared = &leftPrepared;
    matchInput.rightPrepared = &rightPrepared;
    matchInput.pairBuilder = &state.StereoPairBuilder();
    matchInput.initializing = initializingStereoFeature;
    matchInput.recovering = recoveringStereoFeature;
    matchInput.trustFrontendInitPairs = trustFrontendInitPairs;
    matchInput.trustFrontendRecoveryPairs = trustFrontendRecoveryPairs;
    matchInput.trustFrontendBootstrapPairs = trustFrontendBootstrapPairs;
    matchInput.previousFrameWeak = PreviousFrameWasWeak(state);

    Core::Ports::StereoMatchSelection matchSelection;
    if (!state.StereoMatchSelector().SelectMatches(matchInput, matchSelection)) {
        return MakeVisualFeatureFailureOutput(engine, state, input);
    }
    std::vector<cv::Point2f> matchedLeftPoints =
        std::move(matchSelection.matchedLeftPoints);
    std::vector<cv::Point2f> matchedRightPoints =
        std::move(matchSelection.matchedRightPoints);
    const size_t znccRefinedPairs = RefineSelectedRightPointsByZncc(
        leftPrepared, rightPrepared, matchedLeftPoints, matchedRightPoints);
    TemporalStereoStateView temporalState;
    temporalState.havePrevStereo = state.m_visualFeatureTemporalHavePrevStereo;
    temporalState.prevLeft = &state.m_visualFeatureTemporalPrevLeft;
    temporalState.prevRight = &state.m_visualFeatureTemporalPrevRight;
    temporalState.prevLeftPoints = &state.m_visualFeatureTemporalPrevLeftPoints;
    temporalState.prevRightPoints = &state.m_visualFeatureTemporalPrevRightPoints;
    temporalState.previousFrameWeak = matchInput.previousFrameWeak;
    TemporalStereoCarryInput temporalInput;
    temporalInput.state = &temporalState;
    temporalInput.leftPrepared = &leftPrepared;
    temporalInput.rightPrepared = &rightPrepared;
    temporalInput.pointTracker = &state.PointTracker2d();
    temporalInput.initializing = initializingStereoFeature;
    temporalInput.recovering = recoveringStereoFeature;
    Core::Ports::TemporalStereoCarryResult temporalResult;
    if (!state.TemporalStereoProcessor().AppendCarry(
            temporalInput, matchedLeftPoints, matchedRightPoints,
            temporalResult)) {
        return MakeVisualFeatureFailureOutput(engine, state, input);
    }
    const size_t temporalCarryPairs = temporalResult.insertedPairCount;
    const double stereoPairMs =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - matchStartTp)
            .count();
    state.m_lastVisualFeatureStereoMatchMs = stereoPairMs;
    state.m_lastVisualFeatureMatchedStereoCount =
        static_cast<int>(matchedLeftPoints.size());

    const auto packStartTp = std::chrono::steady_clock::now();
    Core::Ports::StereoFeaturePacketBuildInput packetInput;
    packetInput.leftPrepared = &leftPrepared;
    packetInput.rightPrepared = &rightPrepared;
    packetInput.matchedLeftPoints = &matchedLeftPoints;
    packetInput.matchedRightPoints = &matchedRightPoints;
    packetInput.filteredMatches = &matchSelection.filteredMatches;
    packetInput.rawMatches = &matchSelection.rawMatches;
    packetInput.leftFeatures = &leftFeatures;
    packetInput.rightFeatures = &rightFeatures;
    packetInput.leftDescriptorProvider =
        descriptorProviders->LeftDescriptorProvider();
    packetInput.rightDescriptorProvider =
        descriptorProviders->RightDescriptorProvider();
    packetInput.initializedForMonoAugmentation =
        trackingStatus->HasTrackingInitialized();
    packetInput.stableOkStreak = state.m_visualFeatureLightGlueOkStreak;

    Core::Ports::StereoFeaturePacket packet;
    Core::Ports::IStereoFeaturePacketBuilder &packetBuilder =
        state.StereoFeaturePacketBuilder();
    if (!packetBuilder.BuildPacket(packetInput, packet)) {
        CopyMatchedStereoPointsFromPairs(leftFeatures, rightFeatures,
                                         matchSelection.filteredMatches,
                                         matchedLeftPoints, matchedRightPoints);
        state.m_lastVisualFeatureMatchedStereoCount =
            static_cast<int>(matchedLeftPoints.size());
        packetInput.matchedLeftPoints = &matchedLeftPoints;
        packetInput.matchedRightPoints = &matchedRightPoints;
        packetInput.allowNativeDescriptorInject = false;
        packetInput.allowAllLeftGeometricDepth = false;
        if (!packetBuilder.BuildPacket(packetInput, packet)) {
            return MakeVisualFeatureFailureOutput(engine, state, input);
        }
    }

    const auto packEndTp = std::chrono::steady_clock::now();
    state.m_lastVisualFeatureInjectedLeftCount =
        static_cast<int>(packet.observations.leftKeypoints.size());
    state.m_lastVisualFeatureInjectedRightCount =
        static_cast<int>(packet.observations.rightKeypoints.size());
    state.m_lastVisualFeatureObservationHash = packet.hash;
    if (EnvFlagEnabled("SMART_DRONE_SP_LG_INJECT_DFX", false)) {
        std::cerr << "[sp_lg_inject_dfx] frame_id=" << input.frameId
                  << " initializing=" << (initializingStereoFeature ? "Y" : "N")
                  << " recovering=" << (recoveringStereoFeature ? "Y" : "N")
                  << " trust_frontend_pairs="
                  << (matchSelection.trustFrontendPairs ? "Y" : "N")
                  << " init_trust_selected="
                  << (matchSelection.initializationTrustedPairSelection ? "Y" : "N")
                  << " bootstrap_trust="
                  << (trustFrontendBootstrapPairs ? "Y" : "N")
                  << " bootstrap_closed="
                  << (state.m_visualFeatureLightGlueBootstrapTrustClosed ? "Y"
                                                                         : "N")
                  << " bootstrap_frames="
                  << state.m_visualFeatureLightGlueBootstrapTrustFrames
                  << " ok_streak=" << state.m_visualFeatureLightGlueOkStreak
                  << " prev_inliers=" << state.m_lastSlamMatchesInliers
                  << " prev_tracked=" << state.m_lastSlamTrackedMapPoints
                  << " frontend_pairs=" << matchSelection.pairedFeatureCount
                  << " init_trust_matches="
                  << matchSelection.initializationTrustedMatches.size()
                  << " raw_matches=" << matchSelection.rawMatches.size()
                  << " filtered_matches=" << matchSelection.filteredMatches.size()
                  << " selected_pairs=" << matchedLeftPoints.size()
                  << " zncc_refined=" << znccRefinedPairs
                  << " temporal_carry=" << temporalCarryPairs
                  << " orb_stereo_aug=" << packet.orbStereoAugmentPairs
                  << " injected=" << packet.observations.leftKeypoints.size() << "/"
                  << packet.observations.rightKeypoints.size()
                  << " hash=" << state.m_lastVisualFeatureObservationHash
                  << " scale=" << frontendResult.leftScaleX << "x"
                  << frontendResult.leftScaleY << "\n";
    }

    StereoFeatureTrackRequest request;
    request.enabled = true;
    request.recordTotalMs = true;
    request.totalStartTp = externalStartTp;
    request.leftPrepared = std::move(leftPrepared);
    request.rightPrepared = std::move(rightPrepared);
    request.observations = std::move(packet.observations);
    request.inputPrepareMs =
        std::chrono::duration<double, std::milli>(prepareEndTp - prepareStartTp)
            .count() +
        frontendResult.inputBuildMs;
    request.frontendMs = frontendResult.frontendCallMs;
    request.stereoPairMs = stereoPairMs;
    request.featurePackMs =
        std::chrono::duration<double, std::milli>(packEndTp - packStartTp)
            .count();
    request.monoAugmentMs = packet.monoAugmentMs;
    request.observationHash = state.m_lastVisualFeatureObservationHash;
    request.leftFeaturePoints = std::move(packet.leftFeaturePoints);
    request.rightFeaturePoints = std::move(packet.rightFeaturePoints);
    StoreTemporalCarrySource(state, request.leftPrepared, request.rightPrepared,
                             request.observations);

    Core::Ports::SlamOutput out = RunSlamTrackingBackend(
        engine, input, extractFeatures, extractPointCloud, &request);
    UpdateLightGlueCadenceState(state, out);
    UpdateBootstrapTrustState(
        state, trustFrontendBootstrapPairs && matchSelection.trustFrontendPairs,
        trustFrontendOkStreak);
    state.m_lastSlamMatchesInliers = out.matchesInliers;
    state.m_lastSlamTrackedMapPoints = static_cast<int>(out.trackedMapPointCount);
    return out;
}

std::unique_ptr<SlamModeStrategy> CreateSuperPointLightGlueModeStrategy()
{
    return std::make_unique<VisualFeatureLightGlueModeStrategy>(
        FeatureFrontend::SuperPointLightGlue);
}

std::unique_ptr<SlamModeStrategy> CreateXFeatLightGlueModeStrategy()
{
    return std::make_unique<VisualFeatureLightGlueModeStrategy>(
        FeatureFrontend::XFeatLightGlue);
}

} // namespace SmartDrone::Adapters::Slam
