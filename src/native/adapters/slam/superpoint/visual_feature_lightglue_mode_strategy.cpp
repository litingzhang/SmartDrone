#include "adapters/slam/engine/slam_mode_strategy.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "adapters/slam/engine/slam_engine_access.h"
#include "adapters/slam/engine/slam_env.h"
#include "adapters/slam/engine/slam_image_utils.h"
#include "adapters/slam/engine/slam_output_utils.h"
#include "adapters/slam/engine/slam_tracking_backend.h"
#include "adapters/slam/stereo/stereo_feature_frontend_runner.h"
#include "adapters/slam/stereo/stereo_feature_packet.h"
#include "adapters/slam/stereo/stereo_geometry.h"
#include "adapters/slam/stereo/stereo_matching.h"
#include "adapters/slam/stereo/temporal_stereo.h"
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

const char *VisualFeatureFrontendName(FeatureFrontend frontend)
{
    if (frontend == FeatureFrontend::XFeatLightGlue) {
        return "xfeat_lightglue";
    }
    return "superpoint_lightglue";
}

const SlamModeStrategyRegistrar
    SUPER_POINT_LIGHT_GLUE_MODE_STRATEGY_REGISTRATION(
        FeatureFrontend::SuperPointLightGlue,
        &CreateRegisteredSuperPointLightGlueModeStrategy);

const SlamModeStrategyRegistrar XFEAT_LIGHT_GLUE_MODE_STRATEGY_REGISTRATION(
    FeatureFrontend::XFeatLightGlue,
    &CreateRegisteredXFeatLightGlueModeStrategy);

Core::Ports::SlamOutput
MakeVisualFeatureFailureOutput(SlamEngineAdapter &engine,
                               SlamModeSharedState &state,
                               const Core::Ports::SlamInputBatch &input)
{
    Core::Ports::SlamOutput out = MakePoseLostSlamOutput(
        &engine, input, Core::Ports::SLAM_TRACKING_RECENTLY_LOST, true, true);
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
    if (trackingState == Core::Ports::SLAM_TRACKING_OK &&
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
    if (out.trackingState == Core::Ports::SLAM_TRACKING_OK &&
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
        if (!RefineRightPointByStereoZncc(
                {left32f, leftPoints[i], right32f, rightPoints[i],
                 refinedRight, zncc})) {
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

struct LightGlueProcessContext {
    SlamEngineAdapter *engine{nullptr};
    const Core::Ports::SlamInputBatch *input{nullptr};
    Core::Ports::ISlamTrackingBackend *backend{nullptr};
    Core::Ports::ISlamTrackingStatusProvider *trackingStatus{nullptr};
    Core::Ports::ISlamDescriptorProviderSource *descriptorProviders{nullptr};
    SlamModeSharedState *state{nullptr};
    FeatureFrontend frontend{FeatureFrontend::SuperPointLightGlue};
    bool extractFeatures{false};
    bool extractPointCloud{false};
};

struct LightGluePreparedFrame {
    std::chrono::steady_clock::time_point externalStartTp{};
    std::chrono::steady_clock::time_point prepareStartTp{};
    std::chrono::steady_clock::time_point prepareEndTp{};
    cv::Mat leftPrepared;
    cv::Mat rightPrepared;
    Core::Ports::VisualFeatureSet leftFeatures;
    Core::Ports::VisualFeatureSet rightFeatures;
    StereoFeatureFrontendRunResult frontendResult;
};

struct LightGlueTrustConfig {
    bool initializingStereoFeature{false};
    bool recoveringStereoFeature{false};
    bool trustFrontendInitPairs{false};
    bool trustFrontendRecoveryPairs{false};
    bool trustFrontendBootstrapPairs{false};
    int trustFrontendOkStreak{0};
};

struct LightGlueStereoPairState {
    LightGlueTrustConfig trust;
    Core::Ports::StereoMatchSelection matchSelection;
    std::vector<cv::Point2f> matchedLeftPoints;
    std::vector<cv::Point2f> matchedRightPoints;
    size_t znccRefinedPairs{0};
    size_t temporalCarryPairs{0};
    double stereoPairMs{0.0};
};

struct LightGluePacketState {
    Core::Ports::StereoFeaturePacket packet;
    double featurePackMs{0.0};
};

bool ResolveLightGlueDependencies(LightGlueProcessContext &context)
{
    context.backend = SlamEngineAccess::TrackingBackend(*context.engine);
    if (context.backend == nullptr || !context.backend->Available()) {
        return false;
    }
    context.trackingStatus = SlamEngineAccess::TrackingStatus(*context.engine);
    context.descriptorProviders =
        SlamEngineAccess::DescriptorProviders(*context.engine);
    if (context.trackingStatus == nullptr ||
        context.descriptorProviders == nullptr) {
        return false;
    }
    context.state = &SlamEngineAccess::ModeState(*context.engine);
    return true;
}

bool IsLightGlueRuntimeAvailable(const LightGlueProcessContext &context)
{
    const bool monoMode =
        SlamEngineAccess::InputMode(*context.engine) != SlamInputMode::Stereo;
    return !monoMode && context.state->m_visualFeatureFrontend != nullptr &&
           context.state->m_visualFeatureFrontend->Running();
}

Core::Ports::SlamOutput MakeLightGlueFailureOutput(
    const LightGlueProcessContext &context)
{
    return MakeVisualFeatureFailureOutput(*context.engine, *context.state,
                                          *context.input);
}

void ConfigureLightGlueCadence(const LightGlueProcessContext &context)
{
    const int lightGlueEveryN =
        ChooseLightGlueCadence(*context.state, *context.trackingStatus);
    context.state->m_visualFeatureFrontend->SetLightGlueEveryNOverride(
        lightGlueEveryN);
    context.state->m_visualFeatureLightGlueLastEveryN = lightGlueEveryN;
    context.state->ResetVisualFeatureStats();
}

bool PrepareWithBackendImages(const LightGlueProcessContext &context,
                              LightGluePreparedFrame &frame)
{
    Core::Ports::StereoPreprocessRequest preprocessRequest;
    preprocessRequest.left = &context.input->stereo.left.gray;
    preprocessRequest.right = &context.input->stereo.right.gray;
    Core::Ports::StereoPreprocessResult preprocessResult;
    if (!context.backend->PrepareStereoImagesForTracking(preprocessRequest,
                                                         preprocessResult)) {
        return false;
    }
    frame.leftPrepared = EnsureGray8(preprocessResult.leftPrepared);
    frame.rightPrepared = EnsureGray8(preprocessResult.rightPrepared);
    return !frame.leftPrepared.empty() && !frame.rightPrepared.empty();
}

void TryPrepareRectifiedCpu(const LightGlueProcessContext &context,
                            LightGluePreparedFrame &frame)
{
    if (!context.state->m_lkCalibrationLoaded) {
        return;
    }
    cv::Mat leftRect;
    cv::Mat rightRect;
    if (context.state->PrepareRectifiedStereoCpu(frame.leftPrepared,
                                                 frame.rightPrepared, leftRect,
                                                 rightRect)) {
        frame.leftPrepared = std::move(leftRect);
        frame.rightPrepared = std::move(rightRect);
    }
}

bool PrepareLightGlueFrame(const LightGlueProcessContext &context,
                           LightGluePreparedFrame &frame)
{
    frame.externalStartTp = std::chrono::steady_clock::now();
    frame.prepareStartTp = frame.externalStartTp;
    frame.leftPrepared = EnsureGray8(context.input->stereo.left.gray);
    frame.rightPrepared = EnsureGray8(context.input->stereo.right.gray);
    if (frame.leftPrepared.empty() || frame.rightPrepared.empty()) {
        return false;
    }
    if (EnvFlagEnabled("SMART_DRONE_SP_LG_USE_ORB_PREPARED_IMAGES", false)) {
        if (!PrepareWithBackendImages(context, frame)) {
            return false;
        }
    } else {
        TryPrepareRectifiedCpu(context, frame);
    }
    frame.prepareEndTp = std::chrono::steady_clock::now();
    return true;
}

void CopyFrontendStatsToState(const StereoFeatureFrontendRunResult &result,
                              SlamModeSharedState &state)
{
    const Core::Ports::IVisualFeatureFrontend::Stats &stats = result.stats;
    state.m_lastVisualFeaturePrepareMs = stats.prepareMs;
    state.m_lastVisualFeatureInputMs = stats.inputMs;
    state.m_lastVisualFeatureForwardMs = stats.forwardMs;
    state.m_lastVisualFeatureFrontendMs = stats.totalMs;
    state.m_lastVisualFeatureImageCount = stats.imageCount;
    state.m_lastVisualFeaturePayloadBytes = stats.payloadBytes;
    state.m_lastVisualFeatureRawLeftCount = stats.rawLeftCount;
    state.m_lastVisualFeatureRawRightCount = stats.rawRightCount;
}

bool RunLightGlueFrontend(const LightGlueProcessContext &context,
                          LightGluePreparedFrame &frame)
{
    StereoFeatureFrontendRunInput frontendInput;
    frontendInput.client = context.state->m_visualFeatureFrontend;
    frontendInput.leftPrepared = &frame.leftPrepared;
    frontendInput.rightPrepared = &frame.rightPrepared;
    frontendInput.inputMaxWidth = context.state->m_visualFeatureInputMaxWidth;
    frontendInput.inputMaxHeight = context.state->m_visualFeatureInputMaxHeight;
    if (!RunStereoFeatureFrontend(frontendInput, frame.frontendResult)) {
        std::cerr << "[" << VisualFeatureFrontendName(context.frontend)
                  << "] frontend_failed frame_id=" << context.input->frameId
                  << " err=" << frame.frontendResult.error << "\n";
        return false;
    }
    frame.leftFeatures = std::move(frame.frontendResult.leftFeatures);
    frame.rightFeatures = std::move(frame.frontendResult.rightFeatures);
    CopyFrontendStatsToState(frame.frontendResult, *context.state);
    return true;
}

LightGlueTrustConfig BuildLightGlueTrustConfig(
    const LightGlueProcessContext &context)
{
    LightGlueTrustConfig trust;
    trust.initializingStereoFeature =
        context.trackingStatus->IsTrackingInitializing();
    trust.recoveringStereoFeature =
        context.trackingStatus->IsTrackingRecovering();
    trust.trustFrontendInitPairs =
        trust.initializingStereoFeature &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_INIT_TRUST_FRONTEND_PAIRS", false);
    trust.trustFrontendRecoveryPairs =
        trust.recoveringStereoFeature &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_RECOVERY_TRUST_FRONTEND_PAIRS", false);
    trust.trustFrontendOkStreak = EnvIntValueClamped(
        "SMART_DRONE_SP_LG_TRUST_FRONTEND_PAIRS_OK_STREAK", 0, 0, 100000);
    trust.trustFrontendBootstrapPairs =
        trust.trustFrontendOkStreak > 0 &&
        (!EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_ONCE", false) ||
         !context.state->m_visualFeatureLightGlueBootstrapTrustClosed) &&
        context.state->m_visualFeatureLightGlueOkStreak <
            trust.trustFrontendOkStreak &&
        (!EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_ONCE", false) ||
         context.state->m_visualFeatureLightGlueBootstrapTrustFrames <
             trust.trustFrontendOkStreak) &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_FRONTEND_PAIRS",
                       false);
    return trust;
}

Core::Ports::StereoMatchSelectionInput BuildLightGlueMatchInput(
    const LightGlueProcessContext &context, const LightGluePreparedFrame &frame,
    const LightGlueTrustConfig &trust)
{
    Core::Ports::StereoMatchSelectionInput input;
    input.leftFeatures = &frame.leftFeatures;
    input.rightFeatures = &frame.rightFeatures;
    input.leftPrepared = &frame.leftPrepared;
    input.rightPrepared = &frame.rightPrepared;
    input.pairBuilder = &context.state->StereoPairBuilder();
    input.initializing = trust.initializingStereoFeature;
    input.recovering = trust.recoveringStereoFeature;
    input.trustFrontendInitPairs = trust.trustFrontendInitPairs;
    input.trustFrontendRecoveryPairs = trust.trustFrontendRecoveryPairs;
    input.trustFrontendBootstrapPairs = trust.trustFrontendBootstrapPairs;
    input.previousFrameWeak = PreviousFrameWasWeak(*context.state);
    return input;
}

TemporalStereoCarryInput BuildTemporalCarryInput(
    const LightGlueProcessContext &context, const LightGluePreparedFrame &frame,
    const Core::Ports::StereoMatchSelectionInput &matchInput,
    TemporalStereoStateView &temporalState)
{
    temporalState.havePrevStereo =
        context.state->m_visualFeatureTemporalHavePrevStereo;
    temporalState.prevLeft = &context.state->m_visualFeatureTemporalPrevLeft;
    temporalState.prevRight = &context.state->m_visualFeatureTemporalPrevRight;
    temporalState.prevLeftPoints =
        &context.state->m_visualFeatureTemporalPrevLeftPoints;
    temporalState.prevRightPoints =
        &context.state->m_visualFeatureTemporalPrevRightPoints;
    temporalState.previousFrameWeak = matchInput.previousFrameWeak;

    TemporalStereoCarryInput temporalInput;
    temporalInput.state = &temporalState;
    temporalInput.leftPrepared = &frame.leftPrepared;
    temporalInput.rightPrepared = &frame.rightPrepared;
    temporalInput.pointTracker = &context.state->PointTracker2d();
    temporalInput.initializing = matchInput.initializing;
    temporalInput.recovering = matchInput.recovering;
    return temporalInput;
}

bool SelectLightGlueStereoPairs(const LightGlueProcessContext &context,
                                const LightGluePreparedFrame &frame,
                                LightGlueStereoPairState &pairState)
{
    const auto matchStartTp = std::chrono::steady_clock::now();
    pairState.trust = BuildLightGlueTrustConfig(context);
    Core::Ports::StereoMatchSelectionInput matchInput =
        BuildLightGlueMatchInput(context, frame, pairState.trust);
    if (!context.state->StereoMatchSelector().SelectMatches(
            matchInput, pairState.matchSelection)) {
        return false;
    }
    pairState.matchedLeftPoints =
        std::move(pairState.matchSelection.matchedLeftPoints);
    pairState.matchedRightPoints =
        std::move(pairState.matchSelection.matchedRightPoints);
    pairState.znccRefinedPairs = RefineSelectedRightPointsByZncc(
        frame.leftPrepared, frame.rightPrepared, pairState.matchedLeftPoints,
        pairState.matchedRightPoints);
    TemporalStereoStateView temporalState;
    TemporalStereoCarryInput temporalInput =
        BuildTemporalCarryInput(context, frame, matchInput, temporalState);
    Core::Ports::TemporalStereoCarryResult temporalResult;
    if (!context.state->TemporalStereoProcessor().AppendCarry(
            temporalInput, pairState.matchedLeftPoints,
            pairState.matchedRightPoints, temporalResult)) {
        return false;
    }
    pairState.temporalCarryPairs = temporalResult.insertedPairCount;
    pairState.stereoPairMs = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() -
                                 matchStartTp)
                                 .count();
    context.state->m_lastVisualFeatureStereoMatchMs = pairState.stereoPairMs;
    context.state->m_lastVisualFeatureMatchedStereoCount =
        static_cast<int>(pairState.matchedLeftPoints.size());
    return true;
}

Core::Ports::StereoFeaturePacketBuildInput BuildFeaturePacketInput(
    const LightGlueProcessContext &context, const LightGluePreparedFrame &frame,
    const LightGlueStereoPairState &pairState)
{
    Core::Ports::StereoFeaturePacketBuildInput input;
    input.leftPrepared = &frame.leftPrepared;
    input.rightPrepared = &frame.rightPrepared;
    input.matchedLeftPoints = &pairState.matchedLeftPoints;
    input.matchedRightPoints = &pairState.matchedRightPoints;
    input.filteredMatches = &pairState.matchSelection.filteredMatches;
    input.rawMatches = &pairState.matchSelection.rawMatches;
    input.leftFeatures = &frame.leftFeatures;
    input.rightFeatures = &frame.rightFeatures;
    input.leftDescriptorProvider =
        context.descriptorProviders->LeftDescriptorProvider();
    input.rightDescriptorProvider =
        context.descriptorProviders->RightDescriptorProvider();
    input.initializedForMonoAugmentation =
        context.trackingStatus->HasTrackingInitialized();
    input.stableOkStreak = context.state->m_visualFeatureLightGlueOkStreak;
    return input;
}

bool BuildLightGluePacketWithFallback(
    const LightGlueProcessContext &context, LightGluePreparedFrame &frame,
    LightGlueStereoPairState &pairState,
    Core::Ports::StereoFeaturePacketBuildInput &packetInput,
    LightGluePacketState &packetState)
{
    Core::Ports::IStereoFeaturePacketBuilder &packetBuilder =
        context.state->StereoFeaturePacketBuilder();
    if (packetBuilder.BuildPacket(packetInput, packetState.packet)) {
        return true;
    }
    CopyMatchedStereoPointsFromPairs(frame.leftFeatures, frame.rightFeatures,
                                     pairState.matchSelection.filteredMatches,
                                     pairState.matchedLeftPoints,
                                     pairState.matchedRightPoints);
    context.state->m_lastVisualFeatureMatchedStereoCount =
        static_cast<int>(pairState.matchedLeftPoints.size());
    packetInput.matchedLeftPoints = &pairState.matchedLeftPoints;
    packetInput.matchedRightPoints = &pairState.matchedRightPoints;
    packetInput.allowNativeDescriptorInject = false;
    packetInput.allowAllLeftGeometricDepth = false;
    return packetBuilder.BuildPacket(packetInput, packetState.packet);
}

bool BuildLightGlueFeaturePacket(const LightGlueProcessContext &context,
                                 LightGluePreparedFrame &frame,
                                 LightGlueStereoPairState &pairState,
                                 LightGluePacketState &packetState)
{
    const auto packStartTp = std::chrono::steady_clock::now();
    Core::Ports::StereoFeaturePacketBuildInput packetInput =
        BuildFeaturePacketInput(context, frame, pairState);
    if (!BuildLightGluePacketWithFallback(context, frame, pairState,
                                          packetInput, packetState)) {
        return false;
    }
    packetState.featurePackMs = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() -
                                    packStartTp)
                                    .count();
    context.state->m_lastVisualFeatureInjectedLeftCount =
        static_cast<int>(packetState.packet.observations.leftKeypoints.size());
    context.state->m_lastVisualFeatureInjectedRightCount =
        static_cast<int>(packetState.packet.observations.rightKeypoints.size());
    context.state->m_lastVisualFeatureObservationHash = packetState.packet.hash;
    return true;
}

void LogLightGlueInjectDfx(const LightGlueProcessContext &context,
                           const LightGluePreparedFrame &frame,
                           const LightGlueStereoPairState &pairState,
                           const LightGluePacketState &packetState)
{
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_INJECT_DFX", false)) {
        return;
    }
    const auto &trust = pairState.trust;
    const auto &selection = pairState.matchSelection;
    std::cerr << "[sp_lg_inject_dfx] frame_id=" << context.input->frameId
              << " initializing=" << (trust.initializingStereoFeature ? "Y" : "N")
              << " recovering=" << (trust.recoveringStereoFeature ? "Y" : "N")
              << " trust_frontend_pairs="
              << (selection.trustFrontendPairs ? "Y" : "N")
              << " init_trust_selected="
              << (selection.initializationTrustedPairSelection ? "Y" : "N")
              << " bootstrap_trust="
              << (trust.trustFrontendBootstrapPairs ? "Y" : "N")
              << " bootstrap_closed="
              << (context.state->m_visualFeatureLightGlueBootstrapTrustClosed
                      ? "Y"
                      : "N")
              << " bootstrap_frames="
              << context.state->m_visualFeatureLightGlueBootstrapTrustFrames
              << " ok_streak="
              << context.state->m_visualFeatureLightGlueOkStreak
              << " prev_inliers=" << context.state->m_lastSlamMatchesInliers
              << " prev_tracked=" << context.state->m_lastSlamTrackedMapPoints
              << " frontend_pairs=" << selection.pairedFeatureCount
              << " init_trust_matches="
              << selection.initializationTrustedMatches.size()
              << " raw_matches=" << selection.rawMatches.size()
              << " filtered_matches=" << selection.filteredMatches.size()
              << " selected_pairs=" << pairState.matchedLeftPoints.size()
              << " zncc_refined=" << pairState.znccRefinedPairs
              << " temporal_carry=" << pairState.temporalCarryPairs
              << " orb_stereo_aug="
              << packetState.packet.orbStereoAugmentPairs
              << " injected="
              << packetState.packet.observations.leftKeypoints.size() << "/"
              << packetState.packet.observations.rightKeypoints.size()
              << " hash="
              << context.state->m_lastVisualFeatureObservationHash
              << " scale=" << frame.frontendResult.leftScaleX << "x"
              << frame.frontendResult.leftScaleY << "\n";
}

StereoFeatureTrackRequest BuildLightGlueTrackRequest(
    const LightGlueProcessContext &context, LightGluePreparedFrame &frame,
    LightGlueStereoPairState &pairState, LightGluePacketState &packetState)
{
    StereoFeatureTrackRequest request;
    request.enabled = true;
    request.recordTotalMs = true;
    request.totalStartTp = frame.externalStartTp;
    request.leftPrepared = std::move(frame.leftPrepared);
    request.rightPrepared = std::move(frame.rightPrepared);
    request.observations = std::move(packetState.packet.observations);
    request.inputPrepareMs =
        std::chrono::duration<double, std::milli>(frame.prepareEndTp -
                                                 frame.prepareStartTp)
            .count() +
        frame.frontendResult.inputBuildMs;
    request.frontendMs = frame.frontendResult.frontendCallMs;
    request.stereoPairMs = pairState.stereoPairMs;
    request.featurePackMs = packetState.featurePackMs;
    request.monoAugmentMs = packetState.packet.monoAugmentMs;
    request.observationHash =
        context.state->m_lastVisualFeatureObservationHash;
    request.leftFeaturePoints = std::move(packetState.packet.leftFeaturePoints);
    request.rightFeaturePoints =
        std::move(packetState.packet.rightFeaturePoints);
    return request;
}

Core::Ports::SlamOutput RunLightGlueTracking(
    const LightGlueProcessContext &context, StereoFeatureTrackRequest &request)
{
    StoreTemporalCarrySource(*context.state, request.leftPrepared,
                             request.rightPrepared, request.observations);
    return RunSlamTrackingBackend(*context.engine, *context.input,
                                  context.extractFeatures,
                                  context.extractPointCloud, &request);
}

void UpdateLightGluePostTrackState(const LightGlueProcessContext &context,
                                   const LightGlueStereoPairState &pairState,
                                   const Core::Ports::SlamOutput &out)
{
    UpdateLightGlueCadenceState(*context.state, out);
    UpdateBootstrapTrustState(
        *context.state,
        pairState.trust.trustFrontendBootstrapPairs &&
            pairState.matchSelection.trustFrontendPairs,
        pairState.trust.trustFrontendOkStreak);
    context.state->m_lastSlamMatchesInliers = out.matchesInliers;
    context.state->m_lastSlamTrackedMapPoints =
        static_cast<int>(out.trackedMapPointCount);
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
    LightGlueProcessContext context;
    context.engine = &engine;
    context.input = &input;
    context.frontend = m_frontend;
    context.extractFeatures = extractFeatures;
    context.extractPointCloud = extractPointCloud;
    if (!ResolveLightGlueDependencies(context)) {
        return {};
    }
    if (!IsLightGlueRuntimeAvailable(context)) {
        return MakeLightGlueFailureOutput(context);
    }

    ConfigureLightGlueCadence(context);
    LightGluePreparedFrame frame;
    if (!PrepareLightGlueFrame(context, frame) ||
        !RunLightGlueFrontend(context, frame)) {
        return MakeLightGlueFailureOutput(context);
    }

    LightGlueStereoPairState pairState;
    if (!SelectLightGlueStereoPairs(context, frame, pairState)) {
        return MakeLightGlueFailureOutput(context);
    }

    LightGluePacketState packetState;
    if (!BuildLightGlueFeaturePacket(context, frame, pairState, packetState)) {
        return MakeLightGlueFailureOutput(context);
    }

    LogLightGlueInjectDfx(context, frame, pairState, packetState);
    StereoFeatureTrackRequest request =
        BuildLightGlueTrackRequest(context, frame, pairState, packetState);
    Core::Ports::SlamOutput out = RunLightGlueTracking(context, request);
    UpdateLightGluePostTrackState(context, pairState, out);
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
