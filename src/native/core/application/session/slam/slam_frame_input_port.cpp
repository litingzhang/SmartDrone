#include "core/application/session/slam/slam_frame_input_port.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <utility>

#include "common/environment.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/session/slam/imu_window_filter.h"
#include "core/application/session/slam/slam_runtime_control_port.h"
#include "core/application/state/live_pose_state.h"

namespace SmartDrone::Core::Application {
namespace {

constexpr int64_t POINT_CLOUD_UPDATE_INTERVAL_NS = 200000000LL;

std::pair<int, int> ComputeVisualFeatureInputBudget(int baseWidth,
                                                    int baseHeight,
                                                    int loadSheddingLevel)
{
    const int safeBaseWidth = std::max(1, baseWidth);
    const int safeBaseHeight = std::max(1, baseHeight);
    float scale = 1.0f;
    if (loadSheddingLevel >= 2) {
        scale = 0.75f;
    } else if (loadSheddingLevel == 1) {
        scale = 0.85f;
    }

    const int width =
        std::max(320, static_cast<int>(std::lround(
                          static_cast<double>(safeBaseWidth) * scale)));
    const int height =
        std::max(240, static_cast<int>(std::lround(
                          static_cast<double>(safeBaseHeight) * scale)));
    return {std::min(width, safeBaseWidth), std::min(height, safeBaseHeight)};
}

const char *DescribeVisualFeatureLoadSheddingLevel(int loadSheddingLevel)
{
    switch (loadSheddingLevel) {
    case 2:
        return "aggressive";
    case 1:
        return "moderate";
    default:
        return "nominal";
    }
}

const char *FeatureFrontendName(FeatureFrontend frontend)
{
    switch (frontend) {
    case FeatureFrontend::LkGfttPerFrame:
        return "lk_gftt_per_frame";
    case FeatureFrontend::LK:
        return "lk";
    case FeatureFrontend::SuperPointLightGlue:
        return "superpoint_lightglue";
    case FeatureFrontend::XFeatLightGlue:
        return "xfeat_lightglue";
    case FeatureFrontend::Orb:
    default:
        return "orb";
    }
}

FeatureFrontend ParseRuntimeFeatureFrontendValue(uint8_t value)
{
    switch (value) {
    case static_cast<uint8_t>(FeatureFrontend::XFeatLightGlue):
        return FeatureFrontend::XFeatLightGlue;
    case static_cast<uint8_t>(FeatureFrontend::SuperPointLightGlue):
        return FeatureFrontend::SuperPointLightGlue;
    case static_cast<uint8_t>(FeatureFrontend::LkGfttPerFrame):
        return FeatureFrontend::LkGfttPerFrame;
    case static_cast<uint8_t>(FeatureFrontend::LK):
        return FeatureFrontend::LK;
    case static_cast<uint8_t>(FeatureFrontend::Orb):
        return FeatureFrontend::Orb;
    default:
        return FeatureFrontend::LkGfttPerFrame;
    }
}

} // namespace

SlamFrameInputPort::SlamFrameInputPort(SlamFrameProcessingContext &context,
                                       SlamFrameInputState &state,
                                       SlamFrameSharedState &sharedState,
                                       SlamFrameOutputState &outputState)
    : m_ctx(context),
      m_state(state),
      m_sharedState(sharedState),
      m_outputState(outputState)
{
}

SlamFrameStageResult SlamFrameInputPort::AcquireAndPrepareFrame(
    SlamPreparedFrameData &frame)
{
    frame = SlamPreparedFrameData{};
    frame.frameStartTp = std::chrono::steady_clock::now();
    SyncRequestedSlamMode();
    const RuntimeFrameConfig config = ApplyRuntimeFrameConfig();
    StereoAcquireResult acquire = AcquireStereoBatch(config);
    if (!acquire.hasFrame) {
        return {acquire.stepResult, acquire.sessionOk};
    }
    const FrameMetadata metadata =
        BuildFrameMetadata(acquire.stereoBatch, config);
    SlamInputPreparation input =
        PrepareSlamInput(acquire.stereoBatch, metadata);
    if (!input.ready) {
        return {input.stepResult, true};
    }
    FillPreparedFrame(frame, std::move(acquire), std::move(input), config,
                      metadata);
    return {SlamFrameStepResult::Continue, true};
}

void SlamFrameInputPort::SyncRequestedSlamMode()
{
    const auto configuredSlamMode =
        static_cast<SmartDrone::Core::Domain::SlamOperationMode>(
            m_ctx.tuning.slamOperationMode.load(std::memory_order_relaxed));
    if (configuredSlamMode == m_sharedState.requestedSlamMode.load()) {
        return;
    }

    m_sharedState.requestedSlamMode.store(configuredSlamMode);
    m_ctx.autoSlamModeController.Reset();
    const auto effectiveSlamMode =
        configuredSlamMode == SmartDrone::Core::Domain::SlamOperationMode::Auto
            ? SmartDrone::Core::Domain::SlamOperationMode::Mapping
            : configuredSlamMode;
    m_sharedState.effectiveSlamMode.store(effectiveSlamMode);
    if (m_ctx.slamControl != nullptr) {
        m_ctx.slamControl->SetOperationMode(effectiveSlamMode);
    }
    std::cerr << "[slam] operation_mode -> "
              << SmartDrone::Core::Domain::ToString(configuredSlamMode);
    if (configuredSlamMode ==
        SmartDrone::Core::Domain::SlamOperationMode::Auto) {
        std::cerr << " effective_mode="
                  << SmartDrone::Core::Domain::ToString(effectiveSlamMode);
    }
    std::cerr << "\n";
    if (configuredSlamMode ==
            SmartDrone::Core::Domain::SlamOperationMode::Relocalization ||
        configuredSlamMode ==
            SmartDrone::Core::Domain::SlamOperationMode::TrackingOnly) {
        std::cerr << "[slam] note: requested mode currently maps to backend "
                     "localization-only mode\n";
    }
    m_ctx.livePose.SetSlamMode(ToRuntimeSlamModeValue(effectiveSlamMode));
}

SlamFrameInputPort::RuntimeFrameConfig
SlamFrameInputPort::ApplyRuntimeFrameConfig()
{
    RuntimeFrameConfig config{};
    config.configuredSlamInputFps = m_ctx.perceptionPipeline.ClampTargetFps(
        m_ctx.tuning.slamInputFps.load(std::memory_order_relaxed));
    config.configuredFrontendValue =
        m_ctx.tuning.featureFrontend.load(std::memory_order_relaxed);
    config.effectiveFrontend =
        ParseRuntimeFeatureFrontendValue(config.configuredFrontendValue);
    config.effectiveSlamInputFps = config.configuredSlamInputFps;
    m_state.adaptiveSlamInputFps = config.effectiveSlamInputFps;
    const auto [budgetWidth, budgetHeight] = ComputeVisualFeatureInputBudget(
        m_ctx.aliases.visualFeatureInputMaxWidth,
        m_ctx.aliases.visualFeatureInputMaxHeight, 0);
    config.visualFeatureBudgetWidth = budgetWidth;
    config.visualFeatureBudgetHeight = budgetHeight;
    ApplySlamControlConfig(config);
    LogFrontendChange(config);
    LogInputFpsChange(config);
    LogVisualFeatureProfile(config);
    return config;
}

void SlamFrameInputPort::ApplySlamControlConfig(
    const RuntimeFrameConfig &config)
{
    if (m_ctx.slamControl == nullptr) {
        return;
    }
    m_ctx.slamControl->SetVisualFeatureInputSizeLimit(
        config.visualFeatureBudgetWidth, config.visualFeatureBudgetHeight);
    m_ctx.slamControl->SetFeatureFrontend(config.effectiveFrontend);
}

void SlamFrameInputPort::LogFrontendChange(
    const RuntimeFrameConfig &config)
{
    if (config.effectiveFrontend == m_state.lastAppliedFeatureFrontend) {
        return;
    }
    std::cerr << "[slam] effective_feature_frontend="
              << FeatureFrontendName(config.effectiveFrontend)
              << " requested_value="
              << static_cast<unsigned>(config.configuredFrontendValue)
              << " visual_feature_gate=mode_selected"
              << " prev_tracking_state="
              << m_sharedState.lastTrackingState.load()
              << " prev_tracking_usable="
              << (m_sharedState.lastTrackingUsable.load() ? 1 : 0) << "\n";
    m_state.lastAppliedFeatureFrontend = config.effectiveFrontend;
}

void SlamFrameInputPort::LogInputFpsChange(
    const RuntimeFrameConfig &config)
{
    if (config.configuredSlamInputFps ==
            m_state.lastLoggedConfiguredSlamInputFps &&
        config.effectiveSlamInputFps ==
            m_state.lastLoggedEffectiveSlamInputFps) {
        return;
    }
    std::cerr << "[slam] configured_input_fps="
              << config.configuredSlamInputFps
              << " effective_input_fps=" << config.effectiveSlamInputFps
              << " camera_fps=" << m_ctx.aliases.fps << " frame_drop="
              << (config.effectiveSlamInputFps < m_ctx.aliases.fps
                      ? "enabled"
                      : "disabled")
              << " dpvo_epg_pacing=" << (config.dpvoEpgPacing ? 1 : 0)
              << " smoothed_slam_ms="
              << m_outputState.smoothedSlamMs.load()
              << " smoothed_total_ms="
              << m_outputState.smoothedTotalMs.load()
              << "\n";
    m_state.lastLoggedConfiguredSlamInputFps =
        config.configuredSlamInputFps;
    m_state.lastLoggedEffectiveSlamInputFps = config.effectiveSlamInputFps;
}

void SlamFrameInputPort::LogVisualFeatureProfile(
    const RuntimeFrameConfig &config)
{
    if (!IsVisualFeatureLightGlueFrontend(config.effectiveFrontend)) {
        return;
    }
    if (config.visualFeatureLoadSheddingLevel ==
        m_state.lastLoggedVisualFeatureLoadSheddingLevel) {
        return;
    }
    std::cerr << "[slam] visual_feature_load_profile="
              << DescribeVisualFeatureLoadSheddingLevel(
                     config.visualFeatureLoadSheddingLevel)
              << " frontend=" << FeatureFrontendName(config.effectiveFrontend)
              << " input_max=" << config.visualFeatureBudgetWidth << "x"
              << config.visualFeatureBudgetHeight
              << " tracking_usable="
              << (m_sharedState.lastTrackingUsable.load() ? 1 : 0)
              << " smoothed_slam_ms="
              << m_outputState.smoothedSlamMs.load()
              << " smoothed_total_ms="
              << m_outputState.smoothedTotalMs.load()
              << "\n";
    m_state.lastLoggedVisualFeatureLoadSheddingLevel =
        config.visualFeatureLoadSheddingLevel;
}

SlamFrameInputPort::StereoAcquireResult
SlamFrameInputPort::AcquireStereoBatch(const RuntimeFrameConfig &config)
{
    StereoAcquireResult result{};
    result.acquireStartTp = std::chrono::steady_clock::now();
    const StereoAcquireStatus status =
        m_ctx.perceptionPipeline.AcquireNextStereoBatch(
            m_ctx.cameraProvider, config.effectiveSlamInputFps, 0,
            result.stereoBatch, &m_ctx.frameTimingTracker);
    result.acquireEndTp = std::chrono::steady_clock::now();
    if (status == StereoAcquireStatus::Timeout) {
        return result;
    }
    if (status == StereoAcquireStatus::CameraClockReset) {
        std::cerr << "[slam] camera measurement clock reset, restarting session\n";
        result.sessionOk = false;
        result.stepResult = SlamFrameStepResult::SessionAbort;
        return result;
    }
    if (status == StereoAcquireStatus::CameraUnhealthy) {
        std::cerr << "[slam] camera pipeline unhealthy, aborting session\n";
        result.sessionOk = false;
        result.stepResult = SlamFrameStepResult::SessionAbort;
        return result;
    }
    result.hasFrame = true;
    return result;
}

SlamFrameInputPort::FrameMetadata SlamFrameInputPort::BuildFrameMetadata(
    const StereoBatch &stereoBatch, const RuntimeFrameConfig &config)
{
    FrameMetadata metadata{};
    const auto &left = stereoBatch.stereo.left;
    const auto &right = stereoBatch.stereo.right;
    const auto cameraDiag = m_ctx.cameraProvider.GetDiagnostics();
    PopulateFrameStreamFlags(metadata);
    PopulateFrameTimingMetadata(metadata, stereoBatch, right, cameraDiag);
    PopulateFrameImageQuality(metadata, left, right);
    PopulateFrameProcessingFlags(metadata);
    MaybeLogFrameGap(stereoBatch, config, metadata);
    return metadata;
}

void SlamFrameInputPort::PopulateFrameStreamFlags(FrameMetadata &metadata)
{
    metadata.sendImage = m_ctx.tuning.sendImage.load(std::memory_order_relaxed);
    metadata.sendFeature =
        m_ctx.tuning.sendFeature.load(std::memory_order_relaxed);
    metadata.sendMap = m_ctx.tuning.sendMap.load(std::memory_order_relaxed);
}

void SlamFrameInputPort::PopulateFrameTimingMetadata(
    FrameMetadata &metadata, const StereoBatch &stereoBatch,
    const Core::Ports::ImageFrame &right,
    const Core::Ports::CameraDiagnostics &cameraDiag)
{
    metadata.pairDtMs = cameraDiag.lastPairDtMs;
    metadata.rejectDtMs =
        static_cast<double>(cameraDiag.lastRejectDtUs) / 1000.0;
    metadata.dropUnpairedL = cameraDiag.droppedUnpairedL;
    metadata.dropUnpairedR = cameraDiag.droppedUnpairedR;
    metadata.pendingL = cameraDiag.pendingL;
    metadata.pendingR = cameraDiag.pendingR;
    metadata.captureTimestampNs =
        m_ctx.monoMode ? static_cast<int64_t>(right.timestampNs)
                       : stereoBatch.captureTimestampNs;
    metadata.logicalFrameTimestampNs =
        m_ctx.monoMode ? metadata.captureTimestampNs
                       : stereoBatch.logicalFrameTimestampNs;
    metadata.frameTime =
        static_cast<double>(metadata.captureTimestampNs) * 1e-9;
    const int64_t lastPublishedFrameNs =
        m_outputState.lastPublishedFrameNs.load();
    metadata.frameGapMs = (lastPublishedFrameNs != 0)
                              ? static_cast<double>(
                                    metadata.logicalFrameTimestampNs -
                                    lastPublishedFrameNs) *
                                    1e-6
                              : 0.0;
    metadata.monoStepMs =
        static_cast<double>(stereoBatch.monotonicFrameStepNs) * 1e-6;
}

void SlamFrameInputPort::PopulateFrameImageQuality(
    FrameMetadata &metadata, const Core::Ports::ImageFrame &left,
    const Core::Ports::ImageFrame &right)
{
    ComputeImageStats(left.gray, metadata.stereoQuality.leftMean,
                      metadata.stereoQuality.leftStddev);
    ComputeImageStats(right.gray, metadata.stereoQuality.rightMean,
                      metadata.stereoQuality.rightStddev);
    metadata.stereoQuality.leftSharpness =
        ComputeSharpnessLaplacianVar(left.gray);
    metadata.stereoQuality.rightSharpness =
        ComputeSharpnessLaplacianVar(right.gray);
}

void SlamFrameInputPort::PopulateFrameProcessingFlags(FrameMetadata &metadata)
{
    const bool avoidancePointCloud =
        m_ctx.tuning.avoidanceEnabled.load(std::memory_order_relaxed);
    metadata.debugRightOnlyFeatures = m_ctx.aliases.debugRightOnlyFeatures;
    metadata.extractFeatures =
        metadata.sendFeature ||
        m_sharedState.requestedSlamMode.load() ==
            SmartDrone::Core::Domain::SlamOperationMode::Auto;
    metadata.updatePointCloud =
        !metadata.debugRightOnlyFeatures &&
        (metadata.sendMap || avoidancePointCloud) &&
        (metadata.captureTimestampNs -
         m_outputState.lastPointCloudUpdateNs.load()) >=
            POINT_CLOUD_UPDATE_INTERVAL_NS;
}

void SlamFrameInputPort::MaybeLogFrameGap(
    const StereoBatch &stereoBatch, const RuntimeFrameConfig &config,
    const FrameMetadata &metadata)
{
    if (metadata.frameGapMs <= 0.0 || config.effectiveSlamInputFps <= 0) {
        return;
    }
    const double expectedFrameGapMs =
        1000.0 / static_cast<double>(config.effectiveSlamInputFps);
    if (metadata.frameGapMs <= expectedFrameGapMs) {
        return;
    }
    constexpr int64_t GAP_WARN_MIN_INTERVAL_NS = 1000000000LL;
    if (m_state.lastFrameGapWarnLogNs != 0 &&
        (metadata.logicalFrameTimestampNs - m_state.lastFrameGapWarnLogNs) <
            GAP_WARN_MIN_INTERVAL_NS) {
        return;
    }
    std::cerr << "[slam_gap_warn] frame_gap_ms=" << metadata.frameGapMs
              << " expected_gap_ms=" << expectedFrameGapMs
              << " target_input_fps=" << config.effectiveSlamInputFps
              << " configured_input_fps=" << config.configuredSlamInputFps
              << " camera_fps=" << m_ctx.aliases.fps
              << " frame=" << stereoBatch.frameId << "\n";
    m_state.lastFrameGapWarnLogNs = metadata.logicalFrameTimestampNs;
}

SlamFrameInputPort::SlamInputPreparation
SlamFrameInputPort::PrepareSlamInput(const StereoBatch &stereoBatch,
                                     const FrameMetadata &metadata)
{
    SlamInputPreparation result{};
    result.imuStartTp = std::chrono::steady_clock::now();
    if (m_ctx.useImu && m_state.lastFrameNs != 0) {
        result.slamInput.imu =
            m_ctx.imuProvider.PopWindow(m_state.lastFrameNs,
                                        metadata.captureTimestampNs);
        ImuWindowValidation imuWindow{};
        const double prevFrameTime =
            static_cast<double>(m_state.lastFrameNs) * 1e-9;
        const double expectedImuDtSec = 1.0 / std::max(1, m_ctx.aliases.imuHz);
        const bool imuWindowOk =
            SanitizeImuWindow(result.slamInput.imu, prevFrameTime,
                              metadata.frameTime, expectedImuDtSec, imuWindow);
        if (!imuWindowOk ||
            (result.slamInput.imu.empty() && !m_ctx.aliases.allowEmptyImu)) {
            result.imuEndTp = std::chrono::steady_clock::now();
            return result;
        }
    }
    result.imuEndTp = std::chrono::steady_clock::now();
    m_state.lastFrameNs = metadata.captureTimestampNs;
    PrepareStereoPairForSlam(stereoBatch.stereo, metadata.stereoQuality,
                             m_ctx.aliases.slamLowLightEnhance,
                             result.slamInput.stereo);
    result.slamInput.frameId = stereoBatch.frameId;
    result.slamInput.captureTimestampNs = metadata.captureTimestampNs;
    result.slamInput.frameTimeSec = metadata.frameTime;
    result.ready = true;
    return result;
}

void SlamFrameInputPort::FillPreparedFrame(
    SlamPreparedFrameData &frame, StereoAcquireResult &&acquire,
    SlamInputPreparation &&input, const RuntimeFrameConfig &config,
    const FrameMetadata &metadata) const
{
    frame.acquireStartTp = acquire.acquireStartTp;
    frame.acquireEndTp = acquire.acquireEndTp;
    frame.imuStartTp = input.imuStartTp;
    frame.imuEndTp = input.imuEndTp;
    frame.stereoBatch = std::move(acquire.stereoBatch);
    frame.slamInput = std::move(input.slamInput);
    frame.slamBackend = m_ctx.aliases.slamBackend;
    frame.featureFrontend = config.effectiveFrontend;
    frame.configuredSlamInputFps = config.configuredSlamInputFps;
    frame.effectiveSlamInputFps = config.effectiveSlamInputFps;
    frame.sendImage = metadata.sendImage;
    frame.sendFeature = metadata.sendFeature;
    frame.sendMap = metadata.sendMap;
    frame.pairDtMs = metadata.pairDtMs;
    frame.rejectDtMs = metadata.rejectDtMs;
    frame.dropUnpairedL = metadata.dropUnpairedL;
    frame.dropUnpairedR = metadata.dropUnpairedR;
    frame.pendingL = metadata.pendingL;
    frame.pendingR = metadata.pendingR;
    frame.captureTimestampNs = metadata.captureTimestampNs;
    frame.logicalFrameTimestampNs = metadata.logicalFrameTimestampNs;
    frame.frameTime = metadata.frameTime;
    frame.frameGapMs = metadata.frameGapMs;
    frame.monoStepMs = metadata.monoStepMs;
    frame.meanL = metadata.stereoQuality.leftMean;
    frame.stdL = metadata.stereoQuality.leftStddev;
    frame.meanR = metadata.stereoQuality.rightMean;
    frame.stdR = metadata.stereoQuality.rightStddev;
    frame.sharpL = metadata.stereoQuality.leftSharpness;
    frame.sharpR = metadata.stereoQuality.rightSharpness;
    frame.debugRightOnlyFeatures = metadata.debugRightOnlyFeatures;
    frame.extractFeatures = metadata.extractFeatures;
    frame.updatePointCloud = metadata.updatePointCloud;
}

} // namespace SmartDrone::Core::Application
