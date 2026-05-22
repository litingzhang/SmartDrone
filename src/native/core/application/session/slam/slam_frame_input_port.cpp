#include "core/application/session/slam/slam_frame_input_port.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <utility>

#include "core/application/config/runtime_app_types.h"
#include "core/application/session/slam/imu_window_filter.h"
#include "core/application/session/slam/slam_runtime_control_port.h"
#include "core/application/state/live_pose_state.h"

namespace SmartDrone::core::application {
namespace {

constexpr int64_t kPointCloudUpdateIntervalNs = 200000000LL;
constexpr double kAdaptiveInputFpsHeadroom = 1.25;
constexpr double kAdaptiveInputExtraOverheadMs = 12.0;
constexpr int kAdaptiveMinInputFps = 10;

int ComputeAdaptiveSlamInputFps(int configuredFps, int cameraFps,
                                double smoothedSlamMs)
{
    const int cappedConfiguredFps =
        std::clamp(configuredFps, 1, std::max(1, cameraFps));
    if (!(smoothedSlamMs > 0.0)) {
        return cappedConfiguredFps;
    }

    const double guardedFrameBudgetMs =
        std::max((smoothedSlamMs + kAdaptiveInputExtraOverheadMs) *
                     kAdaptiveInputFpsHeadroom,
                 1.0);
    const int throughputAlignedFps =
        static_cast<int>(std::floor(1000.0 / guardedFrameBudgetMs));
    return std::clamp(throughputAlignedFps, kAdaptiveMinInputFps,
                      cappedConfiguredFps);
}

bool DpvoEpgPacingEnabled()
{
    const char *value = std::getenv("SMART_DRONE_DPVO_EPG_PACING");
    if (value == nullptr || value[0] == '\0') {
        return false;
    }
    std::string normalized(value);
    std::transform(
        normalized.begin(), normalized.end(), normalized.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return !(normalized == "0" || normalized == "false" ||
             normalized == "off" || normalized == "no" ||
             normalized == "disabled");
}

int ComputeVisualFeatureLoadSheddingLevel(int currentLevel,
                                          bool visualFeatureEnabled,
                                          bool lastTrackingUsable,
                                          double smoothedSlamMs,
                                          double smoothedTotalMs)
{
    if (!visualFeatureEnabled || !lastTrackingUsable) {
        return 0;
    }

    if (currentLevel >= 2) {
        if (smoothedTotalMs < 125.0 && smoothedSlamMs < 120.0) {
            return 1;
        }
        return 2;
    }

    if (currentLevel == 1) {
        if (smoothedTotalMs > 155.0 || smoothedSlamMs > 150.0) {
            return 2;
        }
        if (smoothedTotalMs < 105.0 && smoothedSlamMs < 100.0) {
            return 0;
        }
        return 1;
    }

    if (smoothedTotalMs > 125.0 || smoothedSlamMs > 120.0) {
        return 1;
    }
    return 0;
}

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
        static_cast<SmartDrone::core::domain::SlamOperationMode>(
            m_ctx.tuning.slamOperationMode.load(std::memory_order_relaxed));
    if (configuredSlamMode == m_sharedState.requestedSlamMode.load()) {
        return;
    }

    m_sharedState.requestedSlamMode.store(configuredSlamMode);
    m_ctx.autoSlamModeController.Reset();
    const auto effectiveSlamMode =
        configuredSlamMode == SmartDrone::core::domain::SlamOperationMode::Auto
            ? SmartDrone::core::domain::SlamOperationMode::Mapping
            : configuredSlamMode;
    m_sharedState.effectiveSlamMode.store(effectiveSlamMode);
    if (m_ctx.slamControl != nullptr) {
        m_ctx.slamControl->SetOperationMode(effectiveSlamMode);
    }
    std::cerr << "[slam] operation_mode -> "
              << SmartDrone::core::domain::ToString(configuredSlamMode);
    if (configuredSlamMode ==
        SmartDrone::core::domain::SlamOperationMode::Auto) {
        std::cerr << " effective_mode="
                  << SmartDrone::core::domain::ToString(effectiveSlamMode);
    }
    std::cerr << "\n";
    if (configuredSlamMode ==
            SmartDrone::core::domain::SlamOperationMode::Relocalization ||
        configuredSlamMode ==
            SmartDrone::core::domain::SlamOperationMode::TrackingOnly) {
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
    config.dpvoEpgPacing =
        m_ctx.aliases.slamBackend == SlamBackend::DpvoTensorRt &&
        DpvoEpgPacingEnabled();
    config.effectiveSlamInputFps =
        config.dpvoEpgPacing
            ? config.configuredSlamInputFps
            : ComputeAdaptiveSlamInputFps(config.configuredSlamInputFps,
                                          m_ctx.aliases.fps,
                                          m_outputState.smoothedSlamMs.load());
    m_state.adaptiveSlamInputFps = config.effectiveSlamInputFps;
    config.visualFeatureLoadSheddingLevel =
        ComputeVisualFeatureLoadSheddingLevel(
            m_state.visualFeatureLoadSheddingLevel,
            IsVisualFeatureLightGlueFrontend(config.effectiveFrontend),
            m_sharedState.lastTrackingUsable.load(),
            m_outputState.smoothedSlamMs.load(),
            m_outputState.smoothedTotalMs.load());
    m_state.visualFeatureLoadSheddingLevel =
        config.visualFeatureLoadSheddingLevel;
    const auto [budgetWidth, budgetHeight] = ComputeVisualFeatureInputBudget(
        m_ctx.aliases.visualFeatureInputMaxWidth,
        m_ctx.aliases.visualFeatureInputMaxHeight,
        config.visualFeatureLoadSheddingLevel);
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
    if (status == StereoAcquireStatus::CameraUnhealthy) {
        std::cerr << "[slam] camera pipeline unhealthy, aborting session\n";
        result.sessionOk = false;
        result.stepResult = SlamFrameStepResult::SessionAbort;
        return result;
    }
    if (status == StereoAcquireStatus::DroppedByRateLimiter) {
        m_outputState.rateLimitedDrops.fetch_add(1);
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
    metadata.sendImage = m_ctx.tuning.sendImage.load(std::memory_order_relaxed);
    metadata.sendFeature =
        m_ctx.tuning.sendFeature.load(std::memory_order_relaxed);
    metadata.sendMap = m_ctx.tuning.sendMap.load(std::memory_order_relaxed);
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
    ComputeImageStats(left.gray, metadata.stereoQuality.leftMean,
                      metadata.stereoQuality.leftStddev);
    ComputeImageStats(right.gray, metadata.stereoQuality.rightMean,
                      metadata.stereoQuality.rightStddev);
    metadata.stereoQuality.leftSharpness =
        ComputeSharpnessLaplacianVar(left.gray);
    metadata.stereoQuality.rightSharpness =
        ComputeSharpnessLaplacianVar(right.gray);
    metadata.debugRightOnlyFeatures = m_ctx.aliases.debugRightOnlyFeatures;
    metadata.extractFeatures =
        metadata.sendFeature ||
        m_sharedState.requestedSlamMode.load() ==
            SmartDrone::core::domain::SlamOperationMode::Auto;
    metadata.updatePointCloud =
        !metadata.debugRightOnlyFeatures && metadata.sendMap &&
        (metadata.captureTimestampNs -
         m_outputState.lastPointCloudUpdateNs.load()) >=
            kPointCloudUpdateIntervalNs;
    MaybeLogFrameGap(stereoBatch, config, metadata);
    return metadata;
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
    constexpr int64_t kGapWarnMinIntervalNs = 1000000000LL;
    if (m_state.lastFrameGapWarnLogNs != 0 &&
        (metadata.logicalFrameTimestampNs - m_state.lastFrameGapWarnLogNs) <
            kGapWarnMinIntervalNs) {
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

} // namespace SmartDrone::core::application
