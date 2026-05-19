#include "core/application/session/slam_frame_processor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "common/logger.h"
#include "core/application/session/imu_window_filter.h"

namespace smartdrone::core::application {

namespace {

constexpr int64_t kPointCloudUpdateIntervalNs = 200000000LL;
constexpr uint64_t kSlamDfxLogEveryNFrames = 30;
constexpr int kVisualFeatureStereoWeakMatchThreshold = 24;
constexpr double kAdaptiveTimingEmaAlpha = 0.18;
constexpr double kAdaptiveInputFpsHeadroom = 1.25;
constexpr double kAdaptiveInputExtraOverheadMs = 12.0;
constexpr int kAdaptiveMinInputFps = 10;
constexpr uint64_t kPoseAxisLogEveryNFrames = 30;

double UpdateEma(double current, double sample) {
  if (!(sample > 0.0)) {
    return current;
  }
  if (!(current > 0.0)) {
    return sample;
  }
  return (1.0 - kAdaptiveTimingEmaAlpha) * current +
         kAdaptiveTimingEmaAlpha * sample;
}

int ComputeAdaptiveSlamInputFps(int configuredFps, int cameraFps,
                                double smoothedSlamMs) {
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

bool DpvoEpgPacingEnabled() {
  const char *value = std::getenv("SMART_DRONE_DPVO_EPG_PACING");
  if (value == nullptr || value[0] == '\0') {
    return false;
  }
  std::string normalized(value);
  std::transform(
      normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return !(normalized == "0" || normalized == "false" || normalized == "off" ||
           normalized == "no" || normalized == "disabled");
}

LivePoseQuality ToLivePoseQuality(smartdrone::core::ports::PoseQuality quality) {
  if (quality == smartdrone::core::ports::PoseQuality::Good) {
    return LivePoseQuality::Good;
  }
  if (quality == smartdrone::core::ports::PoseQuality::Weak) {
    return LivePoseQuality::Weak;
  }
  return LivePoseQuality::Lost;
}

int ComputeVisualFeatureLoadSheddingLevel(int currentLevel,
                                          bool visualFeatureEnabled,
                                          bool lastTrackingUsable,
                                          double smoothedSlamMs,
                                          double smoothedTotalMs) {
  if (!visualFeatureEnabled) {
    return 0;
  }
  if (!lastTrackingUsable) {
    return 0;
  }

  if (currentLevel >= 2) {
    if (lastTrackingUsable && smoothedTotalMs < 125.0 &&
        smoothedSlamMs < 120.0) {
      return 1;
    }
    return 2;
  }

  if (currentLevel == 1) {
    if (!lastTrackingUsable || smoothedTotalMs > 155.0 ||
        smoothedSlamMs > 150.0) {
      return 2;
    }
    if (lastTrackingUsable && smoothedTotalMs < 105.0 &&
        smoothedSlamMs < 100.0) {
      return 0;
    }
    return 1;
  }

  if (!lastTrackingUsable || smoothedTotalMs > 125.0 ||
      smoothedSlamMs > 120.0) {
    return 1;
  }
  return 0;
}

std::pair<int, int> ComputeVisualFeatureInputBudget(int baseWidth,
                                                    int baseHeight,
                                                    int loadSheddingLevel) {
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

const char *DescribeVisualFeatureLoadSheddingLevel(int loadSheddingLevel) {
  switch (loadSheddingLevel) {
  case 2:
    return "aggressive";
  case 1:
    return "moderate";
  default:
    return "nominal";
  }
}

uint8_t ComposeResetCounter(uint8_t sessionBase, uint8_t continuityCounter) {
  return static_cast<uint8_t>(sessionBase + continuityCounter);
}

uint16_t ComposeResetMapCount(uint16_t sessionBase,
                              uint16_t continuityResetMapCount) {
  return static_cast<uint16_t>(sessionBase + continuityResetMapCount);
}

struct RuntimeTbcOverride {
  float tx{0.0f};
  float ty{0.0f};
  float tz{0.0f};
  float rollDeg{0.0f};
  float pitchDeg{0.0f};
  float yawDeg{0.0f};
};

Sophus::SE3f BuildBodyToCamFromRuntimeOverride(
    const RuntimeTbcOverride &overrideValue) {
  constexpr float kDegToRad = 0.017453292519943295769f;
  const float rollRad = overrideValue.rollDeg * kDegToRad;
  const float pitchRad = overrideValue.pitchDeg * kDegToRad;
  const float yawRad = overrideValue.yawDeg * kDegToRad;
  const Eigen::AngleAxisf rollRotation(rollRad, Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitchRotation(pitchRad, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf yawRotation(yawRad, Eigen::Vector3f::UnitZ());
  const Eigen::Quaternionf q = yawRotation * pitchRotation * rollRotation;
  return Sophus::SE3f(
      Sophus::SO3f(q),
      Eigen::Vector3f(overrideValue.tx, overrideValue.ty, overrideValue.tz));
}

Sophus::SE3f BuildBodyToCamPitchDelta(float pitchDeg) {
  constexpr float kDegToRad = 0.017453292519943295769f;
  const Eigen::AngleAxisf pitchRotation(pitchDeg * kDegToRad,
                                        Eigen::Vector3f::UnitY());
  return Sophus::SE3f(Sophus::SO3f(Eigen::Quaternionf(pitchRotation)),
                      Eigen::Vector3f::Zero());
}

const char *FeatureFrontendName(FeatureFrontend frontend) {
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

FeatureFrontend ParseRuntimeFeatureFrontendValue(uint8_t value) {
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

SlamFrameProcessor::SlamFrameProcessor(Context &context, State &state)
    : m_ctx(context), m_state(state) {}

SlamFrameProcessor::StepResult SlamFrameProcessor::StepBackend() {
  if (m_ctx.slamControl != nullptr) {
    m_ctx.slamControl->StepBackend();
  }
  return StepResult::Continue;
}

SlamFrameProcessor::StepResult
SlamFrameProcessor::AcquireAndPrepareFrame(bool &sessionOk,
                                           PreparedFrame &frame) {
  frame = PreparedFrame{};
  frame.frameStartTp = std::chrono::steady_clock::now();
  SyncRequestedSlamMode();
  const RuntimeFrameConfig config = ApplyRuntimeFrameConfig();
  StereoAcquireResult acquire = AcquireStereoBatch(config, sessionOk);
  if (!acquire.hasFrame) {
    return acquire.stepResult;
  }
  const FrameMetadata metadata =
      BuildFrameMetadata(acquire.stereoBatch, config);
  SlamInputPreparation input =
      PrepareSlamInput(acquire.stereoBatch, metadata);
  if (!input.ready) {
    return input.stepResult;
  }
  FillPreparedFrame(frame, std::move(acquire), std::move(input), config,
                    metadata);
  return StepResult::Continue;
}

void SlamFrameProcessor::SyncRequestedSlamMode() {
  const auto configuredSlamMode =
      static_cast<smartdrone::core::domain::SlamOperationMode>(
          m_ctx.tuning.slamOperationMode.load(std::memory_order_relaxed));
  if (configuredSlamMode == m_state.requestedSlamMode) {
    return;
  }

  m_state.requestedSlamMode = configuredSlamMode;
  m_ctx.autoSlamModeController.Reset();
  m_state.effectiveSlamMode =
      m_state.requestedSlamMode ==
              smartdrone::core::domain::SlamOperationMode::Auto
          ? smartdrone::core::domain::SlamOperationMode::Mapping
          : m_state.requestedSlamMode;
  if (m_ctx.slamControl != nullptr) {
    m_ctx.slamControl->SetOperationMode(m_state.effectiveSlamMode);
  }
  std::cerr << "[slam] operation_mode -> "
            << smartdrone::core::domain::ToString(m_state.requestedSlamMode);
  if (m_state.requestedSlamMode ==
      smartdrone::core::domain::SlamOperationMode::Auto) {
    std::cerr << " effective_mode="
              << smartdrone::core::domain::ToString(m_state.effectiveSlamMode);
  }
  std::cerr << "\n";
  if (m_state.requestedSlamMode ==
          smartdrone::core::domain::SlamOperationMode::Relocalization ||
      m_state.requestedSlamMode ==
          smartdrone::core::domain::SlamOperationMode::TrackingOnly) {
    std::cerr << "[slam] note: requested mode currently maps to backend "
                 "localization-only mode\n";
  }
  m_ctx.livePose.SetSlamMode(ToRuntimeSlamModeValue(m_state.effectiveSlamMode));
}

SlamFrameProcessor::RuntimeFrameConfig
SlamFrameProcessor::ApplyRuntimeFrameConfig() {
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
                                        m_state.smoothedSlamMs);
  m_state.adaptiveSlamInputFps = config.effectiveSlamInputFps;
  config.visualFeatureLoadSheddingLevel = ComputeVisualFeatureLoadSheddingLevel(
      m_state.visualFeatureLoadSheddingLevel,
      IsVisualFeatureLightGlueFrontend(config.effectiveFrontend),
      m_state.lastTrackingUsable, m_state.smoothedSlamMs,
      m_state.smoothedTotalMs);
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

void SlamFrameProcessor::ApplySlamControlConfig(
    const RuntimeFrameConfig &config) {
  if (m_ctx.slamControl == nullptr) {
    return;
  }
  m_ctx.slamControl->SetVisualFeatureInputSizeLimit(
      config.visualFeatureBudgetWidth, config.visualFeatureBudgetHeight);
  m_ctx.slamControl->SetFeatureFrontend(config.effectiveFrontend);
}

void SlamFrameProcessor::LogFrontendChange(const RuntimeFrameConfig &config) {
  if (config.effectiveFrontend == m_state.lastAppliedFeatureFrontend) {
    return;
  }
  std::cerr << "[slam] effective_feature_frontend="
            << FeatureFrontendName(config.effectiveFrontend)
            << " requested_value="
            << static_cast<unsigned>(config.configuredFrontendValue)
            << " visual_feature_gate=mode_selected"
            << " prev_tracking_state=" << m_state.lastTrackingState
            << " prev_tracking_usable=" << (m_state.lastTrackingUsable ? 1 : 0)
            << "\n";
  m_state.lastAppliedFeatureFrontend = config.effectiveFrontend;
}

void SlamFrameProcessor::LogInputFpsChange(const RuntimeFrameConfig &config) {
  if (config.configuredSlamInputFps ==
          m_state.lastLoggedConfiguredSlamInputFps &&
      config.effectiveSlamInputFps ==
          m_state.lastLoggedEffectiveSlamInputFps) {
    return;
  }
  std::cerr << "[slam] configured_input_fps=" << config.configuredSlamInputFps
            << " effective_input_fps=" << config.effectiveSlamInputFps
            << " camera_fps=" << m_ctx.aliases.fps << " frame_drop="
            << (config.effectiveSlamInputFps < m_ctx.aliases.fps ? "enabled"
                                                                 : "disabled")
            << " dpvo_epg_pacing=" << (config.dpvoEpgPacing ? 1 : 0)
            << " smoothed_slam_ms=" << m_state.smoothedSlamMs
            << " smoothed_total_ms=" << m_state.smoothedTotalMs << "\n";
  m_state.lastLoggedConfiguredSlamInputFps = config.configuredSlamInputFps;
  m_state.lastLoggedEffectiveSlamInputFps = config.effectiveSlamInputFps;
}

void SlamFrameProcessor::LogVisualFeatureProfile(
    const RuntimeFrameConfig &config) {
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
            << " tracking_usable=" << (m_state.lastTrackingUsable ? 1 : 0)
            << " smoothed_slam_ms=" << m_state.smoothedSlamMs
            << " smoothed_total_ms=" << m_state.smoothedTotalMs << "\n";
  m_state.lastLoggedVisualFeatureLoadSheddingLevel =
      config.visualFeatureLoadSheddingLevel;
}

SlamFrameProcessor::StereoAcquireResult
SlamFrameProcessor::AcquireStereoBatch(const RuntimeFrameConfig &config,
                                       bool &sessionOk) {
  StereoAcquireResult result{};
  result.acquireStartTp = std::chrono::steady_clock::now();
  const StereoAcquireStatus status = m_ctx.perceptionPipeline.AcquireNextStereoBatch(
      m_ctx.cameraProvider, config.effectiveSlamInputFps, 0, result.stereoBatch,
      &m_ctx.frameTimingTracker);
  result.acquireEndTp = std::chrono::steady_clock::now();
  if (status == StereoAcquireStatus::Timeout) {
    return result;
  }
  if (status == StereoAcquireStatus::CameraUnhealthy) {
    std::cerr << "[slam] camera pipeline unhealthy, aborting session\n";
    sessionOk = false;
    result.stepResult = StepResult::SessionAbort;
    return result;
  }
  if (status == StereoAcquireStatus::DroppedByRateLimiter) {
    ++m_state.rateLimitedDrops;
    return result;
  }
  result.hasFrame = true;
  return result;
}

SlamFrameProcessor::FrameMetadata
SlamFrameProcessor::BuildFrameMetadata(const StereoBatch &stereoBatch,
                                       const RuntimeFrameConfig &config) {
  FrameMetadata metadata{};
  const auto &left = stereoBatch.stereo.left;
  const auto &right = stereoBatch.stereo.right;
  const auto cameraDiag = m_ctx.cameraProvider.GetDiagnostics();
  metadata.sendImage = m_ctx.tuning.sendImage.load(std::memory_order_relaxed);
  metadata.sendFeature =
      m_ctx.tuning.sendFeature.load(std::memory_order_relaxed);
  metadata.sendMap = m_ctx.tuning.sendMap.load(std::memory_order_relaxed);
  metadata.pairDtMs = cameraDiag.lastPairDtMs;
  metadata.rejectDtMs = static_cast<double>(cameraDiag.lastRejectDtUs) / 1000.0;
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
  metadata.frameTime = static_cast<double>(metadata.captureTimestampNs) * 1e-9;
  metadata.frameGapMs =
      (m_state.lastPublishedFrameNs != 0)
          ? static_cast<double>(metadata.logicalFrameTimestampNs -
                                m_state.lastPublishedFrameNs) *
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
      metadata.sendFeature || m_state.requestedSlamMode ==
                                  smartdrone::core::domain::SlamOperationMode::Auto;
  metadata.updatePointCloud =
      !metadata.debugRightOnlyFeatures && metadata.sendMap &&
      (metadata.captureTimestampNs - m_state.lastPointCloudUpdateNs) >=
          kPointCloudUpdateIntervalNs;
  MaybeLogFrameGap(stereoBatch, config, metadata);
  return metadata;
}

void SlamFrameProcessor::MaybeLogFrameGap(
    const StereoBatch &stereoBatch, const RuntimeFrameConfig &config,
    const FrameMetadata &metadata) {
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

SlamFrameProcessor::SlamInputPreparation
SlamFrameProcessor::PrepareSlamInput(const StereoBatch &stereoBatch,
                                     const FrameMetadata &metadata) {
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
    const bool imuWindowOk = SanitizeImuWindow(
        result.slamInput.imu, prevFrameTime, metadata.frameTime,
        expectedImuDtSec, imuWindow);
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

void SlamFrameProcessor::FillPreparedFrame(
    PreparedFrame &frame, StereoAcquireResult &&acquire,
    SlamInputPreparation &&input, const RuntimeFrameConfig &config,
    const FrameMetadata &metadata) const {
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

SlamFrameProcessor::StepResult
SlamFrameProcessor::TrackPreparedFrame(std::shared_ptr<PreparedFrame> frame,
                                       TrackedFrame &tracked) {
  if (!frame) {
    return StepResult::Continue;
  }
  auto &stereoBatch = frame->stereoBatch;
  auto &slamInput = frame->slamInput;
  auto &L = stereoBatch.stereo.left;
  auto &R = stereoBatch.stereo.right;
  const bool debugRightOnlyFeatures = frame->debugRightOnlyFeatures;
  const bool extractFeatures = frame->extractFeatures;
  const bool updatePointCloud = frame->updatePointCloud;

  const auto slamStartTp = std::chrono::steady_clock::now();
  const uint64_t slamInputTimestampNs = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          slamStartTp.time_since_epoch())
          .count());
  m_ctx.frameTimingTracker.MarkSlamIn(slamInput.frameId, slamInputTimestampNs);
  smartdrone::core::ports::SlamOutput slamOutput{};
  slamOutput.frameId = slamInput.frameId;
  slamOutput.captureTimestampNs = slamInput.captureTimestampNs;
  if (debugRightOnlyFeatures) {
    slamOutput.leftFeatures.clear();
    slamOutput.rightFeatures = ComputeOrbDebugFeatures(R.gray);
  } else {
    slamOutput =
        m_ctx.slamEngine.Process(slamInput, extractFeatures, updatePointCloud);
  }
  const auto slamEndTp = std::chrono::steady_clock::now();
  const uint64_t slamOutputTimestampNs = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          slamEndTp.time_since_epoch())
          .count());
  m_ctx.frameTimingTracker.MarkSlamOut(slamInput.frameId,
                                       slamOutputTimestampNs);

  tracked.frame = std::move(frame);
  tracked.slamOutput = std::move(slamOutput);
  tracked.slamStartTp = slamStartTp;
  tracked.slamEndTp = slamEndTp;
  return StepResult::Continue;
}

SlamFrameProcessor::StepResult SlamFrameProcessor::PostprocessTrackedFrame(
    std::shared_ptr<TrackedFrame> tracked, PublishedFrame &published) {
  if (!tracked || !tracked->frame) {
    return StepResult::Continue;
  }

  const auto postStartTp = std::chrono::steady_clock::now();
  const TrackingContext tracking = ResolveTrackingContext(*tracked);
  const Sophus::SE3f twcRaw = ResolveRawPose(*tracked, tracking);
  const StereoExtrinsicsContext extrinsics = ResolveStereoExtrinsics();
  const PosePostprocessor::ProcessRequest poseRequest = BuildPoseRequest(
      twcRaw, tracking, tracked->frame->captureTimestampNs, extrinsics);
  const auto poseResult = m_ctx.posePostprocessor.ProcessPose(poseRequest);
  MaybeLogPoseAxis(*tracked, tracking, twcRaw, poseResult);
  const auto postEndTp = std::chrono::steady_clock::now();

  UpdateAutoSlamMode(*tracked, tracking, poseResult,
                     tracked->frame->frameGapMs);
  const PostprocessArtifacts artifacts = BuildPostprocessArtifacts(
      *tracked, tracking, poseResult, postStartTp, postEndTp);
  FillPublishedFrame(std::move(tracked), artifacts, published);
  return StepResult::Continue;
}

SlamFrameProcessor::TrackingContext
SlamFrameProcessor::ResolveTrackingContext(const TrackedFrame &tracked) {
  const bool debugRightOnlyFeatures = tracked.frame->debugRightOnlyFeatures;
  TrackingContext tracking{};
  tracking.state = debugRightOnlyFeatures ? ports::kSlamTrackingLost
                                          : tracked.slamOutput.trackingState;
  tracking.usable = !debugRightOnlyFeatures &&
                    ports::IsSlamTrackingPoseUsable(tracking.state);
  tracking.mapId = debugRightOnlyFeatures ? 0UL : tracked.slamOutput.mapId;
  m_state.lastTrackingState = tracking.state;
  m_state.lastTrackingUsable = tracking.usable;
  if (tracking.mapId != PosePostprocessor::ContinuityMapper::kInvalidMapId &&
      tracking.mapId != m_state.lastRawMapId) {
    m_state.lastRawMapId = tracking.mapId;
  }
  return tracking;
}

Sophus::SE3f
SlamFrameProcessor::ResolveRawPose(const TrackedFrame &tracked,
                                   const TrackingContext &tracking) {
  Sophus::SE3f twcRaw =
      m_state.haveLastValidTwcRaw ? m_state.lastValidTwcRaw : Sophus::SE3f();
  if (!tracking.usable && !tracked.slamOutput.poseValid) {
    return twcRaw;
  }
  if (tracked.frame->debugRightOnlyFeatures || !tracked.slamOutput.poseValid) {
    return twcRaw;
  }

  const auto &pose = tracked.slamOutput.pose;
  const Eigen::Quaternionf rawQ(pose.qw, pose.qx, pose.qy, pose.qz);
  twcRaw = Sophus::SE3f(Sophus::SO3f(rawQ),
                        Eigen::Vector3f(pose.x, pose.y, pose.z));
  m_state.lastValidTwcRaw = twcRaw;
  m_state.haveLastValidTwcRaw = true;
  return twcRaw;
}

SlamFrameProcessor::StereoExtrinsicsContext
SlamFrameProcessor::ResolveStereoExtrinsics() const {
  StereoExtrinsicsContext extrinsics{m_ctx.stereoBodyExtrinsics.loaded,
                                     m_ctx.stereoBodyExtrinsics.Tbc};
  if (m_ctx.useImu || m_ctx.monoMode ||
      !m_ctx.tuning.useCustomTbc.load(std::memory_order_relaxed)) {
    return extrinsics;
  }

  const float pitchDeg =
      m_ctx.tuning.tbcPitchDeg.load(std::memory_order_relaxed);
  extrinsics.loaded = true;
  if (m_ctx.stereoBodyExtrinsics.loaded) {
    extrinsics.bodyToCamera =
        m_ctx.stereoBodyExtrinsics.Tbc * BuildBodyToCamPitchDelta(pitchDeg);
    return extrinsics;
  }

  const RuntimeTbcOverride overrideValue{
      m_ctx.tuning.tbcTx.load(std::memory_order_relaxed),
      m_ctx.tuning.tbcTy.load(std::memory_order_relaxed),
      m_ctx.tuning.tbcTz.load(std::memory_order_relaxed),
      m_ctx.tuning.tbcRollDeg.load(std::memory_order_relaxed),
      pitchDeg,
      m_ctx.tuning.tbcYawDeg.load(std::memory_order_relaxed)};
  extrinsics.bodyToCamera = BuildBodyToCamFromRuntimeOverride(overrideValue);
  return extrinsics;
}

PosePostprocessor::ProcessRequest SlamFrameProcessor::BuildPoseRequest(
    const Sophus::SE3f &twcRaw, const TrackingContext &tracking,
    int64_t captureTimestampNs,
    const StereoExtrinsicsContext &extrinsics) const {
  PosePostprocessor::ProcessRequest request{};
  request.twcRaw = twcRaw;
  request.useImu = m_ctx.useImu;
  request.trackingUsable = tracking.usable;
  request.mapId = tracking.mapId;
  request.stereoExtrinsicsLoaded = extrinsics.loaded;
  request.stereoBodyExtrinsics = extrinsics.bodyToCamera;
  request.stereoReferencePoseSet = &m_state.stereoReferencePoseSet;
  request.stereoReferencePose = &m_state.stereoReferencePose;
  request.frameNs = captureTimestampNs;
  request.readRangeSensor = m_ctx.readRangeSensor;
  return request;
}

void SlamFrameProcessor::MaybeLogPoseAxis(
    const TrackedFrame &tracked, const TrackingContext &tracking,
    const Sophus::SE3f &twcRaw,
    const PosePostprocessor::Result &poseResult) const {
  if (m_ctx.useImu || m_ctx.monoMode || !tracked.slamOutput.poseValid ||
      (tracked.slamOutput.frameId % kPoseAxisLogEveryNFrames) != 0) {
    return;
  }

  const Eigen::Vector3f camT = twcRaw.translation();
  const auto &dbg = poseResult.debug;
  const Eigen::Vector3f frdT(poseResult.poseEstimate.x,
                             poseResult.poseEstimate.y,
                             poseResult.poseEstimate.z);
  std::cerr << "[pose_axis] frame=" << tracked.slamOutput.frameId
            << " cam_t=" << camT.x() << "," << camT.y() << "," << camT.z()
            << " body_t=" << dbg.bodyX << "," << dbg.bodyY << ","
            << dbg.bodyZ << " local_t=" << dbg.localX << "," << dbg.localY
            << "," << dbg.localZ << " frd_t=" << frdT.x() << ","
            << frdT.y() << "," << frdT.z()
            << " tbc=" << (dbg.stereoExtrinsicsApplied ? 1 : 0)
            << " ref=" << (dbg.referenceApplied ? 1 : 0)
            << " tracking=" << (tracking.usable ? 1 : 0)
            << " q=" << poseResult.poseEstimate.qw << ","
            << poseResult.poseEstimate.qx << "," << poseResult.poseEstimate.qy
            << "," << poseResult.poseEstimate.qz << "\n";
}

void SlamFrameProcessor::UpdateAutoSlamMode(
    const TrackedFrame &tracked, const TrackingContext &tracking,
    const PosePostprocessor::Result &poseResult, double frameGapMs) {
  if (m_state.requestedSlamMode !=
      smartdrone::core::domain::SlamOperationMode::Auto) {
    return;
  }

  const auto autoEffectiveMode = m_ctx.autoSlamModeController.Observe(
      tracking.usable, poseResult.quality, frameGapMs,
      tracked.slamOutput.leftFeatures.size(),
      tracked.slamOutput.rightFeatures.size());
  if (autoEffectiveMode == m_state.effectiveSlamMode) {
    return;
  }

  m_state.effectiveSlamMode = autoEffectiveMode;
  if (m_ctx.slamControl != nullptr) {
    m_ctx.slamControl->SetOperationMode(m_state.effectiveSlamMode);
  }
  m_ctx.livePose.SetSlamMode(ToRuntimeSlamModeValue(m_state.effectiveSlamMode));
  std::cerr << "[slam_auto] effective_mode -> "
            << smartdrone::core::domain::ToString(m_state.effectiveSlamMode)
            << " quality=" << static_cast<int>(poseResult.quality)
            << " state=" << tracking.state
            << " featL=" << tracked.slamOutput.leftFeatures.size()
            << " featR=" << tracked.slamOutput.rightFeatures.size()
            << " frame_gap_ms=" << frameGapMs << "\n";
}

SlamFrameProcessor::PostprocessArtifacts
SlamFrameProcessor::BuildPostprocessArtifacts(
    const TrackedFrame &tracked, const TrackingContext &tracking,
    const PosePostprocessor::Result &poseResult,
    std::chrono::steady_clock::time_point postStartTp,
    std::chrono::steady_clock::time_point postEndTp) const {
  PostprocessArtifacts artifacts{};
  artifacts.poseResult = poseResult;
  artifacts.postStartTp = postStartTp;
  artifacts.postEndTp = postEndTp;
  artifacts.trackingState = tracking.state;
  artifacts.trackingUsable = tracking.usable;
  artifacts.effectiveResetCounter =
      ComposeResetCounter(m_state.sessionResetCounterBase,
                          poseResult.resetCounter);
  artifacts.effectiveResetMapCount =
      ComposeResetMapCount(m_state.sessionResetMapCountBase,
                           poseResult.resetMapCount);
  artifacts.pointCount =
      tracked.frame->sendMap ? tracked.slamOutput.pointCloudXyz.size() / 3 : 0;
  return artifacts;
}

void SlamFrameProcessor::FillPublishedFrame(
    std::shared_ptr<TrackedFrame> tracked,
    const PostprocessArtifacts &artifacts, PublishedFrame &published) const {
  published.frame = std::move(tracked);
  published.poseResult = artifacts.poseResult;
  published.cloudStartTp = artifacts.postEndTp;
  published.cloudEndTp = artifacts.postEndTp;
  published.udpStartTp = artifacts.postEndTp;
  published.udpEndTp = artifacts.postEndTp;
  published.postStartTp = artifacts.postStartTp;
  published.postEndTp = artifacts.postEndTp;
  published.livePoseStartTp = artifacts.postEndTp;
  published.livePoseEndTp = artifacts.postEndTp;
  published.publishStartTp = artifacts.postEndTp;
  published.publishEndTp = artifacts.postEndTp;
  published.pointCount = artifacts.pointCount;
  published.trackingState = artifacts.trackingState;
  published.trackingUsable = artifacts.trackingUsable;
  published.effectiveResetCounter = artifacts.effectiveResetCounter;
  published.effectiveResetMapCount = artifacts.effectiveResetMapCount;
}

SlamFrameProcessor::StepResult
SlamFrameProcessor::PublishTrackedFrame(std::shared_ptr<TrackedFrame> tracked,
                                        PublishedFrame &published) {
  return PostprocessTrackedFrame(std::move(tracked), published);
}

SlamFrameProcessor::StepResult
SlamFrameProcessor::EmitPointCloud(PublishedFrame &published) {
  if (!published.frame || !published.frame->frame) {
    return StepResult::Continue;
  }
  auto &tracked = *published.frame;
  auto &frame = *tracked.frame;
  auto &slamOutput = tracked.slamOutput;
  const bool sendImage = frame.sendImage;
  const bool sendFeature = frame.sendFeature;
  const bool sendMap = frame.sendMap;
  const bool updatePointCloud = frame.updatePointCloud;

  const auto cloudStartTp = std::chrono::steady_clock::now();
  size_t pointCount = 0;
  if (m_ctx.aliases.udpEnable && (sendImage || sendFeature || sendMap)) {
    if (updatePointCloud) {
      m_ctx.livePose.UpdatePointCloud(slamOutput.pointCloudXyz);
      m_state.lastPointCloudUpdateNs = frame.captureTimestampNs;
    }
    pointCount = slamOutput.pointCloudXyz.size() / 3;
  }
  published.cloudStartTp = cloudStartTp;
  published.cloudEndTp = std::chrono::steady_clock::now();
  published.pointCount = pointCount;
  return StepResult::Continue;
}

SlamFrameProcessor::StepResult
SlamFrameProcessor::EmitLivePose(PublishedFrame &published) {
  if (!published.frame || !published.frame->frame) {
    return StepResult::Continue;
  }
  auto &tracked = *published.frame;
  auto &slamOutput = tracked.slamOutput;
  const auto &poseResult = published.poseResult;
  const auto livePoseStartTp = std::chrono::steady_clock::now();
  const bool livePoseValid =
      slamOutput.poseValid && poseResult.poseEstimate.valid &&
      published.trackingUsable &&
      poseResult.quality != smartdrone::core::ports::PoseQuality::Lost;
  LivePoseUpdate update{};
  update.runtimeMode = RUNTIME_MODE_SLAM;
  update.trackingState = static_cast<uint8_t>(published.trackingState);
  update.resetCounter = published.effectiveResetCounter;
  update.resetMapCount = published.effectiveResetMapCount;
  update.pose.x = poseResult.poseEstimate.x;
  update.pose.y = poseResult.poseEstimate.y;
  update.pose.z = poseResult.poseEstimate.z;
  update.pose.qw = poseResult.poseEstimate.qw;
  update.pose.qx = poseResult.poseEstimate.qx;
  update.pose.qy = poseResult.poseEstimate.qy;
  update.pose.qz = poseResult.poseEstimate.qz;
  update.quality = ToLivePoseQuality(poseResult.quality);
  update.poseValid = livePoseValid;
  m_ctx.livePose.UpdatePose(update);
  published.livePoseStartTp = livePoseStartTp;
  published.livePoseEndTp = std::chrono::steady_clock::now();
  return StepResult::Continue;
}

SlamFrameProcessor::StepResult
SlamFrameProcessor::EmitMavlink(PublishedFrame &published) {
  if (!published.frame || !published.frame->frame) {
    return StepResult::Continue;
  }
  auto &tracked = *published.frame;
  auto &slamOutput = tracked.slamOutput;
  const auto &poseResult = published.poseResult;
  const auto publishStartTp = std::chrono::steady_clock::now();
  m_ctx.posePublisher.PublishPose(
      slamOutput.frameId, poseResult.poseEstimate, poseResult.velocityEstimate,
      published.effectiveResetCounter, published.effectiveResetMapCount,
      published.trackingState, poseResult.quality);
  published.publishStartTp = publishStartTp;
  published.publishEndTp = std::chrono::steady_clock::now();
  return StepResult::Continue;
}

SlamFrameProcessor::StepResult
SlamFrameProcessor::EmitUdp(PublishedFrame &published) {
  if (!published.frame || !published.frame->frame) {
    return StepResult::Continue;
  }
  auto &tracked = *published.frame;
  auto &frame = *tracked.frame;
  auto &stereoBatch = frame.stereoBatch;
  auto &slamOutput = tracked.slamOutput;
  auto &L = stereoBatch.stereo.left;
  auto &R = stereoBatch.stereo.right;
  const bool sendImage = frame.sendImage;
  const bool sendFeature = frame.sendFeature;
  const bool sendMap = frame.sendMap;
  const double frameTime = frame.frameTime;

  const auto udpStartTp = std::chrono::steady_clock::now();
  if (m_ctx.aliases.udpEnable && (sendImage || sendFeature || sendMap)) {
    if (m_ctx.monoMode) {
      m_ctx.udpSender.Enqueue(1, slamOutput.frameId, R.sequence, frameTime,
                              R.gray, slamOutput.rightFeatures, sendImage,
                              sendFeature);
    } else {
      m_ctx.udpSender.Enqueue(0, slamOutput.frameId, L.sequence, frameTime,
                              L.gray, slamOutput.leftFeatures, sendImage,
                              sendFeature);
      m_ctx.udpSender.Enqueue(1, slamOutput.frameId, R.sequence, frameTime,
                              R.gray, slamOutput.rightFeatures, sendImage,
                              sendFeature);
    }
    m_ctx.udpSender.StepAll();
  }
  published.udpStartTp = udpStartTp;
  published.udpEndTp = std::chrono::steady_clock::now();
  return StepResult::Continue;
}

SlamFrameProcessor::StepResult
SlamFrameProcessor::EmitDfx(PublishedFrame &published) {
  if (!published.frame || !published.frame->frame) {
    return StepResult::Continue;
  }
  const DfxSample sample = MakeDfxSample(published);
  ++m_state.frameIndex;
  m_state.lastPublishedFrameNs = sample.frame.logicalFrameTimestampNs;
  if (ShouldLogDfx(sample)) {
    LogDfxLine(sample);
  }
  UpdateDfxAverages(sample);
  return StepResult::Continue;
}

SlamFrameProcessor::DfxSample
SlamFrameProcessor::MakeDfxSample(PublishedFrame &published) const {
  auto &tracked = *published.frame;
  auto &frame = *tracked.frame;
  auto &slamOutput = tracked.slamOutput;
  const bool stereoWeak = slamOutput.usedVisualFeatureFrontend &&
                          !m_ctx.monoMode &&
                          slamOutput.visualFeatureMatchedStereoCount <
                              kVisualFeatureStereoWeakMatchThreshold;
  return {frame, tracked, published, frame.slamInput, slamOutput,
          published.poseResult, MakeDfxTiming(frame, tracked, published),
          stereoWeak};
}

SlamFrameProcessor::DfxTiming
SlamFrameProcessor::MakeDfxTiming(const PreparedFrame &frame,
                                  const TrackedFrame &tracked,
                                  const PublishedFrame &published) const {
  return {
      DurationMs(frame.acquireStartTp, frame.acquireEndTp),
      DurationMs(frame.imuStartTp, frame.imuEndTp),
      DurationMs(tracked.slamStartTp, tracked.slamEndTp),
      DurationMs(published.cloudStartTp, published.cloudEndTp),
      DurationMs(published.udpStartTp, published.udpEndTp),
      DurationMs(published.postStartTp, published.postEndTp),
      DurationMs(published.livePoseStartTp, published.livePoseEndTp),
      DurationMs(published.publishStartTp, published.publishEndTp),
      DurationMs(frame.frameStartTp, published.publishEndTp),
  };
}

bool SlamFrameProcessor::ShouldLogDfx(const DfxSample &sample) const {
  const auto &output = sample.slamOutput;
  const bool periodic = kSlamDfxLogEveryNFrames > 0 &&
                        (m_state.frameIndex % kSlamDfxLogEveryNFrames) == 0;
  const bool abnormal = !sample.poseResult.poseEstimate.valid ||
                        sample.published.trackingState <= 0 ||
                        sample.timing.totalMs > 80.0 ||
                        output.leftFeatures.empty() ||
                        output.rightFeatures.empty() ||
                        sample.visualFeatureStereoWeak;
  return periodic || abnormal;
}

void SlamFrameProcessor::LogDfxLine(const DfxSample &sample) const {
  const std::string line = m_ctx.aliases.jsonDiagnostics
                               ? BuildJsonDfxLine(sample)
                               : BuildTextDfxLine(sample);
  Logger::Logf(Logger::INFO, "%s", line.c_str());
  std::fprintf(stderr, "%s\n", line.c_str());
}

void SlamFrameProcessor::UpdateDfxAverages(const DfxSample &sample) {
  m_state.smoothedAcquireMs =
      UpdateEma(m_state.smoothedAcquireMs, sample.timing.acquireMs);
  m_state.smoothedSlamMs =
      UpdateEma(m_state.smoothedSlamMs, sample.timing.slamMs);
  m_state.smoothedTotalMs =
      UpdateEma(m_state.smoothedTotalMs, sample.timing.totalMs);
}

std::string SlamFrameProcessor::BuildJsonDfxLine(
    const DfxSample &sample) const {
  std::ostringstream out;
  out << std::fixed << std::setprecision(3);
  out << "{\"tag\":\"slam_dfx\",";
  AppendDfxJsonCore(out, sample);
  AppendDfxJsonBackend(out, sample);
  AppendDfxJsonVisual(out, sample);
  AppendDfxJsonFrameStats(out, sample);
  AppendDfxJsonTiming(out, sample);
  out << "}";
  return out.str();
}

std::string SlamFrameProcessor::BuildTextDfxLine(
    const DfxSample &sample) const {
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << "[slam_dfx]";
  AppendDfxTextCore(out, sample);
  AppendDfxTextBackend(out, sample);
  AppendDfxTextVisual(out, sample);
  AppendDfxTextFrameStats(out, sample);
  AppendDfxTextTiming(out, sample);
  return out.str();
}

void SlamFrameProcessor::AppendDfxJsonCore(std::ostream &out,
                                           const DfxSample &sample) const {
  const auto &output = sample.slamOutput;
  const auto &published = sample.published;
  out << "\"frame\":" << static_cast<unsigned long long>(output.frameId)
      << ",\"state\":" << published.trackingState
      << ",\"quality\":" << static_cast<int>(sample.poseResult.quality)
      << ",\"pose_valid\":" << (sample.poseResult.poseEstimate.valid ? 1 : 0)
      << ",\"reset_counter\":"
      << static_cast<unsigned>(published.effectiveResetCounter)
      << ",\"reset_map_count\":"
      << static_cast<unsigned>(published.effectiveResetMapCount)
      << ",\"imu_count\":" << sample.slamInput.imu.size()
      << ",\"feat_left\":" << output.leftFeatures.size()
      << ",\"feat_right\":" << output.rightFeatures.size()
      << ",\"points\":" << published.pointCount;
}

void SlamFrameProcessor::AppendDfxJsonBackend(std::ostream &out,
                                              const DfxSample &sample) const {
  const auto &output = sample.slamOutput;
  out << ",\"track_points\":" << output.trackedMapPointCount
      << ",\"local_points\":" << output.localMapPointCount
      << ",\"close_points\":" << output.closeMapPointCount
      << ",\"inliers\":" << output.matchesInliers
      << ",\"orb_frame_id\":" << static_cast<unsigned long long>(output.orbFrameId)
      << ",\"ref_kf\":" << static_cast<long long>(output.referenceKeyFrameId)
      << ",\"last_kf\":" << static_cast<long long>(output.lastKeyFrameId)
      << ",\"last_kf_frame\":"
      << static_cast<long long>(output.lastKeyFrameFrameId)
      << ",\"keyframes_in_map\":" << output.keyFramesInMap
      << ",\"stereo_feature_init_frame\":" << output.stereoFeatureInitFrameId
      << ",\"stereo_feature_injected\":"
      << (output.stereoFeatureInjected ? 1 : 0)
      << ",\"stereo_feature_bootstrap\":"
      << (output.stereoFeatureBootstrap ? 1 : 0)
      << ",\"stereo_feature_stabilizing\":"
      << (output.stereoFeatureStabilizing ? 1 : 0)
      << ",\"realtime_pose_gate\":" << (output.realtimePoseQualityGate ? 1 : 0)
      << ",\"raw_pose_step_m\":" << output.rawPoseStepMeters
      << ",\"gated_pose_step_m\":" << output.gatedPoseStepMeters;
}

void SlamFrameProcessor::AppendDfxJsonVisual(std::ostream &out,
                                             const DfxSample &sample) const {
  const auto &output = sample.slamOutput;
  out << ",\"visual_feature_used\":"
      << (output.usedVisualFeatureFrontend ? 1 : 0)
      << ",\"visual_feature_stereo_weak\":"
      << (sample.visualFeatureStereoWeak ? 1 : 0)
      << ",\"visual_feature_raw_left\":" << output.visualFeatureRawLeftCount
      << ",\"visual_feature_raw_right\":" << output.visualFeatureRawRightCount
      << ",\"visual_feature_match_stereo\":"
      << output.visualFeatureMatchedStereoCount
      << ",\"visual_feature_injected_left\":"
      << output.visualFeatureInjectedLeftCount
      << ",\"visual_feature_injected_right\":"
      << output.visualFeatureInjectedRightCount
      << ",\"visual_feature_lg_every_n\":" << output.visualFeatureMatchEveryN
      << ",\"visual_feature_prepare_ms\":" << output.visualFeaturePrepareMs
      << ",\"visual_feature_input_ms\":" << output.visualFeatureInputMs
      << ",\"visual_feature_forward_ms\":" << output.visualFeatureForwardMs
      << ",\"visual_feature_frontend_ms\":" << output.visualFeatureFrontendMs
      << ",\"visual_feature_match_ms\":"
      << output.visualFeatureStereoMatchMs
      << ",\"visual_feature_total_ms\":" << output.visualFeatureTotalMs
      << ",\"orb_track_ms\":" << output.orbTrackMs
      << ",\"orb_extract_ms\":" << output.orbExtractMs
      << ",\"orb_stereo_ms\":" << output.orbStereoMatchMs;
}

void SlamFrameProcessor::AppendDfxJsonFrameStats(
    std::ostream &out, const DfxSample &sample) const {
  const auto &frame = sample.frame;
  const auto &output = sample.slamOutput;
  out << ",\"local_mapping_wait_ms\":" << output.localMappingWaitMs
      << ",\"local_mapping_wait_timeout_ms\":"
      << output.localMappingWaitTimeoutMs
      << ",\"local_mapping_queue_before\":"
      << output.localMappingWaitQueueBefore
      << ",\"local_mapping_queue_after\":" << output.localMappingWaitQueueAfter
      << ",\"local_mapping_accept_before\":"
      << (output.localMappingAcceptingBefore ? 1 : 0)
      << ",\"local_mapping_accept_after\":"
      << (output.localMappingAcceptingAfter ? 1 : 0)
      << ",\"local_mapping_wait_requested\":"
      << (output.localMappingWaitRequested ? 1 : 0)
      << ",\"local_mapping_wait_timeout\":"
      << (output.localMappingWaitTimedOut ? 1 : 0)
      << ",\"visual_feature_image_count\":" << output.visualFeatureImageCount
      << ",\"visual_feature_payload_bytes\":"
      << output.visualFeaturePayloadBytes
      << ",\"pair_dt_ms\":" << static_cast<double>(frame.pairDtMs)
      << ",\"reject_dt_ms\":" << frame.rejectDtMs
      << ",\"pending_left\":" << frame.pendingL
      << ",\"pending_right\":" << frame.pendingR
      << ",\"drop_left\":" << static_cast<unsigned long long>(frame.dropUnpairedL)
      << ",\"drop_right\":"
      << static_cast<unsigned long long>(frame.dropUnpairedR)
      << ",\"rate_drop\":"
      << static_cast<unsigned long long>(m_state.rateLimitedDrops);
}

void SlamFrameProcessor::AppendDfxJsonTiming(
    std::ostream &out, const DfxSample &sample) const {
  const auto &frame = sample.frame;
  const auto &timing = sample.timing;
  out << ",\"img_std_left\":" << frame.stdL
      << ",\"img_std_right\":" << frame.stdR
      << ",\"sharp_left\":" << frame.sharpL
      << ",\"sharp_right\":" << frame.sharpR
      << ",\"gap_ms\":" << frame.frameGapMs
      << ",\"mono_step_ms\":" << frame.monoStepMs
      << ",\"acquire_ms\":" << timing.acquireMs
      << ",\"imu_ms\":" << timing.imuMs
      << ",\"slam_ms\":" << timing.slamMs
      << ",\"cloud_ms\":" << timing.cloudMs
      << ",\"udp_ms\":" << timing.udpMs
      << ",\"post_ms\":" << timing.postMs
      << ",\"live_ms\":" << timing.livePoseMs
      << ",\"publish_ms\":" << timing.publishMs
      << ",\"total_ms\":" << timing.totalMs;
}

void SlamFrameProcessor::AppendDfxTextCore(std::ostream &out,
                                           const DfxSample &sample) const {
  const auto &output = sample.slamOutput;
  const auto &published = sample.published;
  out << " frame=" << static_cast<unsigned long long>(output.frameId)
      << " state=" << published.trackingState
      << " quality=" << static_cast<int>(sample.poseResult.quality)
      << " pose_valid=" << (sample.poseResult.poseEstimate.valid ? 1 : 0)
      << " reset=" << static_cast<unsigned>(published.effectiveResetCounter)
      << "/" << static_cast<unsigned>(published.effectiveResetMapCount)
      << " imu=" << sample.slamInput.imu.size()
      << " feat=" << output.leftFeatures.size() << "/"
      << output.rightFeatures.size()
      << " points=" << published.pointCount;
}

void SlamFrameProcessor::AppendDfxTextBackend(std::ostream &out,
                                              const DfxSample &sample) const {
  const auto &output = sample.slamOutput;
  out << " track=" << output.trackedMapPointCount
      << " local=" << output.localMapPointCount
      << " close=" << output.closeMapPointCount
      << " inliers=" << output.matchesInliers
      << " orb_frame=" << static_cast<unsigned long long>(output.orbFrameId)
      << " ref_kf=" << static_cast<long long>(output.referenceKeyFrameId)
      << " last_kf=" << static_cast<long long>(output.lastKeyFrameId)
      << " last_kf_frame="
      << static_cast<long long>(output.lastKeyFrameFrameId)
      << " kfs=" << output.keyFramesInMap
      << " stereo_feature=init:" << output.stereoFeatureInitFrameId
      << " injected:" << (output.stereoFeatureInjected ? 1 : 0)
      << " bootstrap:" << (output.stereoFeatureBootstrap ? 1 : 0)
      << " stabilizing:" << (output.stereoFeatureStabilizing ? 1 : 0)
      << " pose_gate=" << (output.realtimePoseQualityGate ? 1 : 0)
      << " raw_step=" << std::setprecision(4) << output.rawPoseStepMeters
      << " gated_step=" << output.gatedPoseStepMeters << std::setprecision(3);
}

void SlamFrameProcessor::AppendDfxTextVisual(std::ostream &out,
                                             const DfxSample &sample) const {
  const auto &output = sample.slamOutput;
  out << " visual_feature="
      << (output.usedVisualFeatureFrontend ? "on" : "off")
      << " stereo_warn=" << (sample.visualFeatureStereoWeak ? "weak" : "ok")
      << " raw=" << output.visualFeatureRawLeftCount << "/"
      << output.visualFeatureRawRightCount
      << " stereo=" << output.visualFeatureMatchedStereoCount
      << " injected=" << output.visualFeatureInjectedLeftCount << "/"
      << output.visualFeatureInjectedRightCount
      << " lg_every_n=" << output.visualFeatureMatchEveryN
      << " visual_feature_ms=prep " << output.visualFeaturePrepareMs
      << " input " << output.visualFeatureInputMs
      << " forward " << output.visualFeatureForwardMs
      << " frontend " << output.visualFeatureFrontendMs
      << " match " << output.visualFeatureStereoMatchMs
      << " total " << output.visualFeatureTotalMs
      << " orb_ms=track " << output.orbTrackMs
      << " extract " << output.orbExtractMs
      << " stereo " << output.orbStereoMatchMs;
}

void SlamFrameProcessor::AppendDfxTextFrameStats(
    std::ostream &out, const DfxSample &sample) const {
  const auto &frame = sample.frame;
  const auto &output = sample.slamOutput;
  out << " local_mapping_wait=" << output.localMappingWaitMs
      << "ms timeout_ms=" << output.localMappingWaitTimeoutMs
      << " queue=" << output.localMappingWaitQueueBefore << "/"
      << output.localMappingWaitQueueAfter
      << " accept=" << (output.localMappingAcceptingBefore ? 1 : 0) << "/"
      << (output.localMappingAcceptingAfter ? 1 : 0)
      << " requested=" << (output.localMappingWaitRequested ? 1 : 0)
      << " timeout=" << (output.localMappingWaitTimedOut ? 1 : 0)
      << " visual_feature_io=" << output.visualFeatureImageCount
      << "img/" << output.visualFeaturePayloadBytes << "bytes"
      << " pair_dt=" << static_cast<double>(frame.pairDtMs)
      << " reject_dt=" << frame.rejectDtMs
      << " pend=" << frame.pendingL << "/" << frame.pendingR
      << " drop=" << static_cast<unsigned long long>(frame.dropUnpairedL)
      << "/" << static_cast<unsigned long long>(frame.dropUnpairedR)
      << " rate_drop="
      << static_cast<unsigned long long>(m_state.rateLimitedDrops);
}

void SlamFrameProcessor::AppendDfxTextTiming(
    std::ostream &out, const DfxSample &sample) const {
  const auto &frame = sample.frame;
  const auto &timing = sample.timing;
  out << " img_std=" << frame.stdL << "/" << frame.stdR
      << " sharp=" << frame.sharpL << "/" << frame.sharpR
      << " timing_ms gap=" << frame.frameGapMs
      << " mono=" << frame.monoStepMs
      << " acquire=" << timing.acquireMs
      << " imu=" << timing.imuMs
      << " slam=" << timing.slamMs
      << " cloud=" << timing.cloudMs
      << " udp=" << timing.udpMs
      << " post=" << timing.postMs
      << " live=" << timing.livePoseMs
      << " publish=" << timing.publishMs
      << " total=" << timing.totalMs;
}

SlamFrameProcessor::StepResult
SlamFrameProcessor::PublishTrackedFrame(TrackedFrame &tracked,
                                        bool &sessionOk) {
  (void)sessionOk;
  PublishedFrame published;
  auto trackedPtr = std::make_shared<TrackedFrame>(std::move(tracked));
  const StepResult publishResult =
      PublishTrackedFrame(std::move(trackedPtr), published);
  if (publishResult != StepResult::Continue || !published.frame) {
    return publishResult;
  }
  const StepResult pointCloudResult = EmitPointCloud(published);
  if (pointCloudResult != StepResult::Continue) {
    return pointCloudResult;
  }
  const StepResult livePoseResult = EmitLivePose(published);
  if (livePoseResult != StepResult::Continue) {
    return livePoseResult;
  }
  const StepResult mavlinkResult = EmitMavlink(published);
  if (mavlinkResult != StepResult::Continue) {
    return mavlinkResult;
  }
  const StepResult udpResult = EmitUdp(published);
  if (udpResult != StepResult::Continue) {
    return udpResult;
  }
  return EmitDfx(published);
}

SlamFrameProcessor::StepResult
SlamFrameProcessor::ProcessPreparedFrame(PreparedFrame &frame,
                                         bool &sessionOk) {
  auto framePtr = std::make_shared<PreparedFrame>(std::move(frame));
  TrackedFrame tracked;
  const StepResult trackResult =
      TrackPreparedFrame(std::move(framePtr), tracked);
  if (trackResult != StepResult::Continue || !tracked.frame) {
    return trackResult;
  }
  return PublishTrackedFrame(tracked, sessionOk);
}

SlamFrameProcessor::StepResult
SlamFrameProcessor::ProcessNextFrame(bool &sessionOk) {
  PreparedFrame frame;
  const StepResult acquireResult = AcquireAndPrepareFrame(sessionOk, frame);
  if (acquireResult != StepResult::Continue || frame.slamInput.frameId == 0) {
    return acquireResult;
  }
  return ProcessPreparedFrame(frame, sessionOk);
}

} // namespace smartdrone::core::application
