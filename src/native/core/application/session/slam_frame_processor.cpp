#include "core/application/session/slam_frame_processor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "common/logger.h"

namespace smartdrone::core::application {

namespace {

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

Sophus::SE3f BuildBodyToCamFromRuntimeOverride(float tx, float ty, float tz,
                                               float rollDeg, float pitchDeg,
                                               float yawDeg) {
  constexpr float kDegToRad = 0.017453292519943295769f;
  const float rollRad = rollDeg * kDegToRad;
  const float pitchRad = pitchDeg * kDegToRad;
  const float yawRad = yawDeg * kDegToRad;
  const Eigen::AngleAxisf rollRotation(rollRad, Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitchRotation(pitchRad, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf yawRotation(yawRad, Eigen::Vector3f::UnitZ());
  const Eigen::Quaternionf q = yawRotation * pitchRotation * rollRotation;
  return Sophus::SE3f(Sophus::SO3f(q), Eigen::Vector3f(tx, ty, tz));
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

SlamFrameProcessor::StepResult
SlamFrameProcessor::AcquireAndPrepareFrame(bool &sessionOk,
                                           PreparedFrame &frame) {
  frame = PreparedFrame{};
  frame.frameStartTp = std::chrono::steady_clock::now();
  const auto configuredSlamMode =
      static_cast<smartdrone::core::domain::SlamOperationMode>(
          m_ctx.tuning.slamOperationMode.load(std::memory_order_relaxed));
  if (configuredSlamMode != m_state.requestedSlamMode) {
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
                << smartdrone::core::domain::ToString(
                       m_state.effectiveSlamMode);
    }
    std::cerr << "\n";
    if (m_state.requestedSlamMode ==
            smartdrone::core::domain::SlamOperationMode::Relocalization ||
        m_state.requestedSlamMode ==
            smartdrone::core::domain::SlamOperationMode::TrackingOnly) {
      std::cerr << "[slam] note: requested mode currently maps to backend "
                   "localization-only mode\n";
    }
    m_ctx.livePose.SetSlamMode(
        ToRuntimeSlamModeValue(m_state.effectiveSlamMode));
  }

  const int configuredSlamInputFps = m_ctx.perceptionPipeline.ClampTargetFps(
      m_ctx.tuning.slamInputFps.load(std::memory_order_relaxed));
  const uint8_t configuredFrontendValue =
      m_ctx.tuning.featureFrontend.load(std::memory_order_relaxed);
  const FeatureFrontend effectiveFrontend =
      ParseRuntimeFeatureFrontendValue(configuredFrontendValue);
  const bool dpvoEpgPacing =
      m_ctx.aliases.slamBackend == SlamBackend::DpvoTensorRt &&
      DpvoEpgPacingEnabled();
  const int effectiveSlamInputFps =
      dpvoEpgPacing ? configuredSlamInputFps
                    : ComputeAdaptiveSlamInputFps(configuredSlamInputFps,
                                                  m_ctx.aliases.fps,
                                                  m_state.smoothedSlamMs);
  m_state.adaptiveSlamInputFps = effectiveSlamInputFps;
  const int visualFeatureLoadSheddingLevel =
      ComputeVisualFeatureLoadSheddingLevel(
          m_state.visualFeatureLoadSheddingLevel,
          IsVisualFeatureLightGlueFrontend(effectiveFrontend),
          m_state.lastTrackingUsable, m_state.smoothedSlamMs,
          m_state.smoothedTotalMs);
  m_state.visualFeatureLoadSheddingLevel = visualFeatureLoadSheddingLevel;
  const auto [visualFeatureBudgetWidth, visualFeatureBudgetHeight] =
      ComputeVisualFeatureInputBudget(m_ctx.aliases.visualFeatureInputMaxWidth,
                                      m_ctx.aliases.visualFeatureInputMaxHeight,
                                      visualFeatureLoadSheddingLevel);
  if (m_ctx.slamControl != nullptr) {
    m_ctx.slamControl->SetVisualFeatureInputSizeLimit(
        visualFeatureBudgetWidth, visualFeatureBudgetHeight);
    m_ctx.slamControl->SetFeatureFrontend(effectiveFrontend);
  }
  if (effectiveFrontend != m_state.lastAppliedFeatureFrontend) {
    std::cerr << "[slam] effective_feature_frontend="
              << FeatureFrontendName(effectiveFrontend) << " requested_value="
              << static_cast<unsigned>(configuredFrontendValue)
              << " visual_feature_gate=mode_selected"
              << " prev_tracking_state=" << m_state.lastTrackingState
              << " prev_tracking_usable="
              << (m_state.lastTrackingUsable ? 1 : 0) << "\n";
    m_state.lastAppliedFeatureFrontend = effectiveFrontend;
  }
  if (configuredSlamInputFps != m_state.lastLoggedConfiguredSlamInputFps ||
      effectiveSlamInputFps != m_state.lastLoggedEffectiveSlamInputFps) {
    std::cerr << "[slam] configured_input_fps=" << configuredSlamInputFps
              << " effective_input_fps=" << effectiveSlamInputFps
              << " camera_fps=" << m_ctx.aliases.fps << " frame_drop="
              << (effectiveSlamInputFps < m_ctx.aliases.fps ? "enabled"
                                                            : "disabled")
              << " dpvo_epg_pacing=" << (dpvoEpgPacing ? 1 : 0)
              << " smoothed_slam_ms=" << m_state.smoothedSlamMs
              << " smoothed_total_ms=" << m_state.smoothedTotalMs << "\n";
    m_state.lastLoggedConfiguredSlamInputFps = configuredSlamInputFps;
    m_state.lastLoggedEffectiveSlamInputFps = effectiveSlamInputFps;
  }
  if (IsVisualFeatureLightGlueFrontend(effectiveFrontend) &&
      visualFeatureLoadSheddingLevel !=
          m_state.lastLoggedVisualFeatureLoadSheddingLevel) {
    std::cerr << "[slam] visual_feature_load_profile="
              << DescribeVisualFeatureLoadSheddingLevel(
                     visualFeatureLoadSheddingLevel)
              << " frontend=" << FeatureFrontendName(effectiveFrontend)
              << " input_max=" << visualFeatureBudgetWidth << "x"
              << visualFeatureBudgetHeight
              << " tracking_usable=" << (m_state.lastTrackingUsable ? 1 : 0)
              << " smoothed_slam_ms=" << m_state.smoothedSlamMs
              << " smoothed_total_ms=" << m_state.smoothedTotalMs << "\n";
    m_state.lastLoggedVisualFeatureLoadSheddingLevel =
        visualFeatureLoadSheddingLevel;
  }

  const auto acquireStartTp = std::chrono::steady_clock::now();
  StereoBatch stereoBatch{};
  const StereoAcquireStatus acquireStatus =
      m_ctx.perceptionPipeline.AcquireNextStereoBatch(
          m_ctx.cameraProvider, effectiveSlamInputFps, 1000, stereoBatch,
          &m_ctx.frameTimingTracker);
  const auto acquireEndTp = std::chrono::steady_clock::now();
  if (acquireStatus == StereoAcquireStatus::Timeout) {
    return StepResult::Continue;
  }
  if (acquireStatus == StereoAcquireStatus::CameraUnhealthy) {
    std::cerr << "[slam] camera pipeline unhealthy, aborting session\n";
    sessionOk = false;
    return StepResult::SessionAbort;
  }
  if (acquireStatus == StereoAcquireStatus::DroppedByRateLimiter) {
    ++m_state.rateLimitedDrops;
    return StepResult::Continue;
  }

  auto &L = stereoBatch.stereo.left;
  auto &R = stereoBatch.stereo.right;
  const bool sendImage = m_ctx.tuning.sendImage.load(std::memory_order_relaxed);
  const bool sendFeature =
      m_ctx.tuning.sendFeature.load(std::memory_order_relaxed);
  const bool sendMap = m_ctx.tuning.sendMap.load(std::memory_order_relaxed);
  const auto cameraDiag = m_ctx.cameraProvider.GetDiagnostics();
  const int64_t pairDtMs = cameraDiag.lastPairDtMs;
  const double rejectDtMs =
      static_cast<double>(cameraDiag.lastRejectDtUs) / 1000.0;
  const uint64_t dropUnpairedL = cameraDiag.droppedUnpairedL;
  const uint64_t dropUnpairedR = cameraDiag.droppedUnpairedR;
  const size_t pendingL = cameraDiag.pendingL;
  const size_t pendingR = cameraDiag.pendingR;
  const int64_t captureTimestampNs = m_ctx.monoMode
                                         ? static_cast<int64_t>(R.timestampNs)
                                         : stereoBatch.captureTimestampNs;
  const int64_t logicalFrameTimestampNs =
      m_ctx.monoMode ? captureTimestampNs : stereoBatch.logicalFrameTimestampNs;
  const double frameTime = static_cast<double>(captureTimestampNs) * 1e-9;
  const double frameGapMs =
      (m_state.lastPublishedFrameNs != 0)
          ? static_cast<double>(logicalFrameTimestampNs -
                                m_state.lastPublishedFrameNs) *
                1e-6
          : 0.0;
  if (frameGapMs > 0.0 && effectiveSlamInputFps > 0) {
    const double expectedFrameGapMs =
        1000.0 / static_cast<double>(effectiveSlamInputFps);
    if (frameGapMs > expectedFrameGapMs) {
      constexpr int64_t kGapWarnMinIntervalNs = 1000000000LL; // 1s
      if (m_state.lastFrameGapWarnLogNs == 0 ||
          (logicalFrameTimestampNs - m_state.lastFrameGapWarnLogNs) >=
              kGapWarnMinIntervalNs) {
        std::cerr << "[slam_gap_warn] frame_gap_ms=" << frameGapMs
                  << " expected_gap_ms=" << expectedFrameGapMs
                  << " target_input_fps=" << effectiveSlamInputFps
                  << " configured_input_fps=" << configuredSlamInputFps
                  << " camera_fps=" << m_ctx.aliases.fps
                  << " frame=" << stereoBatch.frameId << "\n";
        m_state.lastFrameGapWarnLogNs = logicalFrameTimestampNs;
      }
    }
  }
  const double monoStepMs =
      static_cast<double>(stereoBatch.monotonicFrameStepNs) * 1e-6;
  double meanL = 0.0, stdL = 0.0, meanR = 0.0, stdR = 0.0;
  ComputeImageStats(L.gray, meanL, stdL);
  ComputeImageStats(R.gray, meanR, stdR);
  const double sharpL = ComputeSharpnessLaplacianVar(L.gray);
  const double sharpR = ComputeSharpnessLaplacianVar(R.gray);

  smartdrone::core::ports::SlamInputBatch slamInput{};
  const auto imuStartTp = std::chrono::steady_clock::now();
  if (m_ctx.useImu && m_state.lastFrameNs != 0) {
    slamInput.imu =
        m_ctx.imuProvider.PopWindow(m_state.lastFrameNs, captureTimestampNs);
    ImuWindowValidation imuWindow{};
    const double prevFrameTime =
        static_cast<double>(m_state.lastFrameNs) * 1e-9;
    const double expectedImuDtSec = 1.0 / std::max(1, m_ctx.aliases.imuHz);
    const bool imuWindowOk = SanitizeImuWindow(
        slamInput.imu, prevFrameTime, frameTime, expectedImuDtSec, imuWindow);
    if (!imuWindowOk) {
      return StepResult::Continue;
    }
    if (slamInput.imu.empty() && !m_ctx.aliases.allowEmptyImu) {
      return StepResult::Continue;
    }
  }
  const auto imuEndTp = std::chrono::steady_clock::now();
  m_state.lastFrameNs = captureTimestampNs;

  PrepareStereoPairForSlam(stereoBatch.stereo, meanL, stdL, meanR, stdR, sharpL,
                           sharpR, m_ctx.aliases.slamLowLightEnhance,
                           slamInput.stereo);
  slamInput.frameId = stereoBatch.frameId;
  slamInput.captureTimestampNs = captureTimestampNs;
  slamInput.frameTimeSec = frameTime;
  const bool debugRightOnlyFeatures = m_ctx.aliases.debugRightOnlyFeatures;
  const bool extractFeatures =
      sendFeature || m_state.requestedSlamMode ==
                         smartdrone::core::domain::SlamOperationMode::Auto;
  const bool updatePointCloud =
      !debugRightOnlyFeatures && sendMap &&
      (captureTimestampNs - m_state.lastPointCloudUpdateNs) >=
          kPointCloudUpdateIntervalNs;

  frame.acquireStartTp = acquireStartTp;
  frame.acquireEndTp = acquireEndTp;
  frame.imuStartTp = imuStartTp;
  frame.imuEndTp = imuEndTp;
  frame.stereoBatch = std::move(stereoBatch);
  frame.slamInput = std::move(slamInput);
  frame.configuredSlamInputFps = configuredSlamInputFps;
  frame.effectiveSlamInputFps = effectiveSlamInputFps;
  frame.sendImage = sendImage;
  frame.sendFeature = sendFeature;
  frame.sendMap = sendMap;
  frame.pairDtMs = pairDtMs;
  frame.rejectDtMs = rejectDtMs;
  frame.dropUnpairedL = dropUnpairedL;
  frame.dropUnpairedR = dropUnpairedR;
  frame.pendingL = pendingL;
  frame.pendingR = pendingR;
  frame.captureTimestampNs = captureTimestampNs;
  frame.logicalFrameTimestampNs = logicalFrameTimestampNs;
  frame.frameTime = frameTime;
  frame.frameGapMs = frameGapMs;
  frame.monoStepMs = monoStepMs;
  frame.meanL = meanL;
  frame.stdL = stdL;
  frame.meanR = meanR;
  frame.stdR = stdR;
  frame.sharpL = sharpL;
  frame.sharpR = sharpR;
  frame.debugRightOnlyFeatures = debugRightOnlyFeatures;
  frame.extractFeatures = extractFeatures;
  frame.updatePointCloud = updatePointCloud;
  return StepResult::Continue;
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
  auto &frame = *tracked->frame;
  auto &stereoBatch = frame.stereoBatch;
  auto &slamInput = frame.slamInput;
  auto &L = stereoBatch.stereo.left;
  auto &R = stereoBatch.stereo.right;
  auto &slamOutput = tracked->slamOutput;
  const auto frameStartTp = frame.frameStartTp;
  const auto acquireStartTp = frame.acquireStartTp;
  const auto acquireEndTp = frame.acquireEndTp;
  const auto imuStartTp = frame.imuStartTp;
  const auto imuEndTp = frame.imuEndTp;
  const auto slamStartTp = tracked->slamStartTp;
  const auto slamEndTp = tracked->slamEndTp;
  const bool sendMap = frame.sendMap;
  const int64_t pairDtMs = frame.pairDtMs;
  const double rejectDtMs = frame.rejectDtMs;
  const uint64_t dropUnpairedL = frame.dropUnpairedL;
  const uint64_t dropUnpairedR = frame.dropUnpairedR;
  const size_t pendingL = frame.pendingL;
  const size_t pendingR = frame.pendingR;
  const int64_t captureTimestampNs = frame.captureTimestampNs;
  const int64_t logicalFrameTimestampNs = frame.logicalFrameTimestampNs;
  const double frameGapMs = frame.frameGapMs;
  const double monoStepMs = frame.monoStepMs;
  const double stdL = frame.stdL;
  const double stdR = frame.stdR;
  const double sharpL = frame.sharpL;
  const double sharpR = frame.sharpR;
  const bool debugRightOnlyFeatures = frame.debugRightOnlyFeatures;
  const int state = debugRightOnlyFeatures ? ports::kSlamTrackingLost
                                           : slamOutput.trackingState;
  const bool trackingUsable =
      !debugRightOnlyFeatures && ports::IsSlamTrackingPoseUsable(state);
  m_state.lastTrackingState = state;
  m_state.lastTrackingUsable = trackingUsable;
  const unsigned long mapId = debugRightOnlyFeatures ? 0UL : slamOutput.mapId;
  const bool mapIdChanged =
      mapId != PosePostprocessor::ContinuityMapper::kInvalidMapId &&
      mapId != m_state.lastRawMapId;
  if (mapIdChanged) {
    m_state.lastRawMapId = mapId;
  }

  Sophus::SE3f twcRaw =
      m_state.haveLastValidTwcRaw ? m_state.lastValidTwcRaw : Sophus::SE3f();
  if (!debugRightOnlyFeatures && slamOutput.poseValid) {
    const Eigen::Quaternionf rawQ(slamOutput.pose.qw, slamOutput.pose.qx,
                                  slamOutput.pose.qy, slamOutput.pose.qz);
    twcRaw = Sophus::SE3f(Sophus::SO3f(rawQ),
                          Eigen::Vector3f(slamOutput.pose.x, slamOutput.pose.y,
                                          slamOutput.pose.z));
    m_state.lastValidTwcRaw = twcRaw;
    m_state.haveLastValidTwcRaw = true;
  }

  const auto postStartTp = std::chrono::steady_clock::now();
  bool useStereoBodyExtrinsics = m_ctx.stereoBodyExtrinsics.loaded;
  Sophus::SE3f stereoBodyExtrinsics = m_ctx.stereoBodyExtrinsics.Tbc;
  if (!m_ctx.useImu && !m_ctx.monoMode &&
      m_ctx.tuning.useCustomTbc.load(std::memory_order_relaxed)) {
    const float pitchDeg =
        m_ctx.tuning.tbcPitchDeg.load(std::memory_order_relaxed);
    if (m_ctx.stereoBodyExtrinsics.loaded) {
      // Runtime pitch is an incremental gimbal motion on top of the
      // calibrated body->camera extrinsics. Right-multiplication makes a
      // forward-facing camera sweep toward downward view as pitch grows.
      stereoBodyExtrinsics =
          m_ctx.stereoBodyExtrinsics.Tbc * BuildBodyToCamPitchDelta(pitchDeg);
    } else {
      const float tx = m_ctx.tuning.tbcTx.load(std::memory_order_relaxed);
      const float ty = m_ctx.tuning.tbcTy.load(std::memory_order_relaxed);
      const float tz = m_ctx.tuning.tbcTz.load(std::memory_order_relaxed);
      const float rollDeg =
          m_ctx.tuning.tbcRollDeg.load(std::memory_order_relaxed);
      const float yawDeg =
          m_ctx.tuning.tbcYawDeg.load(std::memory_order_relaxed);
      stereoBodyExtrinsics = BuildBodyToCamFromRuntimeOverride(
          tx, ty, tz, rollDeg, pitchDeg, yawDeg);
    }
    useStereoBodyExtrinsics = true;
  }

  const auto poseResult = m_ctx.posePostprocessor.ProcessPose(
      twcRaw, m_ctx.useImu, trackingUsable, state, mapId,
      useStereoBodyExtrinsics, stereoBodyExtrinsics,
      m_state.stereoReferencePoseSet, m_state.stereoReferencePose,
      captureTimestampNs, m_ctx.mav);
  if (!m_ctx.useImu && !m_ctx.monoMode && slamOutput.poseValid &&
      (slamOutput.frameId % kPoseAxisLogEveryNFrames) == 0) {
    const Eigen::Vector3f camT = twcRaw.translation();
    const auto &dbg = poseResult.debug;
    const Eigen::Vector3f frdT(poseResult.poseEstimate.x,
                               poseResult.poseEstimate.y,
                               poseResult.poseEstimate.z);
    std::cerr << "[pose_axis] frame=" << slamOutput.frameId
              << " cam_t=" << camT.x() << "," << camT.y() << "," << camT.z()
              << " body_t=" << dbg.bodyX << "," << dbg.bodyY << "," << dbg.bodyZ
              << " local_t=" << dbg.localX << "," << dbg.localY << ","
              << dbg.localZ << " frd_t=" << frdT.x() << "," << frdT.y() << ","
              << frdT.z() << " tbc=" << (dbg.stereoExtrinsicsApplied ? 1 : 0)
              << " ref=" << (dbg.referenceApplied ? 1 : 0)
              << " tracking=" << (trackingUsable ? 1 : 0)
              << " q=" << poseResult.poseEstimate.qw << ","
              << poseResult.poseEstimate.qx << "," << poseResult.poseEstimate.qy
              << "," << poseResult.poseEstimate.qz << "\n";
  }
  const uint8_t effectiveResetCounter = ComposeResetCounter(
      m_state.sessionResetCounterBase, poseResult.resetCounter);
  const uint16_t effectiveResetMapCount = ComposeResetMapCount(
      m_state.sessionResetMapCountBase, poseResult.resetMapCount);
  const auto postEndTp = std::chrono::steady_clock::now();

  if (m_state.requestedSlamMode ==
      smartdrone::core::domain::SlamOperationMode::Auto) {
    const auto autoEffectiveMode = m_ctx.autoSlamModeController.Observe(
        trackingUsable, poseResult.quality, frameGapMs,
        slamOutput.leftFeatures.size(), slamOutput.rightFeatures.size());
    if (autoEffectiveMode != m_state.effectiveSlamMode) {
      m_state.effectiveSlamMode = autoEffectiveMode;
      if (m_ctx.slamControl != nullptr) {
        m_ctx.slamControl->SetOperationMode(m_state.effectiveSlamMode);
      }
      m_ctx.livePose.SetSlamMode(
          ToRuntimeSlamModeValue(m_state.effectiveSlamMode));
      std::cerr << "[slam_auto] effective_mode -> "
                << smartdrone::core::domain::ToString(m_state.effectiveSlamMode)
                << " quality=" << static_cast<int>(poseResult.quality)
                << " state=" << state
                << " featL=" << slamOutput.leftFeatures.size()
                << " featR=" << slamOutput.rightFeatures.size()
                << " frame_gap_ms=" << frameGapMs << "\n";
    }
  }

  published.frame = std::move(tracked);
  published.poseResult = poseResult;
  published.cloudStartTp = postEndTp;
  published.cloudEndTp = postEndTp;
  published.udpStartTp = postEndTp;
  published.udpEndTp = postEndTp;
  published.postStartTp = postStartTp;
  published.postEndTp = postEndTp;
  published.livePoseStartTp = postEndTp;
  published.livePoseEndTp = postEndTp;
  published.publishStartTp = postEndTp;
  published.publishEndTp = postEndTp;
  published.pointCount = sendMap ? slamOutput.pointCloudXyz.size() / 3 : 0;
  published.trackingState = state;
  published.trackingUsable = trackingUsable;
  published.effectiveResetCounter = effectiveResetCounter;
  published.effectiveResetMapCount = effectiveResetMapCount;
  return StepResult::Continue;
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
  m_ctx.livePose.UpdatePose(
      RUNTIME_MODE_SLAM, static_cast<uint8_t>(published.trackingState),
      published.effectiveResetCounter, published.effectiveResetMapCount,
      poseResult.alignedPose,
      poseResult.quality == smartdrone::core::ports::PoseQuality::Good
          ? OdomQualityMode::GOOD
      : poseResult.quality == smartdrone::core::ports::PoseQuality::Weak
          ? OdomQualityMode::WEAK
          : OdomQualityMode::LOST,
      livePoseValid);
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
  auto &tracked = *published.frame;
  auto &frame = *tracked.frame;
  auto &slamInput = frame.slamInput;
  auto &slamOutput = tracked.slamOutput;
  const auto frameStartTp = frame.frameStartTp;
  const auto acquireStartTp = frame.acquireStartTp;
  const auto acquireEndTp = frame.acquireEndTp;
  const auto imuStartTp = frame.imuStartTp;
  const auto imuEndTp = frame.imuEndTp;
  const auto slamStartTp = tracked.slamStartTp;
  const auto slamEndTp = tracked.slamEndTp;
  const auto cloudStartTp = published.cloudStartTp;
  const auto cloudEndTp = published.cloudEndTp;
  const auto udpStartTp = published.udpStartTp;
  const auto udpEndTp = published.udpEndTp;
  const auto postStartTp = published.postStartTp;
  const auto postEndTp = published.postEndTp;
  const auto livePoseStartTp = published.livePoseStartTp;
  const auto livePoseEndTp = published.livePoseEndTp;
  const auto publishStartTp = published.publishStartTp;
  const auto publishEndTp = published.publishEndTp;
  const auto &poseResult = published.poseResult;
  const int state = published.trackingState;
  const uint8_t effectiveResetCounter = published.effectiveResetCounter;
  const uint16_t effectiveResetMapCount = published.effectiveResetMapCount;
  const int64_t logicalFrameTimestampNs = frame.logicalFrameTimestampNs;
  const int64_t pairDtMs = frame.pairDtMs;
  const double rejectDtMs = frame.rejectDtMs;
  const uint64_t dropUnpairedL = frame.dropUnpairedL;
  const uint64_t dropUnpairedR = frame.dropUnpairedR;
  const size_t pendingL = frame.pendingL;
  const size_t pendingR = frame.pendingR;
  const double frameGapMs = frame.frameGapMs;
  const double monoStepMs = frame.monoStepMs;
  const double stdL = frame.stdL;
  const double stdR = frame.stdR;
  const double sharpL = frame.sharpL;
  const double sharpR = frame.sharpR;
  const size_t pointCount = published.pointCount;

  ++m_state.frameIndex;
  m_state.lastPublishedFrameNs = logicalFrameTimestampNs;
  const bool visualFeatureStereoWeak =
      slamOutput.usedVisualFeatureFrontend && !m_ctx.monoMode &&
      slamOutput.visualFeatureMatchedStereoCount <
          kVisualFeatureStereoWeakMatchThreshold;
  const double totalMs = DurationMs(frameStartTp, publishEndTp);
  const bool slamDfxPeriodic =
      (kSlamDfxLogEveryNFrames > 0) &&
      ((m_state.frameIndex % kSlamDfxLogEveryNFrames) == 0);
  const bool slamDfxAbnormal =
      !poseResult.poseEstimate.valid || state <= 0 || totalMs > 80.0 ||
      slamOutput.leftFeatures.empty() || slamOutput.rightFeatures.empty() ||
      visualFeatureStereoWeak;
  if (slamDfxPeriodic || slamDfxAbnormal) {
    char dfxLine[3200];
    if (m_ctx.aliases.jsonDiagnostics) {
      std::snprintf(
          dfxLine, sizeof(dfxLine),
          "{\"tag\":\"slam_dfx\",\"frame\":%llu,\"state\":%d,\"quality\":%d,"
          "\"pose_valid\":%d,"
          "\"reset_counter\":%u,\"reset_map_count\":%u,"
          "\"imu_count\":%zu,\"feat_left\":%zu,\"feat_right\":%zu,\"points\":%"
          "zu,"
          "\"track_points\":%u,\"local_points\":%u,\"close_points\":%u,"
          "\"inliers\":%d,"
          "\"orb_frame_id\":%llu,\"ref_kf\":%lld,\"last_kf\":%lld,\"last_kf_"
          "frame\":%lld,"
          "\"keyframes_in_map\":%u,\"stereo_feature_init_frame\":%d,"
          "\"stereo_feature_injected\":%d,"
          "\"stereo_feature_bootstrap\":%d,\"stereo_feature_stabilizing\":%d,"
          "\"realtime_pose_gate\":%d,\"raw_pose_step_m\":%.4f,\"gated_pose_"
          "step_m\":%.4f,"
          "\"visual_feature_used\":%d,\"visual_feature_stereo_weak\":%d,"
          "\"visual_feature_raw_left\":%d,\"visual_feature_raw_right\":%d,"
          "\"visual_feature_match_stereo\":%d,"
          "\"visual_feature_injected_left\":%d,"
          "\"visual_feature_injected_right\":%d,"
          "\"visual_feature_lg_every_n\":%d,"
          "\"visual_feature_prepare_ms\":%.3f,"
          "\"visual_feature_input_ms\":%.3f,"
          "\"visual_feature_forward_ms\":%.3f,"
          "\"visual_feature_frontend_ms\":%.3f,"
          "\"visual_feature_match_ms\":%.3f,"
          "\"visual_feature_total_ms\":%.3f,"
          "\"orb_track_ms\":%.3f,\"orb_extract_ms\":%.3f,\"orb_stereo_ms\":%."
          "3f,"
          "\"local_mapping_wait_ms\":%.3f,\"local_mapping_wait_timeout_ms\":%d,"
          "\"local_mapping_queue_before\":%d,\"local_mapping_queue_after\":%d,"
          "\"local_mapping_accept_before\":%d,\"local_mapping_accept_after\":%"
          "d,"
          "\"local_mapping_wait_requested\":%d,\"local_mapping_wait_timeout\":%"
          "d,"
          "\"visual_feature_image_count\":%u,"
          "\"visual_feature_payload_bytes\":%u,"
          "\"pair_dt_ms\":%.3f,\"reject_dt_ms\":%.3f,\"pending_left\":%zu,"
          "\"pending_right\":%zu,"
          "\"drop_left\":%llu,\"drop_right\":%llu,\"rate_drop\":%llu,"
          "\"img_std_left\":%.2f,\"img_std_right\":%.2f,\"sharp_left\":%.2f,"
          "\"sharp_right\":%.2f,"
          "\"gap_ms\":%.3f,\"mono_step_ms\":%.3f,"
          "\"acquire_ms\":%.3f,\"imu_ms\":%.3f,\"slam_ms\":%.3f,\"cloud_ms\":%."
          "3f,\"udp_ms\":%.3f,"
          "\"post_ms\":%.3f,\"live_ms\":%.3f,\"publish_ms\":%.3f,\"total_ms\":%"
          ".3f}",
          static_cast<unsigned long long>(slamOutput.frameId), state,
          static_cast<int>(poseResult.quality),
          poseResult.poseEstimate.valid ? 1 : 0,
          static_cast<unsigned>(effectiveResetCounter),
          static_cast<unsigned>(effectiveResetMapCount), slamInput.imu.size(),
          slamOutput.leftFeatures.size(), slamOutput.rightFeatures.size(),
          pointCount, slamOutput.trackedMapPointCount,
          slamOutput.localMapPointCount, slamOutput.closeMapPointCount,
          slamOutput.matchesInliers,
          static_cast<unsigned long long>(slamOutput.orbFrameId),
          static_cast<long long>(slamOutput.referenceKeyFrameId),
          static_cast<long long>(slamOutput.lastKeyFrameId),
          static_cast<long long>(slamOutput.lastKeyFrameFrameId),
          slamOutput.keyFramesInMap, slamOutput.stereoFeatureInitFrameId,
          slamOutput.stereoFeatureInjected ? 1 : 0,
          slamOutput.stereoFeatureBootstrap ? 1 : 0,
          slamOutput.stereoFeatureStabilizing ? 1 : 0,
          slamOutput.realtimePoseQualityGate ? 1 : 0,
          slamOutput.rawPoseStepMeters, slamOutput.gatedPoseStepMeters,
          slamOutput.usedVisualFeatureFrontend ? 1 : 0,
          visualFeatureStereoWeak ? 1 : 0, slamOutput.visualFeatureRawLeftCount,
          slamOutput.visualFeatureRawRightCount,
          slamOutput.visualFeatureMatchedStereoCount,
          slamOutput.visualFeatureInjectedLeftCount,
          slamOutput.visualFeatureInjectedRightCount,
          slamOutput.visualFeatureMatchEveryN,
          slamOutput.visualFeaturePrepareMs, slamOutput.visualFeatureInputMs,
          slamOutput.visualFeatureForwardMs, slamOutput.visualFeatureFrontendMs,
          slamOutput.visualFeatureStereoMatchMs,
          slamOutput.visualFeatureTotalMs, slamOutput.orbTrackMs,
          slamOutput.orbExtractMs, slamOutput.orbStereoMatchMs,
          slamOutput.localMappingWaitMs, slamOutput.localMappingWaitTimeoutMs,
          slamOutput.localMappingWaitQueueBefore,
          slamOutput.localMappingWaitQueueAfter,
          slamOutput.localMappingAcceptingBefore ? 1 : 0,
          slamOutput.localMappingAcceptingAfter ? 1 : 0,
          slamOutput.localMappingWaitRequested ? 1 : 0,
          slamOutput.localMappingWaitTimedOut ? 1 : 0,
          slamOutput.visualFeatureImageCount,
          slamOutput.visualFeaturePayloadBytes, static_cast<double>(pairDtMs),
          rejectDtMs, pendingL, pendingR,
          static_cast<unsigned long long>(dropUnpairedL),
          static_cast<unsigned long long>(dropUnpairedR),
          static_cast<unsigned long long>(m_state.rateLimitedDrops), stdL, stdR,
          sharpL, sharpR, frameGapMs, monoStepMs,
          DurationMs(acquireStartTp, acquireEndTp),
          DurationMs(imuStartTp, imuEndTp), DurationMs(slamStartTp, slamEndTp),
          DurationMs(cloudStartTp, cloudEndTp),
          DurationMs(udpStartTp, udpEndTp), DurationMs(postStartTp, postEndTp),
          DurationMs(livePoseStartTp, livePoseEndTp),
          DurationMs(publishStartTp, publishEndTp), totalMs);
    } else {
      std::snprintf(
          dfxLine, sizeof(dfxLine),
          "[slam_dfx] frame=%llu state=%d quality=%d pose_valid=%d reset=%u/%u "
          "imu=%zu feat=%zu/%zu points=%zu track=%u local=%u close=%u "
          "inliers=%d "
          "orb_frame=%llu ref_kf=%lld last_kf=%lld last_kf_frame=%lld kfs=%u "
          "stereo_feature=init:%d injected:%d bootstrap:%d stabilizing:%d "
          "pose_gate=%d raw_step=%.4f gated_step=%.4f "
          "visual_feature=%s stereo_warn=%s raw=%d/%d stereo=%d "
          "injected=%d/%d "
          "lg_every_n=%d "
          "visual_feature_ms=prep %.3f input %.3f forward %.3f frontend %.3f "
          "match %.3f total %.3f "
          "orb_ms=track %.3f extract %.3f stereo %.3f "
          "local_mapping_wait=%.3fms timeout_ms=%d queue=%d/%d accept=%d/%d "
          "requested=%d timeout=%d "
          "visual_feature_io=%uimg/%ubytes "
          "pair_dt=%.3f reject_dt=%.3f pend=%zu/%zu drop=%llu/%llu "
          "rate_drop=%llu "
          "img_std=%.2f/%.2f sharp=%.2f/%.2f "
          "timing_ms gap=%.3f mono=%.3f acquire=%.3f imu=%.3f slam=%.3f "
          "cloud=%.3f udp=%.3f post=%.3f "
          "live=%.3f publish=%.3f total=%.3f",
          static_cast<unsigned long long>(slamOutput.frameId), state,
          static_cast<int>(poseResult.quality),
          poseResult.poseEstimate.valid ? 1 : 0,
          static_cast<unsigned>(effectiveResetCounter),
          static_cast<unsigned>(effectiveResetMapCount), slamInput.imu.size(),
          slamOutput.leftFeatures.size(), slamOutput.rightFeatures.size(),
          pointCount, slamOutput.trackedMapPointCount,
          slamOutput.localMapPointCount, slamOutput.closeMapPointCount,
          slamOutput.matchesInliers,
          static_cast<unsigned long long>(slamOutput.orbFrameId),
          static_cast<long long>(slamOutput.referenceKeyFrameId),
          static_cast<long long>(slamOutput.lastKeyFrameId),
          static_cast<long long>(slamOutput.lastKeyFrameFrameId),
          slamOutput.keyFramesInMap, slamOutput.stereoFeatureInitFrameId,
          slamOutput.stereoFeatureInjected ? 1 : 0,
          slamOutput.stereoFeatureBootstrap ? 1 : 0,
          slamOutput.stereoFeatureStabilizing ? 1 : 0,
          slamOutput.realtimePoseQualityGate ? 1 : 0,
          slamOutput.rawPoseStepMeters, slamOutput.gatedPoseStepMeters,
          slamOutput.usedVisualFeatureFrontend ? "on" : "off",
          visualFeatureStereoWeak ? "weak" : "ok",
          slamOutput.visualFeatureRawLeftCount,
          slamOutput.visualFeatureRawRightCount,
          slamOutput.visualFeatureMatchedStereoCount,
          slamOutput.visualFeatureInjectedLeftCount,
          slamOutput.visualFeatureInjectedRightCount,
          slamOutput.visualFeatureMatchEveryN,
          slamOutput.visualFeaturePrepareMs, slamOutput.visualFeatureInputMs,
          slamOutput.visualFeatureForwardMs, slamOutput.visualFeatureFrontendMs,
          slamOutput.visualFeatureStereoMatchMs,
          slamOutput.visualFeatureTotalMs, slamOutput.orbTrackMs,
          slamOutput.orbExtractMs, slamOutput.orbStereoMatchMs,
          slamOutput.localMappingWaitMs, slamOutput.localMappingWaitTimeoutMs,
          slamOutput.localMappingWaitQueueBefore,
          slamOutput.localMappingWaitQueueAfter,
          slamOutput.localMappingAcceptingBefore ? 1 : 0,
          slamOutput.localMappingAcceptingAfter ? 1 : 0,
          slamOutput.localMappingWaitRequested ? 1 : 0,
          slamOutput.localMappingWaitTimedOut ? 1 : 0,
          slamOutput.visualFeatureImageCount,
          slamOutput.visualFeaturePayloadBytes, static_cast<double>(pairDtMs),
          rejectDtMs, pendingL, pendingR,
          static_cast<unsigned long long>(dropUnpairedL),
          static_cast<unsigned long long>(dropUnpairedR),
          static_cast<unsigned long long>(m_state.rateLimitedDrops), stdL, stdR,
          sharpL, sharpR, frameGapMs, monoStepMs,
          DurationMs(acquireStartTp, acquireEndTp),
          DurationMs(imuStartTp, imuEndTp), DurationMs(slamStartTp, slamEndTp),
          DurationMs(cloudStartTp, cloudEndTp),
          DurationMs(udpStartTp, udpEndTp), DurationMs(postStartTp, postEndTp),
          DurationMs(livePoseStartTp, livePoseEndTp),
          DurationMs(publishStartTp, publishEndTp), totalMs);
    }
    Logger::Logf(Logger::INFO, "%s", dfxLine);
    std::fprintf(stderr, "%s\n", dfxLine);
  }

  m_state.smoothedAcquireMs = UpdateEma(
      m_state.smoothedAcquireMs, DurationMs(acquireStartTp, acquireEndTp));
  m_state.smoothedSlamMs =
      UpdateEma(m_state.smoothedSlamMs, DurationMs(slamStartTp, slamEndTp));
  m_state.smoothedTotalMs = UpdateEma(m_state.smoothedTotalMs, totalMs);

  return StepResult::Continue;
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
