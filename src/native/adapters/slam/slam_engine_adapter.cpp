#include "adapters/slam/slam_engine_adapter.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <utility>

#include <sophus/se3.hpp>

#include "adapters/slam/slam_env.h"
#include "adapters/slam/slam_mode_strategy.h"
#include "adapters/slam/slam_pose_utils.h"
#include "core/ports/slam_tracking_state.h"

namespace smartdrone::adapters::slam {

SlamEngineAdapter::SlamEngineAdapter(
    std::unique_ptr<core::ports::ISlamTrackingBackend> backend,
    SlamInputMode inputMode, bool useImu, std::string settingsPath)
    : m_trackingBackend(std::move(backend)),
      m_modeState(std::make_unique<SlamModeSharedState>()),
      m_inputMode(inputMode), m_useImu(useImu),
      m_modeStrategy(CreateSlamModeStrategy(FeatureFrontend::Orb)),
      m_settingsPath(std::move(settingsPath)) {
  m_modeState->LoadStereoCalibration(m_settingsPath);
}

SlamEngineAdapter::~SlamEngineAdapter() = default;

bool SlamEngineAdapter::Start() {
  m_lastStablePose = core::ports::PoseEstimate{};
  m_haveLastStablePose = false;
  m_lastStableTimestampSec = 0.0;
  ResetRealtimeOutputAlignment();
  m_realtimeOutputHaveLastMapId = false;
  m_realtimeOutputLastMapId = 0;
  ResetOutputSmoother();
  m_modeState->ResetTrackingState();
  m_stableVelX = 0.0f;
  m_stableVelY = 0.0f;
  m_stableVelZ = 0.0f;
  return m_trackingBackend && m_trackingBackend->Available();
}

void SlamEngineAdapter::SetOperationMode(core::domain::SlamOperationMode mode) {
  if (!m_trackingBackend) {
    return;
  }
  m_trackingBackend->SetOperationMode(mode);
}

void SlamEngineAdapter::SetFeatureFrontend(FeatureFrontend frontend) {
  if (m_featureFrontend != frontend) {
    m_modeState->ResetTrackingState();
    m_featureFrontend = frontend;
    m_modeStrategy = CreateSlamModeStrategy(m_featureFrontend);
    return;
  }
  if (!m_modeStrategy) {
    m_modeStrategy = CreateSlamModeStrategy(m_featureFrontend);
  }
}

void SlamEngineAdapter::SetStereoVoLoopClosure(bool enabled, float scale,
                                               float relaxation) {
  m_modeState->m_lkLoop.enabled = enabled;
  m_modeState->m_lkLoop.scale = std::clamp(scale, 0.25f, 4.0f);
  m_modeState->m_lkLoop.relaxation = std::clamp(relaxation, 0.0f, 4.0f);
}

void SlamEngineAdapter::SetStereoVoPerFrameAcceleration(
    std::string acceleration) {
  std::transform(
      acceleration.begin(), acceleration.end(), acceleration.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (acceleration.empty()) {
    acceleration = "auto";
  }
  if (m_modeState->m_lkPerFrameAcceleration == acceleration) {
    return;
  }
  m_modeState->m_lkPerFrameAcceleration = std::move(acceleration);
  m_modeState->m_lkPerFrameVpi.reset();
  m_modeState->m_lkPerFrameAccelLogged = false;
}

void SlamEngineAdapter::Stop() {
  m_lastStablePose = core::ports::PoseEstimate{};
  m_haveLastStablePose = false;
  m_lastStableTimestampSec = 0.0;
  ResetRealtimeOutputAlignment();
  m_realtimeOutputHaveLastMapId = false;
  m_realtimeOutputLastMapId = 0;
  ResetOutputSmoother();
  m_modeState->ResetTrackingState();
  m_stableVelX = 0.0f;
  m_stableVelY = 0.0f;
  m_stableVelZ = 0.0f;
  if (m_trackingBackend) {
    m_trackingBackend->Shutdown();
  }
}

bool SlamEngineAdapter::ShutdownAndSaveTrajectoryEuRoC(
    const std::string &path) {
  if (!m_trackingBackend) {
    return false;
  }
  return m_trackingBackend->ShutdownAndSaveTrajectoryEuRoC(path);
}

void SlamEngineAdapter::ResetRealtimeOutputAlignment() {
  m_realtimeOutputFromRawPose = Sophus::SE3f();
  m_realtimeOutputMapContinuityActive = false;
  m_realtimeOutputMapContinuityMapId = 0;
}

void SlamEngineAdapter::ResetOutputSmoother() {
  m_smoothedOutputPose = core::ports::PoseEstimate{};
  m_haveSmoothedOutputPose = false;
  m_smoothedOutputTimestampSec = 0.0;
  m_smoothVelX = 0.0f;
  m_smoothVelY = 0.0f;
  m_smoothVelZ = 0.0f;
}

void SlamEngineAdapter::MaintainRealtimePoseContinuity(
    core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
    int trackingState) {
  pose.valid = poseValid && IsFinitePose(pose);
  if (pose.valid) {
    NormalizePoseQuaternion(pose);
  }

  double dt = kPoseStabilizerDefaultDtSec;
  if (m_haveLastStablePose && timestampSec > m_lastStableTimestampSec) {
    dt = std::clamp(timestampSec - m_lastStableTimestampSec,
                    kPoseStabilizerMinDtSec, kPoseStabilizerMaxDtSec);
  }

  const bool rawIdentity = pose.x == 0.0f && pose.y == 0.0f && pose.z == 0.0f &&
                           pose.qw == 1.0f && pose.qx == 0.0f &&
                           pose.qy == 0.0f && pose.qz == 0.0f;

  if (pose.valid && !rawIdentity &&
      trackingState == core::ports::kSlamTrackingOk) {
    if (m_haveLastStablePose) {
      float measuredVelX =
          (pose.x - m_lastStablePose.x) / static_cast<float>(dt);
      float measuredVelY =
          (pose.y - m_lastStablePose.y) / static_cast<float>(dt);
      float measuredVelZ =
          (pose.z - m_lastStablePose.z) / static_cast<float>(dt);
      ClampVelocityVector(measuredVelX, measuredVelY, measuredVelZ);
      m_stableVelX = (1.0f - kPoseStabilizerVelocityAlpha) * m_stableVelX +
                     kPoseStabilizerVelocityAlpha * measuredVelX;
      m_stableVelY = (1.0f - kPoseStabilizerVelocityAlpha) * m_stableVelY +
                     kPoseStabilizerVelocityAlpha * measuredVelY;
      m_stableVelZ = (1.0f - kPoseStabilizerVelocityAlpha) * m_stableVelZ +
                     kPoseStabilizerVelocityAlpha * measuredVelZ;
    }
    poseValid = true;
    m_lastStablePose = pose;
    m_haveLastStablePose = true;
    m_lastStableTimestampSec = timestampSec;
    return;
  }

  const bool bootstrapFrame =
      !m_haveLastStablePose &&
      trackingState == core::ports::kSlamTrackingNotInitialized &&
      IsFinitePose(pose) && rawIdentity;
  if (bootstrapFrame) {
    pose.valid = true;
    poseValid = true;
    m_lastStablePose = pose;
    m_haveLastStablePose = true;
    m_lastStableTimestampSec = timestampSec;
    return;
  }

  if (!m_haveLastStablePose) {
    poseValid = pose.valid;
    return;
  }

  const bool canPredict =
      !pose.valid || rawIdentity ||
      trackingState == core::ports::kSlamTrackingRecentlyLost ||
      trackingState == core::ports::kSlamTrackingOkKlt;
  if (!canPredict) {
    poseValid = pose.valid;
    return;
  }

  ClampVelocityVector(m_stableVelX, m_stableVelY, m_stableVelZ);
  core::ports::PoseEstimate predicted = m_lastStablePose;
  predicted.x += m_stableVelX * static_cast<float>(dt);
  predicted.y += m_stableVelY * static_cast<float>(dt);
  predicted.z += m_stableVelZ * static_cast<float>(dt);
  predicted.valid = true;
  pose = predicted;
  poseValid = true;
  m_lastStablePose = pose;
  m_lastStableTimestampSec = timestampSec;
  m_stableVelX *= kPoseStabilizerPredictedVelocityDecay;
  m_stableVelY *= kPoseStabilizerPredictedVelocityDecay;
  m_stableVelZ *= kPoseStabilizerPredictedVelocityDecay;
}

void SlamEngineAdapter::GateRealtimePoseQuality(core::ports::SlamOutput &out,
                                                double timestampSec) {
  const bool canBridgeMapReset =
      out.poseValid && out.pose.valid && IsFinitePose(out.pose);
  if (canBridgeMapReset) {
    const bool mapChanged =
        m_realtimeOutputHaveLastMapId && out.mapId != m_realtimeOutputLastMapId;
    if (mapChanged) {
      ResetRealtimeOutputAlignment();
      if (out.usedVisualFeatureFrontend && m_haveLastStablePose &&
          EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_MAP_BRIDGE", true)) {
        const Sophus::SE3f rawPose = PoseEstimateToSe3(out.pose);
        const Sophus::SE3f outputPose = PoseEstimateToSe3(m_lastStablePose);
        m_realtimeOutputFromRawPose = outputPose * rawPose.inverse();
        m_realtimeOutputMapContinuityActive = true;
        m_realtimeOutputMapContinuityMapId = out.mapId;
        out.rawPoseStepMeters =
            PoseTranslationDistance(out.pose, m_lastStablePose);
        out.pose = Se3ToPoseEstimate(m_realtimeOutputFromRawPose * rawPose);
        out.poseValid = true;
        out.pose.valid = true;
        out.trackingState = core::ports::kSlamTrackingRecentlyLost;
        out.realtimePoseQualityGate = true;
        out.gatedPoseStepMeters =
            PoseTranslationDistance(out.pose, m_lastStablePose);
        m_realtimeOutputHaveLastMapId = true;
        m_realtimeOutputLastMapId = out.mapId;
        return;
      }
    }
    m_realtimeOutputHaveLastMapId = true;
    m_realtimeOutputLastMapId = out.mapId;
  } else if (!m_realtimeOutputHaveLastMapId) {
    m_realtimeOutputHaveLastMapId = true;
    m_realtimeOutputLastMapId = out.mapId;
  }

  if (m_realtimeOutputMapContinuityActive && out.poseValid && out.pose.valid &&
      IsFinitePose(out.pose)) {
    const core::ports::PoseEstimate rawPose = out.pose;
    out.pose = Se3ToPoseEstimate(m_realtimeOutputFromRawPose *
                                 PoseEstimateToSe3(rawPose));
    out.poseValid = true;
    out.pose.valid = true;
    if (m_haveLastStablePose) {
      out.rawPoseStepMeters =
          PoseTranslationDistance(rawPose, m_lastStablePose);
      out.gatedPoseStepMeters =
          PoseTranslationDistance(out.pose, m_lastStablePose);
    }
    if (out.trackingState != core::ports::kSlamTrackingOk) {
      out.trackingState = core::ports::kSlamTrackingRecentlyLost;
    }
    out.realtimePoseQualityGate = true;
    return;
  }

  const bool canMeasurePoseStep = out.poseValid && out.pose.valid &&
                                  IsFinitePose(out.pose) &&
                                  m_haveLastStablePose;
  if (canMeasurePoseStep) {
    out.rawPoseStepMeters = PoseTranslationDistance(out.pose, m_lastStablePose);
    out.gatedPoseStepMeters = out.rawPoseStepMeters;
  }
  if (canMeasurePoseStep && out.usedVisualFeatureFrontend &&
      out.trackingState == core::ports::kSlamTrackingOk &&
      EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_RESET_GUARD", false)) {
    const float resetJumpMax =
        EnvFloatValueClamped("SMART_DRONE_REALTIME_POSE_RESET_GUARD_MAX_STEP_M",
                             0.75f, 0.05f, 10.0f);
    if (out.rawPoseStepMeters > resetJumpMax) {
      const Sophus::SE3f rawPose = PoseEstimateToSe3(out.pose);
      const Sophus::SE3f outputPose = PoseEstimateToSe3(m_lastStablePose);
      m_realtimeOutputFromRawPose = outputPose * rawPose.inverse();
      m_realtimeOutputMapContinuityActive = true;
      m_realtimeOutputMapContinuityMapId = out.mapId;
      out.pose = Se3ToPoseEstimate(m_realtimeOutputFromRawPose * rawPose);
      out.poseValid = true;
      out.trackingState = core::ports::kSlamTrackingRecentlyLost;
      out.realtimePoseQualityGate = true;
      out.gatedPoseStepMeters =
          PoseTranslationDistance(out.pose, m_lastStablePose);
      return;
    }
  }
  if (!EnvFlagEnabled("SMART_DRONE_SP_LG_REALTIME_QUALITY_GATE", false)) {
    return;
  }
  if (!canMeasurePoseStep) {
    return;
  }
  const bool gateDuringStabilizing =
      EnvFlagEnabled("SMART_DRONE_SP_LG_REALTIME_GATE_STABILIZING", false);
  if (!out.usedVisualFeatureFrontend || !out.stereoFeatureInjected ||
      (!gateDuringStabilizing && out.stereoFeatureStabilizing) ||
      out.trackingState != core::ports::kSlamTrackingOk) {
    return;
  }

  const int minInliers = EnvIntValueClamped(
      "SMART_DRONE_SP_LG_REALTIME_GATE_MIN_INLIERS", 120, 1, 100000);
  const int minTracked = EnvIntValueClamped(
      "SMART_DRONE_SP_LG_REALTIME_GATE_MIN_TRACKED_MAP", 140, 1, 100000);
  const float maxStep =
      EnvFloatValueClamped("SMART_DRONE_SP_LG_REALTIME_GATE_MAX_STEP_M",
                           kPoseStabilizerMaxStepMeters, 0.005f, 0.25f);
  const bool weakTracking =
      out.matchesInliers < minInliers ||
      static_cast<int>(out.trackedMapPointCount) < minTracked;
  const float rawStep = out.rawPoseStepMeters;
  const bool gateAllSuperPointLightGlue =
      EnvFlagEnabled("SMART_DRONE_SP_LG_REALTIME_GATE_ALL", false);
  if (!gateAllSuperPointLightGlue && !weakTracking) {
    return;
  }

  core::ports::PoseEstimate gated = out.pose;
  const std::string gateMode =
      EnvStringValue("SMART_DRONE_SP_LG_REALTIME_GATE_MODE", "step");
  if (gateMode == "innovation" || gateMode == "predict" ||
      gateMode == "prediction") {
    double dt = kPoseStabilizerDefaultDtSec;
    if (timestampSec > m_lastStableTimestampSec) {
      dt = std::clamp(timestampSec - m_lastStableTimestampSec,
                      kPoseStabilizerMinDtSec, kPoseStabilizerMaxDtSec);
    }

    float vx = m_stableVelX;
    float vy = m_stableVelY;
    float vz = m_stableVelZ;
    ClampVelocityVector(vx, vy, vz);

    core::ports::PoseEstimate predicted = m_lastStablePose;
    predicted.x += vx * static_cast<float>(dt);
    predicted.y += vy * static_cast<float>(dt);
    predicted.z += vz * static_cast<float>(dt);
    predicted.valid = true;

    const float maxInnovation =
        EnvFloatValueClamped("SMART_DRONE_SP_LG_REALTIME_GATE_MAX_INNOVATION_M",
                             maxStep, 0.005f, 0.50f);
    const float innovation = PoseTranslationDistance(out.pose, predicted);
    if (innovation <= maxInnovation) {
      return;
    }

    const float scale = maxInnovation / std::max(innovation, 1.0e-6f);
    gated.x = predicted.x + (out.pose.x - predicted.x) * scale;
    gated.y = predicted.y + (out.pose.y - predicted.y) * scale;
    gated.z = predicted.z + (out.pose.z - predicted.z) * scale;
    if (EnvFlagEnabled("SMART_DRONE_SP_LG_REALTIME_GATE_LIMIT_ROTATION",
                       false)) {
      LimitPoseRotationStep(predicted, gated, kPoseStabilizerMaxRotStepDeg);
    }
    gated.valid = true;

    out.pose = gated;
    out.poseValid = true;
    out.realtimePoseQualityGate = true;
    out.gatedPoseStepMeters =
        PoseTranslationDistance(out.pose, m_lastStablePose);
    return;
  }

  if (rawStep <= maxStep) {
    return;
  }

  const float scale = maxStep / std::max(rawStep, 1.0e-6f);
  gated.x = m_lastStablePose.x + (out.pose.x - m_lastStablePose.x) * scale;
  gated.y = m_lastStablePose.y + (out.pose.y - m_lastStablePose.y) * scale;
  gated.z = m_lastStablePose.z + (out.pose.z - m_lastStablePose.z) * scale;
  LimitPoseRotationStep(m_lastStablePose, gated, kPoseStabilizerMaxRotStepDeg);
  gated.valid = true;

  out.pose = gated;
  out.poseValid = true;
  out.realtimePoseQualityGate = true;
  out.gatedPoseStepMeters = PoseTranslationDistance(out.pose, m_lastStablePose);
}

void SlamEngineAdapter::StabilizeOutputPose(core::ports::PoseEstimate &pose,
                                            bool &poseValid,
                                            double timestampSec,
                                            int trackingState) {
  pose.valid = poseValid && IsFinitePose(pose);
  if (pose.valid) {
    NormalizePoseQuaternion(pose);
  }

  double dt = kPoseStabilizerDefaultDtSec;
  if (m_haveSmoothedOutputPose && timestampSec > m_smoothedOutputTimestampSec) {
    dt = std::clamp(timestampSec - m_smoothedOutputTimestampSec,
                    kPoseStabilizerMinDtSec, kPoseStabilizerMaxDtSec);
  }

  if (!pose.valid) {
    if (!m_haveSmoothedOutputPose) {
      poseValid = false;
      return;
    }
    pose = m_smoothedOutputPose;
    poseValid = true;
    m_smoothedOutputTimestampSec = timestampSec;
    m_smoothVelX *= kPoseStabilizerPredictedVelocityDecay;
    m_smoothVelY *= kPoseStabilizerPredictedVelocityDecay;
    m_smoothVelZ *= kPoseStabilizerPredictedVelocityDecay;
    return;
  }

  const bool rawIdentity = IsIdentityPose(pose);
  if (!m_haveSmoothedOutputPose) {
    m_smoothedOutputPose = pose;
    m_haveSmoothedOutputPose = true;
    m_smoothedOutputTimestampSec = timestampSec;
    poseValid = true;
    return;
  }

  if (rawIdentity) {
    ClampVelocityVector(m_smoothVelX, m_smoothVelY, m_smoothVelZ);
    pose = m_smoothedOutputPose;
    pose.x += m_smoothVelX * static_cast<float>(dt);
    pose.y += m_smoothVelY * static_cast<float>(dt);
    pose.z += m_smoothVelZ * static_cast<float>(dt);
    pose.valid = true;
    poseValid = true;
    m_smoothVelX *= kPoseStabilizerPredictedVelocityDecay;
    m_smoothVelY *= kPoseStabilizerPredictedVelocityDecay;
    m_smoothVelZ *= kPoseStabilizerPredictedVelocityDecay;
    m_smoothedOutputPose = pose;
    m_smoothedOutputTimestampSec = timestampSec;
    return;
  }

  const std::string smootherMode =
      EnvStringValue("SMART_DRONE_POSE_STABILIZER_MODE", "guard");
  if (smootherMode == "alpha_beta" || smootherMode == "alphabeta" ||
      smootherMode == "ab") {
    const float alpha = EnvFloatValueClamped(
        "SMART_DRONE_POSE_STABILIZER_ALPHA", 0.80f, 0.01f, 1.0f);
    const float beta = EnvFloatValueClamped("SMART_DRONE_POSE_STABILIZER_BETA",
                                            0.03f, 0.0f, 1.0f);
    const float maxInnovation = EnvFloatValueClamped(
        "SMART_DRONE_POSE_STABILIZER_MAX_INNOVATION_M", 0.09f, 0.005f, 1.0f);
    const float maxSpeed =
        EnvFloatValueClamped("SMART_DRONE_POSE_STABILIZER_MAX_SPEED_MPS",
                             kPoseStabilizerMaxSpeedMps, 0.1f, 20.0f);
    const float maxStep =
        EnvFloatValueClamped("SMART_DRONE_POSE_STABILIZER_MAX_STEP_M",
                             kPoseStabilizerMaxStepMeters, 0.005f, 1.0f);

    core::ports::PoseEstimate predicted = m_smoothedOutputPose;
    predicted.x += m_smoothVelX * static_cast<float>(dt);
    predicted.y += m_smoothVelY * static_cast<float>(dt);
    predicted.z += m_smoothVelZ * static_cast<float>(dt);
    predicted.valid = true;

    float innovationX = pose.x - predicted.x;
    float innovationY = pose.y - predicted.y;
    float innovationZ = pose.z - predicted.z;
    const float innovationNorm =
        std::sqrt(innovationX * innovationX + innovationY * innovationY +
                  innovationZ * innovationZ);
    if (innovationNorm > maxInnovation && innovationNorm > 1.0e-6f) {
      const float scale = maxInnovation / innovationNorm;
      innovationX *= scale;
      innovationY *= scale;
      innovationZ *= scale;
    }

    core::ports::PoseEstimate smoothed = pose;
    smoothed.x = predicted.x + alpha * innovationX;
    smoothed.y = predicted.y + alpha * innovationY;
    smoothed.z = predicted.z + alpha * innovationZ;

    const float step = PoseTranslationDistance(smoothed, m_smoothedOutputPose);
    if (step > maxStep && step > 1.0e-6f) {
      const float scale = maxStep / step;
      smoothed.x = m_smoothedOutputPose.x +
                   (smoothed.x - m_smoothedOutputPose.x) * scale;
      smoothed.y = m_smoothedOutputPose.y +
                   (smoothed.y - m_smoothedOutputPose.y) * scale;
      smoothed.z = m_smoothedOutputPose.z +
                   (smoothed.z - m_smoothedOutputPose.z) * scale;
    }
    LimitPoseRotationStep(m_smoothedOutputPose, smoothed,
                          kPoseStabilizerMaxRotStepDeg);
    smoothed.valid = true;

    m_smoothVelX += beta * innovationX / static_cast<float>(dt);
    m_smoothVelY += beta * innovationY / static_cast<float>(dt);
    m_smoothVelZ += beta * innovationZ / static_cast<float>(dt);
    const float speed =
        std::sqrt(m_smoothVelX * m_smoothVelX + m_smoothVelY * m_smoothVelY +
                  m_smoothVelZ * m_smoothVelZ);
    if (speed > maxSpeed && speed > 1.0e-6f) {
      const float scale = maxSpeed / speed;
      m_smoothVelX *= scale;
      m_smoothVelY *= scale;
      m_smoothVelZ *= scale;
    }

    pose = smoothed;
    poseValid = true;
    m_smoothedOutputPose = pose;
    m_haveSmoothedOutputPose = true;
    m_smoothedOutputTimestampSec = timestampSec;
    return;
  }

  const float rawStep = PoseTranslationDistance(pose, m_smoothedOutputPose);
  const float maxGuardStep = EnvFloatValueClamped(
      "SMART_DRONE_POSE_STABILIZER_MAX_STEP_M", 0.22f, 0.02f, 2.0f);
  const float maxSpeed =
      EnvFloatValueClamped("SMART_DRONE_POSE_STABILIZER_MAX_SPEED_MPS",
                           kPoseStabilizerMaxSpeedMps, 0.1f, 20.0f);
  const bool rawPoseStuck =
      trackingState != core::ports::kSlamTrackingOk && rawStep < 1.0e-5f;
  if (rawPoseStuck || rawStep > maxGuardStep) {
    ClampVelocityVector(m_smoothVelX, m_smoothVelY, m_smoothVelZ);
    core::ports::PoseEstimate guarded = m_smoothedOutputPose;
    guarded.x += m_smoothVelX * static_cast<float>(dt);
    guarded.y += m_smoothVelY * static_cast<float>(dt);
    guarded.z += m_smoothVelZ * static_cast<float>(dt);
    const float predictedStep =
        PoseTranslationDistance(guarded, m_smoothedOutputPose);
    const float maxPredictedStep =
        std::min(maxSpeed * static_cast<float>(dt), maxGuardStep);
    if (predictedStep > maxPredictedStep && predictedStep > 1.0e-6f) {
      const float scale = maxPredictedStep / predictedStep;
      guarded.x =
          m_smoothedOutputPose.x + (guarded.x - m_smoothedOutputPose.x) * scale;
      guarded.y =
          m_smoothedOutputPose.y + (guarded.y - m_smoothedOutputPose.y) * scale;
      guarded.z =
          m_smoothedOutputPose.z + (guarded.z - m_smoothedOutputPose.z) * scale;
    }
    guarded.valid = true;
    pose = guarded;
    poseValid = true;
    m_smoothVelX *= kPoseStabilizerPredictedVelocityDecay;
    m_smoothVelY *= kPoseStabilizerPredictedVelocityDecay;
    m_smoothVelZ *= kPoseStabilizerPredictedVelocityDecay;
  } else {
    float measuredVelX =
        (pose.x - m_smoothedOutputPose.x) / static_cast<float>(dt);
    float measuredVelY =
        (pose.y - m_smoothedOutputPose.y) / static_cast<float>(dt);
    float measuredVelZ =
        (pose.z - m_smoothedOutputPose.z) / static_cast<float>(dt);
    ClampVelocityVector(measuredVelX, measuredVelY, measuredVelZ);
    const float measuredSpeed =
        std::sqrt(measuredVelX * measuredVelX + measuredVelY * measuredVelY +
                  measuredVelZ * measuredVelZ);
    if (measuredSpeed > maxSpeed && measuredSpeed > 1.0e-6f) {
      const float scale = maxSpeed / measuredSpeed;
      measuredVelX *= scale;
      measuredVelY *= scale;
      measuredVelZ *= scale;
    }
    m_smoothVelX = (1.0f - kPoseStabilizerVelocityAlpha) * m_smoothVelX +
                   kPoseStabilizerVelocityAlpha * measuredVelX;
    m_smoothVelY = (1.0f - kPoseStabilizerVelocityAlpha) * m_smoothVelY +
                   kPoseStabilizerVelocityAlpha * measuredVelY;
    m_smoothVelZ = (1.0f - kPoseStabilizerVelocityAlpha) * m_smoothVelZ +
                   kPoseStabilizerVelocityAlpha * measuredVelZ;
    poseValid = true;
  }

  if (poseValid && pose.valid) {
    m_smoothedOutputPose = pose;
    m_haveSmoothedOutputPose = true;
    m_smoothedOutputTimestampSec = timestampSec;
  }
}

} // namespace smartdrone::adapters::slam
