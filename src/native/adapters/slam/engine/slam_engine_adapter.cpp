#include "adapters/slam/engine/slam_engine_adapter.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <string>
#include <utility>

#include <sophus/se3.hpp>

#include "adapters/slam/engine/slam_env.h"
#include "adapters/slam/engine/slam_mode_strategy.h"
#include "adapters/slam/engine/slam_pose_utils.h"
#include "core/ports/slam_tracking_state.h"

namespace SmartDrone::Adapters::Slam {

SlamEngineAdapter::SlamEngineAdapter(
    std::unique_ptr<Core::Ports::ISlamTrackingBackend> backend,
    SlamInputMode inputMode, bool useImu, std::string settingsPath)
    : m_trackingBackend(std::move(backend)),
      m_modeState(std::make_unique<SlamModeSharedState>()),
      m_inputMode(inputMode), m_useImu(useImu),
      m_modeStrategy(CreateSlamModeStrategy(FeatureFrontend::Orb)),
      m_settingsPath(std::move(settingsPath))
{
    m_modeState->LoadStereoCalibration(m_settingsPath);
}

SlamEngineAdapter::~SlamEngineAdapter() = default;

bool SlamEngineAdapter::Start()
{
    m_lastStablePose = Core::Ports::PoseEstimate{};
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

void SlamEngineAdapter::SetOperationMode(Core::Domain::SlamOperationMode mode)
{
    if (!m_trackingBackend) {
        return;
    }
    m_trackingBackend->SetOperationMode(mode);
}

void SlamEngineAdapter::SetFeatureFrontend(FeatureFrontend frontend)
{
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

void SlamEngineAdapter::SetVisualFeatureFrontend(
    Core::Ports::IVisualFeatureFrontend *frontend)
{
    m_modeState->m_visualFeatureFrontend = frontend;
}

void SlamEngineAdapter::SetVisualFeatureInputSizeLimit(int maxWidth,
                                                       int maxHeight)
{
    m_modeState->m_visualFeatureInputMaxWidth = std::max(0, maxWidth);
    m_modeState->m_visualFeatureInputMaxHeight = std::max(0, maxHeight);
}

void SlamEngineAdapter::SetStereoVoLoopClosure(bool enabled, float scale,
                                               float relaxation)
{
    m_modeState->m_lkLoop.enabled = enabled;
    m_modeState->m_lkLoop.scale = std::clamp(scale, 0.25f, 4.0f);
    m_modeState->m_lkLoop.relaxation = std::clamp(relaxation, 0.0f, 4.0f);
}

void SlamEngineAdapter::SetStereoVoPerFrameAcceleration(
    std::string acceleration)
{
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

void SlamEngineAdapter::RequestBackendStop()
{
    if (m_trackingBackend) {
        m_trackingBackend->RequestBackendStop();
    }
}

bool SlamEngineAdapter::BackendStopped() const
{
    return !m_trackingBackend || m_trackingBackend->BackendStopped();
}

void SlamEngineAdapter::StepBackend()
{
    if (m_trackingBackend) {
        m_trackingBackend->StepBackend();
    }
}

void SlamEngineAdapter::Stop()
{
    m_lastStablePose = Core::Ports::PoseEstimate{};
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
    const std::string &path)
{
    if (!m_trackingBackend) {
        return false;
    }
    return m_trackingBackend->ShutdownAndSaveTrajectoryEuRoC(path);
}

void SlamEngineAdapter::ResetRealtimeOutputAlignment()
{
    m_realtimeOutputFromRawPose = Sophus::SE3f();
    m_realtimeOutputMapContinuityActive = false;
    m_realtimeOutputMapContinuityMapId = 0;
}

void SlamEngineAdapter::ResetOutputSmoother()
{
    m_smoothedOutputPose = Core::Ports::PoseEstimate{};
    m_haveSmoothedOutputPose = false;
    m_smoothedOutputTimestampSec = 0.0;
    m_smoothVelX = 0.0f;
    m_smoothVelY = 0.0f;
    m_smoothVelZ = 0.0f;
}

double SlamEngineAdapter::StablePoseDeltaTime(double timestampSec) const
{
    if (!m_haveLastStablePose || timestampSec <= m_lastStableTimestampSec) {
        return POSE_STABILIZER_DEFAULT_DT_SEC;
    }
    return std::clamp(timestampSec - m_lastStableTimestampSec,
                      POSE_STABILIZER_MIN_DT_SEC, POSE_STABILIZER_MAX_DT_SEC);
}

double SlamEngineAdapter::SmoothedPoseDeltaTime(double timestampSec) const
{
    if (!m_haveSmoothedOutputPose ||
        timestampSec <= m_smoothedOutputTimestampSec) {
        return POSE_STABILIZER_DEFAULT_DT_SEC;
    }
    return std::clamp(timestampSec - m_smoothedOutputTimestampSec,
                      POSE_STABILIZER_MIN_DT_SEC, POSE_STABILIZER_MAX_DT_SEC);
}

void SlamEngineAdapter::StoreSmoothedPose(
    const Core::Ports::PoseEstimate &pose, double timestampSec)
{
    m_smoothedOutputPose = pose;
    m_haveSmoothedOutputPose = true;
    m_smoothedOutputTimestampSec = timestampSec;
}

bool SlamEngineAdapter::AcceptStableRealtimePose(
    Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
    double dt, int trackingState)
{
    if (!pose.valid || IsIdentityPose(pose) ||
        trackingState != Core::Ports::SLAM_TRACKING_OK) {
        return false;
    }
    if (m_haveLastStablePose) {
        float measuredVelX =
            (pose.x - m_lastStablePose.x) / static_cast<float>(dt);
        float measuredVelY =
            (pose.y - m_lastStablePose.y) / static_cast<float>(dt);
        float measuredVelZ =
            (pose.z - m_lastStablePose.z) / static_cast<float>(dt);
        ClampVelocityVector(measuredVelX, measuredVelY, measuredVelZ);
        m_stableVelX = (1.0f - POSE_STABILIZER_VELOCITY_ALPHA) * m_stableVelX +
                       POSE_STABILIZER_VELOCITY_ALPHA * measuredVelX;
        m_stableVelY = (1.0f - POSE_STABILIZER_VELOCITY_ALPHA) * m_stableVelY +
                       POSE_STABILIZER_VELOCITY_ALPHA * measuredVelY;
        m_stableVelZ = (1.0f - POSE_STABILIZER_VELOCITY_ALPHA) * m_stableVelZ +
                       POSE_STABILIZER_VELOCITY_ALPHA * measuredVelZ;
    }
    poseValid = true;
    m_lastStablePose = pose;
    m_haveLastStablePose = true;
    m_lastStableTimestampSec = timestampSec;
    return true;
}

bool SlamEngineAdapter::AcceptRealtimeBootstrapPose(
    Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
    int trackingState, bool rawIdentity)
{
    const bool bootstrapFrame =
        !m_haveLastStablePose &&
        trackingState == Core::Ports::SLAM_TRACKING_NOT_INITIALIZED &&
        IsFinitePose(pose) && rawIdentity;
    if (!bootstrapFrame) {
        return false;
    }
    pose.valid = true;
    poseValid = true;
    m_lastStablePose = pose;
    m_haveLastStablePose = true;
    m_lastStableTimestampSec = timestampSec;
    return true;
}

bool SlamEngineAdapter::PredictRealtimePose(
    const RealtimePosePredictionRequest &request)
{
    Core::Ports::PoseEstimate &pose = request.pose;
    bool &poseValid = request.poseValid;
    if (!m_haveLastStablePose) {
        poseValid = pose.valid;
        return true;
    }
    const bool canPredict =
        !pose.valid || request.rawIdentity ||
        request.trackingState == Core::Ports::SLAM_TRACKING_RECENTLY_LOST ||
        request.trackingState == Core::Ports::SLAM_TRACKING_OK_KLT;
    if (!canPredict) {
        poseValid = pose.valid;
        return true;
    }

    ClampVelocityVector(m_stableVelX, m_stableVelY, m_stableVelZ);
    Core::Ports::PoseEstimate predicted = m_lastStablePose;
    predicted.x += m_stableVelX * static_cast<float>(request.dt);
    predicted.y += m_stableVelY * static_cast<float>(request.dt);
    predicted.z += m_stableVelZ * static_cast<float>(request.dt);
    predicted.valid = true;
    pose = predicted;
    poseValid = true;
    m_lastStablePose = pose;
    m_lastStableTimestampSec = request.timestampSec;
    m_stableVelX *= POSE_STABILIZER_PREDICTED_VELOCITY_DECAY;
    m_stableVelY *= POSE_STABILIZER_PREDICTED_VELOCITY_DECAY;
    m_stableVelZ *= POSE_STABILIZER_PREDICTED_VELOCITY_DECAY;
    return true;
}

void SlamEngineAdapter::MaintainRealtimePoseContinuity(
    Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
    int trackingState)
{
    const bool rawFinite = IsFinitePose(pose);
    const bool rawIdentity = rawFinite && HasIdentityPoseValues(pose);
    pose.valid = poseValid && IsFinitePose(pose);
    if (pose.valid) {
        NormalizePoseQuaternion(pose);
    }

    const double dt = StablePoseDeltaTime(timestampSec);
    if (AcceptStableRealtimePose(pose, poseValid, timestampSec, dt,
                                 trackingState)) {
        return;
    }
    if (AcceptRealtimeBootstrapPose(pose, poseValid, timestampSec,
                                    trackingState, rawIdentity)) {
        return;
    }
    (void)PredictRealtimePose(
        {pose, poseValid, timestampSec, dt, trackingState, rawIdentity});
}

bool SlamEngineAdapter::HandleRealtimeMapBridge(Core::Ports::SlamOutput &out)
{
    const bool canBridgeMapReset =
        out.poseValid && out.pose.valid && IsFinitePose(out.pose);
    if (!canBridgeMapReset) {
        if (!m_realtimeOutputHaveLastMapId) {
            m_realtimeOutputHaveLastMapId = true;
            m_realtimeOutputLastMapId = out.mapId;
        }
        return false;
    }

    const bool mapChanged =
        m_realtimeOutputHaveLastMapId && out.mapId != m_realtimeOutputLastMapId;
    if (!mapChanged) {
        m_realtimeOutputHaveLastMapId = true;
        m_realtimeOutputLastMapId = out.mapId;
        return false;
    }

    ResetRealtimeOutputAlignment();
    if (!out.usedVisualFeatureFrontend || !m_haveLastStablePose ||
        !EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_MAP_BRIDGE", true)) {
        m_realtimeOutputHaveLastMapId = true;
        m_realtimeOutputLastMapId = out.mapId;
        return false;
    }

    const Sophus::SE3f rawPose = PoseEstimateToSe3(out.pose);
    const Sophus::SE3f outputPose = PoseEstimateToSe3(m_lastStablePose);
    m_realtimeOutputFromRawPose = outputPose * rawPose.inverse();
    m_realtimeOutputMapContinuityActive = true;
    m_realtimeOutputMapContinuityMapId = out.mapId;
    out.rawPoseStepMeters = PoseTranslationDistance(out.pose, m_lastStablePose);
    out.pose = Se3ToPoseEstimate(m_realtimeOutputFromRawPose * rawPose);
    out.poseValid = true;
    out.pose.valid = true;
    out.trackingState = Core::Ports::SLAM_TRACKING_RECENTLY_LOST;
    out.realtimePoseQualityGate = true;
    out.gatedPoseStepMeters = PoseTranslationDistance(out.pose, m_lastStablePose);
    m_realtimeOutputHaveLastMapId = true;
    m_realtimeOutputLastMapId = out.mapId;
    return true;
}

bool SlamEngineAdapter::ApplyRealtimeMapContinuity(
    Core::Ports::SlamOutput &out)
{
    if (!m_realtimeOutputMapContinuityActive || !out.poseValid ||
        !out.pose.valid || !IsFinitePose(out.pose)) {
        return false;
    }
    const Core::Ports::PoseEstimate rawPose = out.pose;
    out.pose = Se3ToPoseEstimate(m_realtimeOutputFromRawPose *
                                 PoseEstimateToSe3(rawPose));
    out.poseValid = true;
    out.pose.valid = true;
    if (m_haveLastStablePose) {
        out.rawPoseStepMeters = PoseTranslationDistance(rawPose, m_lastStablePose);
        out.gatedPoseStepMeters =
            PoseTranslationDistance(out.pose, m_lastStablePose);
    }
    if (out.trackingState != Core::Ports::SLAM_TRACKING_OK) {
        out.trackingState = Core::Ports::SLAM_TRACKING_RECENTLY_LOST;
    }
    out.realtimePoseQualityGate = true;
    return true;
}

bool SlamEngineAdapter::ApplyRealtimeResetGuard(
    Core::Ports::SlamOutput &out)
{
    const bool canMeasurePoseStep = out.poseValid && out.pose.valid &&
                                    IsFinitePose(out.pose) &&
                                    m_haveLastStablePose;
    if (canMeasurePoseStep) {
        out.rawPoseStepMeters = PoseTranslationDistance(out.pose, m_lastStablePose);
        out.gatedPoseStepMeters = out.rawPoseStepMeters;
    }
    if (!canMeasurePoseStep || !out.usedVisualFeatureFrontend ||
        out.trackingState != Core::Ports::SLAM_TRACKING_OK ||
        !EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_RESET_GUARD", false)) {
        return canMeasurePoseStep;
    }

    const float resetJumpMax =
        EnvFloatValueClamped("SMART_DRONE_REALTIME_POSE_RESET_GUARD_MAX_STEP_M",
                             0.75f, 0.05f, 10.0f);
    if (out.rawPoseStepMeters <= resetJumpMax) {
        return true;
    }
    const Sophus::SE3f rawPose = PoseEstimateToSe3(out.pose);
    const Sophus::SE3f outputPose = PoseEstimateToSe3(m_lastStablePose);
    m_realtimeOutputFromRawPose = outputPose * rawPose.inverse();
    m_realtimeOutputMapContinuityActive = true;
    m_realtimeOutputMapContinuityMapId = out.mapId;
    out.pose = Se3ToPoseEstimate(m_realtimeOutputFromRawPose * rawPose);
    out.poseValid = true;
    out.trackingState = Core::Ports::SLAM_TRACKING_RECENTLY_LOST;
    out.realtimePoseQualityGate = true;
    out.gatedPoseStepMeters = PoseTranslationDistance(out.pose, m_lastStablePose);
    return false;
}

bool SlamEngineAdapter::ShouldApplyRealtimeFeatureGate(
    const Core::Ports::SlamOutput &out) const
{
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_REALTIME_QUALITY_GATE", false)) {
        return false;
    }
    const bool gateDuringStabilizing =
        EnvFlagEnabled("SMART_DRONE_SP_LG_REALTIME_GATE_STABILIZING", false);
    if (!out.usedVisualFeatureFrontend || !out.stereoFeatureInjected ||
        (!gateDuringStabilizing && out.stereoFeatureStabilizing) ||
        out.trackingState != Core::Ports::SLAM_TRACKING_OK) {
        return false;
    }
    const int minInliers = EnvIntValueClamped(
        "SMART_DRONE_SP_LG_REALTIME_GATE_MIN_INLIERS", 120, 1, 100000);
    const int minTracked = EnvIntValueClamped(
        "SMART_DRONE_SP_LG_REALTIME_GATE_MIN_TRACKED_MAP", 140, 1, 100000);
    const bool weakTracking =
        out.matchesInliers < minInliers ||
        static_cast<int>(out.trackedMapPointCount) < minTracked;
    return EnvFlagEnabled("SMART_DRONE_SP_LG_REALTIME_GATE_ALL", false) ||
           weakTracking;
}

void SlamEngineAdapter::ApplyRealtimeInnovationGate(
    Core::Ports::SlamOutput &out, double timestampSec, float maxStep)
{
    const double dt = StablePoseDeltaTime(timestampSec);
    float velocityX = m_stableVelX;
    float velocityY = m_stableVelY;
    float velocityZ = m_stableVelZ;
    ClampVelocityVector(velocityX, velocityY, velocityZ);

    Core::Ports::PoseEstimate predicted = m_lastStablePose;
    predicted.x += velocityX * static_cast<float>(dt);
    predicted.y += velocityY * static_cast<float>(dt);
    predicted.z += velocityZ * static_cast<float>(dt);
    predicted.valid = true;

    const float maxInnovation =
        EnvFloatValueClamped("SMART_DRONE_SP_LG_REALTIME_GATE_MAX_INNOVATION_M",
                             maxStep, 0.005f, 0.50f);
    const float innovation = PoseTranslationDistance(out.pose, predicted);
    if (innovation <= maxInnovation) {
        return;
    }

    const float scale = maxInnovation / std::max(innovation, 1.0e-6f);
    Core::Ports::PoseEstimate gated = out.pose;
    gated.x = predicted.x + (out.pose.x - predicted.x) * scale;
    gated.y = predicted.y + (out.pose.y - predicted.y) * scale;
    gated.z = predicted.z + (out.pose.z - predicted.z) * scale;
    if (EnvFlagEnabled("SMART_DRONE_SP_LG_REALTIME_GATE_LIMIT_ROTATION",
                       false)) {
        LimitPoseRotationStep(predicted, gated, POSE_STABILIZER_MAX_ROT_STEP_DEG);
    }
    gated.valid = true;
    out.pose = gated;
    out.poseValid = true;
    out.realtimePoseQualityGate = true;
    out.gatedPoseStepMeters = PoseTranslationDistance(out.pose, m_lastStablePose);
}

void SlamEngineAdapter::ApplyRealtimeStepGate(Core::Ports::SlamOutput &out,
                                              float maxStep)
{
    const float rawStep = out.rawPoseStepMeters;
    if (rawStep <= maxStep) {
        return;
    }
    const float scale = maxStep / std::max(rawStep, 1.0e-6f);
    Core::Ports::PoseEstimate gated = out.pose;
    gated.x = m_lastStablePose.x + (out.pose.x - m_lastStablePose.x) * scale;
    gated.y = m_lastStablePose.y + (out.pose.y - m_lastStablePose.y) * scale;
    gated.z = m_lastStablePose.z + (out.pose.z - m_lastStablePose.z) * scale;
    LimitPoseRotationStep(m_lastStablePose, gated, POSE_STABILIZER_MAX_ROT_STEP_DEG);
    gated.valid = true;
    out.pose = gated;
    out.poseValid = true;
    out.realtimePoseQualityGate = true;
    out.gatedPoseStepMeters = PoseTranslationDistance(out.pose, m_lastStablePose);
}

void SlamEngineAdapter::GateRealtimePoseQuality(Core::Ports::SlamOutput &out,
                                                double timestampSec)
{
    if (HandleRealtimeMapBridge(out)) {
        return;
    }
    if (ApplyRealtimeMapContinuity(out)) {
        return;
    }
    if (!ApplyRealtimeResetGuard(out)) {
        return;
    }
    if (!ShouldApplyRealtimeFeatureGate(out)) {
        return;
    }

    const float maxStep =
        EnvFloatValueClamped("SMART_DRONE_SP_LG_REALTIME_GATE_MAX_STEP_M",
                             POSE_STABILIZER_MAX_STEP_METERS, 0.005f, 0.25f);
    const std::string gateMode =
        EnvStringValue("SMART_DRONE_SP_LG_REALTIME_GATE_MODE", "step");
    if (gateMode == "innovation" || gateMode == "predict" ||
        gateMode == "prediction") {
        ApplyRealtimeInnovationGate(out, timestampSec, maxStep);
        return;
    }
    ApplyRealtimeStepGate(out, maxStep);
}

bool SlamEngineAdapter::UseAlphaBetaSmoother() const
{
    const std::string smootherMode =
        EnvStringValue("SMART_DRONE_POSE_STABILIZER_MODE", "guard");
    return smootherMode == "alpha_beta" || smootherMode == "alphabeta" ||
           smootherMode == "ab";
}

void SlamEngineAdapter::PredictInvalidSmoothedPose(
    Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec)
{
    if (!m_haveSmoothedOutputPose) {
        poseValid = false;
        return;
    }
    pose = m_smoothedOutputPose;
    poseValid = true;
    m_smoothedOutputTimestampSec = timestampSec;
    m_smoothVelX *= POSE_STABILIZER_PREDICTED_VELOCITY_DECAY;
    m_smoothVelY *= POSE_STABILIZER_PREDICTED_VELOCITY_DECAY;
    m_smoothVelZ *= POSE_STABILIZER_PREDICTED_VELOCITY_DECAY;
}

void SlamEngineAdapter::PredictIdentitySmoothedPose(
    Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
    double dt)
{
    ClampVelocityVector(m_smoothVelX, m_smoothVelY, m_smoothVelZ);
    pose = m_smoothedOutputPose;
    pose.x += m_smoothVelX * static_cast<float>(dt);
    pose.y += m_smoothVelY * static_cast<float>(dt);
    pose.z += m_smoothVelZ * static_cast<float>(dt);
    pose.valid = true;
    poseValid = true;
    m_smoothVelX *= POSE_STABILIZER_PREDICTED_VELOCITY_DECAY;
    m_smoothVelY *= POSE_STABILIZER_PREDICTED_VELOCITY_DECAY;
    m_smoothVelZ *= POSE_STABILIZER_PREDICTED_VELOCITY_DECAY;
    StoreSmoothedPose(pose, timestampSec);
}

Core::Ports::PoseEstimate SlamEngineAdapter::PredictSmoothedPose(
    double dt) const
{
    Core::Ports::PoseEstimate predicted = m_smoothedOutputPose;
    predicted.x += m_smoothVelX * static_cast<float>(dt);
    predicted.y += m_smoothVelY * static_cast<float>(dt);
    predicted.z += m_smoothVelZ * static_cast<float>(dt);
    predicted.valid = true;
    return predicted;
}

void SlamEngineAdapter::ClampInnovationVector(float &innovationX,
                                              float &innovationY,
                                              float &innovationZ,
                                              float maxInnovation) const
{
    const float innovationNorm =
        std::sqrt(innovationX * innovationX + innovationY * innovationY +
                  innovationZ * innovationZ);
    if (innovationNorm <= maxInnovation || innovationNorm <= 1.0e-6f) {
        return;
    }
    const float scale = maxInnovation / innovationNorm;
    innovationX *= scale;
    innovationY *= scale;
    innovationZ *= scale;
}

void SlamEngineAdapter::LimitSmoothedStep(Core::Ports::PoseEstimate &pose,
                                          float maxStep) const
{
    const float step = PoseTranslationDistance(pose, m_smoothedOutputPose);
    if (step <= maxStep || step <= 1.0e-6f) {
        return;
    }
    const float scale = maxStep / step;
    pose.x = m_smoothedOutputPose.x + (pose.x - m_smoothedOutputPose.x) * scale;
    pose.y = m_smoothedOutputPose.y + (pose.y - m_smoothedOutputPose.y) * scale;
    pose.z = m_smoothedOutputPose.z + (pose.z - m_smoothedOutputPose.z) * scale;
}

void SlamEngineAdapter::UpdateAlphaBetaVelocity(
    const AlphaBetaVelocityUpdateRequest &request)
{
    m_smoothVelX +=
        request.beta * request.innovationX / static_cast<float>(request.dt);
    m_smoothVelY +=
        request.beta * request.innovationY / static_cast<float>(request.dt);
    m_smoothVelZ +=
        request.beta * request.innovationZ / static_cast<float>(request.dt);
    ClampVelocityVector(m_smoothVelX, m_smoothVelY, m_smoothVelZ);
    const float speed =
        std::sqrt(m_smoothVelX * m_smoothVelX + m_smoothVelY * m_smoothVelY +
                  m_smoothVelZ * m_smoothVelZ);
    if (speed <= request.maxSpeed || speed <= 1.0e-6f) {
        return;
    }
    const float scale = request.maxSpeed / speed;
    m_smoothVelX *= scale;
    m_smoothVelY *= scale;
    m_smoothVelZ *= scale;
}

void SlamEngineAdapter::ApplyAlphaBetaSmoother(
    Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
    double dt)
{
    const float alpha = EnvFloatValueClamped(
        "SMART_DRONE_POSE_STABILIZER_ALPHA", 0.80f, 0.01f, 1.0f);
    const float beta = EnvFloatValueClamped("SMART_DRONE_POSE_STABILIZER_BETA",
                                            0.03f, 0.0f, 1.0f);
    const float maxInnovation = EnvFloatValueClamped(
        "SMART_DRONE_POSE_STABILIZER_MAX_INNOVATION_M", 0.09f, 0.005f, 1.0f);
    const float maxSpeed =
        EnvFloatValueClamped("SMART_DRONE_POSE_STABILIZER_MAX_SPEED_MPS",
                             POSE_STABILIZER_MAX_SPEED_MPS, 0.1f, 20.0f);
    const float maxStep =
        EnvFloatValueClamped("SMART_DRONE_POSE_STABILIZER_MAX_STEP_M",
                             POSE_STABILIZER_MAX_STEP_METERS, 0.005f, 1.0f);

    const Core::Ports::PoseEstimate predicted = PredictSmoothedPose(dt);
    float innovationX = pose.x - predicted.x;
    float innovationY = pose.y - predicted.y;
    float innovationZ = pose.z - predicted.z;
    ClampInnovationVector(innovationX, innovationY, innovationZ, maxInnovation);

    Core::Ports::PoseEstimate smoothed = pose;
    smoothed.x = predicted.x + alpha * innovationX;
    smoothed.y = predicted.y + alpha * innovationY;
    smoothed.z = predicted.z + alpha * innovationZ;
    LimitSmoothedStep(smoothed, maxStep);
    LimitPoseRotationStep(m_smoothedOutputPose, smoothed,
                          POSE_STABILIZER_MAX_ROT_STEP_DEG);
    smoothed.valid = true;

    UpdateAlphaBetaVelocity(
        {innovationX, innovationY, innovationZ, dt, beta, maxSpeed});
    pose = smoothed;
    poseValid = true;
    StoreSmoothedPose(pose, timestampSec);
}

Core::Ports::PoseEstimate SlamEngineAdapter::PredictGuardedSmoothedPose(
    double dt, float maxGuardStep, float maxSpeed)
{
    ClampVelocityVector(m_smoothVelX, m_smoothVelY, m_smoothVelZ);
    Core::Ports::PoseEstimate guarded = PredictSmoothedPose(dt);
    const float predictedStep =
        PoseTranslationDistance(guarded, m_smoothedOutputPose);
    const float maxPredictedStep =
        std::min(maxSpeed * static_cast<float>(dt), maxGuardStep);
    LimitSmoothedStep(guarded, maxPredictedStep);
    if (predictedStep <= maxPredictedStep || predictedStep <= 1.0e-6f) {
        return guarded;
    }
    return guarded;
}

void SlamEngineAdapter::UpdateGuardMeasuredVelocity(
    const Core::Ports::PoseEstimate &pose, double dt, float maxSpeed)
{
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
    m_smoothVelX = (1.0f - POSE_STABILIZER_VELOCITY_ALPHA) * m_smoothVelX +
                   POSE_STABILIZER_VELOCITY_ALPHA * measuredVelX;
    m_smoothVelY = (1.0f - POSE_STABILIZER_VELOCITY_ALPHA) * m_smoothVelY +
                   POSE_STABILIZER_VELOCITY_ALPHA * measuredVelY;
    m_smoothVelZ = (1.0f - POSE_STABILIZER_VELOCITY_ALPHA) * m_smoothVelZ +
                   POSE_STABILIZER_VELOCITY_ALPHA * measuredVelZ;
}

void SlamEngineAdapter::ApplyGuardSmoother(
    Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
    double dt, int trackingState)
{
    const float rawStep = PoseTranslationDistance(pose, m_smoothedOutputPose);
    const float maxGuardStep = EnvFloatValueClamped(
        "SMART_DRONE_POSE_STABILIZER_MAX_STEP_M", 0.22f, 0.02f, 2.0f);
    const float maxSpeed =
        EnvFloatValueClamped("SMART_DRONE_POSE_STABILIZER_MAX_SPEED_MPS",
                             POSE_STABILIZER_MAX_SPEED_MPS, 0.1f, 20.0f);
    const bool rawPoseStuck =
        trackingState != Core::Ports::SLAM_TRACKING_OK && rawStep < 1.0e-5f;

    if (rawPoseStuck || rawStep > maxGuardStep) {
        Core::Ports::PoseEstimate guarded =
            PredictGuardedSmoothedPose(dt, maxGuardStep, maxSpeed);
        guarded.valid = true;
        pose = guarded;
        poseValid = true;
        m_smoothVelX *= POSE_STABILIZER_PREDICTED_VELOCITY_DECAY;
        m_smoothVelY *= POSE_STABILIZER_PREDICTED_VELOCITY_DECAY;
        m_smoothVelZ *= POSE_STABILIZER_PREDICTED_VELOCITY_DECAY;
    } else {
        UpdateGuardMeasuredVelocity(pose, dt, maxSpeed);
        poseValid = true;
    }

    if (poseValid && pose.valid) {
        StoreSmoothedPose(pose, timestampSec);
    }
}

void SlamEngineAdapter::StabilizeOutputPose(Core::Ports::PoseEstimate &pose,
                                            bool &poseValid,
                                            double timestampSec,
                                            int trackingState)
{
    pose.valid = poseValid && IsFinitePose(pose);
    if (pose.valid) {
        NormalizePoseQuaternion(pose);
    }

    const double dt = SmoothedPoseDeltaTime(timestampSec);

    if (!pose.valid) {
        PredictInvalidSmoothedPose(pose, poseValid, timestampSec);
        return;
    }

    const bool rawIdentity = IsIdentityPose(pose);
    if (!m_haveSmoothedOutputPose) {
        StoreSmoothedPose(pose, timestampSec);
        poseValid = true;
        return;
    }

    if (rawIdentity) {
        PredictIdentitySmoothedPose(pose, poseValid, timestampSec, dt);
        return;
    }

    if (UseAlphaBetaSmoother()) {
        ApplyAlphaBetaSmoother(pose, poseValid, timestampSec, dt);
        return;
    }
    ApplyGuardSmoother(pose, poseValid, timestampSec, dt, trackingState);
}

} // namespace SmartDrone::Adapters::Slam
