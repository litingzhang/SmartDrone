#include "adapters/slam/slam_engine_adapter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <utility>

#include <sophus/se3.hpp>

#include "ImuTypes.h"
#include "TrackedVisualData.h"
#include "adapters/slam/slam_mode_strategy.h"
#include "adapters/slam/slam_mode_common.h"

namespace smartdrone::adapters::slam {

SlamEngineAdapter::SlamEngineAdapter(std::unique_ptr<ORB_SLAM3::System> system, SlamInputMode inputMode, bool useImu,
                               std::string settingsPath)
    : m_system(std::move(system)), m_modeState(std::make_unique<SlamModeSharedState>()), m_inputMode(inputMode),
      m_useImu(useImu), m_modeStrategy(CreateSlamModeStrategy(FeatureFrontend::Orb)),
      m_settingsPath(std::move(settingsPath))
{
    m_modeState->LoadStereoCalibration(m_settingsPath);
}

SlamEngineAdapter::~SlamEngineAdapter() = default;

bool SlamEngineAdapter::Start()
{
    m_lastStablePose = core::ports::PoseEstimate{};
    m_haveLastStablePose = false;
    m_lastStableTimestampSec = 0.0;
    m_modeState->ResetTrackingState();
    m_stableVelX = 0.0f;
    m_stableVelY = 0.0f;
    m_stableVelZ = 0.0f;
    return static_cast<bool>(m_system);
}

void SlamEngineAdapter::SetOperationMode(core::domain::SlamOperationMode mode)
{
    if (!m_system || m_operationMode == mode) {
        return;
    }

    const bool localizationOnly = mode == core::domain::SlamOperationMode::Localization ||
                                  mode == core::domain::SlamOperationMode::Relocalization ||
                                  mode == core::domain::SlamOperationMode::TrackingOnly;

    if (localizationOnly) {
        m_system->ActivateLocalizationMode();
    } else {
        m_system->DeactivateLocalizationMode();
    }
    m_operationMode = mode;
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

void SlamEngineAdapter::Stop()
{
    m_lastStablePose = core::ports::PoseEstimate{};
    m_haveLastStablePose = false;
    m_lastStableTimestampSec = 0.0;
    m_modeState->ResetTrackingState();
    m_stableVelX = 0.0f;
    m_stableVelY = 0.0f;
    m_stableVelZ = 0.0f;
    if (m_system) {
        m_system->Shutdown();
    }
}

bool SlamEngineAdapter::ShutdownAndSaveTrajectoryEuRoC(const std::string &path)
{
    if (!m_system || path.empty()) {
        return false;
    }
    m_system->Shutdown();
    m_system->SaveTrajectoryEuRoC(path);
    return true;
}

void SlamEngineAdapter::MaintainRealtimePoseContinuity(core::ports::PoseEstimate &pose, bool &poseValid,
                                                    double timestampSec, int trackingState)
{
    pose.valid = poseValid && IsFinitePose(pose);
    if (pose.valid) {
        NormalizePoseQuaternion(pose);
    }

    double dt = kPoseStabilizerDefaultDtSec;
    if (m_haveLastStablePose && timestampSec > m_lastStableTimestampSec) {
        dt = std::clamp(timestampSec - m_lastStableTimestampSec, kPoseStabilizerMinDtSec, kPoseStabilizerMaxDtSec);
    }

    if (pose.valid && trackingState == ORB_SLAM3::Tracking::OK) {
        if (m_haveLastStablePose) {
            float measuredVelX = (pose.x - m_lastStablePose.x) / static_cast<float>(dt);
            float measuredVelY = (pose.y - m_lastStablePose.y) / static_cast<float>(dt);
            float measuredVelZ = (pose.z - m_lastStablePose.z) / static_cast<float>(dt);
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

    const bool rawIdentity = pose.x == 0.0f && pose.y == 0.0f && pose.z == 0.0f && pose.qw == 1.0f &&
                             pose.qx == 0.0f && pose.qy == 0.0f && pose.qz == 0.0f;
    const bool bootstrapFrame =
        !m_haveLastStablePose && trackingState == ORB_SLAM3::Tracking::NOT_INITIALIZED &&
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
        !pose.valid || trackingState == ORB_SLAM3::Tracking::RECENTLY_LOST || trackingState == ORB_SLAM3::Tracking::OK_KLT;
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

void SlamEngineAdapter::GateRealtimePoseQuality(core::ports::SlamOutput &out, double timestampSec)
{
    const bool canMeasurePoseStep = out.poseValid && out.pose.valid && IsFinitePose(out.pose) && m_haveLastStablePose;
    if (canMeasurePoseStep) {
        out.rawPoseStepMeters = PoseTranslationDistance(out.pose, m_lastStablePose);
        out.gatedPoseStepMeters = out.rawPoseStepMeters;
    }
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_REALTIME_QUALITY_GATE", false)) {
        return;
    }
    if (!canMeasurePoseStep) {
        return;
    }
    if (!out.usedSuperPointFrontend || !out.externalStereoInjected || out.externalStereoStabilizing ||
        out.trackingState != ORB_SLAM3::Tracking::OK) {
        return;
    }

    const int minInliers = EnvIntValueClamped("SMART_DRONE_SP_LG_REALTIME_GATE_MIN_INLIERS", 120, 1, 100000);
    const int minTracked = EnvIntValueClamped("SMART_DRONE_SP_LG_REALTIME_GATE_MIN_TRACKED_MAP", 140, 1, 100000);
    const float maxStep =
        EnvFloatValueClamped("SMART_DRONE_SP_LG_REALTIME_GATE_MAX_STEP_M", kPoseStabilizerMaxStepMeters,
                             0.005f, 0.25f);
    const bool weakTracking = out.matchesInliers < minInliers || static_cast<int>(out.trackedMapPointCount) < minTracked;
    const float rawStep = out.rawPoseStepMeters;
    if (!weakTracking) {
        return;
    }

    core::ports::PoseEstimate gated = out.pose;
    const std::string gateMode = EnvStringValue("SMART_DRONE_SP_LG_REALTIME_GATE_MODE", "step");
    if (gateMode == "innovation" || gateMode == "predict" || gateMode == "prediction") {
        double dt = kPoseStabilizerDefaultDtSec;
        if (timestampSec > m_lastStableTimestampSec) {
            dt = std::clamp(timestampSec - m_lastStableTimestampSec, kPoseStabilizerMinDtSec, kPoseStabilizerMaxDtSec);
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
            EnvFloatValueClamped("SMART_DRONE_SP_LG_REALTIME_GATE_MAX_INNOVATION_M", maxStep, 0.005f, 0.50f);
        const float innovation = PoseTranslationDistance(out.pose, predicted);
        if (innovation <= maxInnovation) {
            return;
        }

        const float scale = maxInnovation / std::max(innovation, 1.0e-6f);
        gated.x = predicted.x + (out.pose.x - predicted.x) * scale;
        gated.y = predicted.y + (out.pose.y - predicted.y) * scale;
        gated.z = predicted.z + (out.pose.z - predicted.z) * scale;
        if (EnvFlagEnabled("SMART_DRONE_SP_LG_REALTIME_GATE_LIMIT_ROTATION", false)) {
            LimitPoseRotationStep(predicted, gated);
        }
        gated.valid = true;

        out.pose = gated;
        out.poseValid = true;
        out.realtimePoseQualityGate = true;
        out.gatedPoseStepMeters = PoseTranslationDistance(out.pose, m_lastStablePose);
        return;
    }

    if (rawStep <= maxStep) {
        return;
    }

    const float scale = maxStep / std::max(rawStep, 1.0e-6f);
    gated.x = m_lastStablePose.x + (out.pose.x - m_lastStablePose.x) * scale;
    gated.y = m_lastStablePose.y + (out.pose.y - m_lastStablePose.y) * scale;
    gated.z = m_lastStablePose.z + (out.pose.z - m_lastStablePose.z) * scale;
    LimitPoseRotationStep(m_lastStablePose, gated);
    gated.valid = true;

    out.pose = gated;
    out.poseValid = true;
    out.realtimePoseQualityGate = true;
    out.gatedPoseStepMeters = PoseTranslationDistance(out.pose, m_lastStablePose);
}

void SlamEngineAdapter::StabilizeOutputPose(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
                                         int trackingState)
{
    pose.valid = poseValid && IsFinitePose(pose);
    if (pose.valid) {
        NormalizePoseQuaternion(pose);
    }

    double dt = kPoseStabilizerDefaultDtSec;
    if (m_haveLastStablePose && timestampSec > m_lastStableTimestampSec) {
        dt = std::clamp(timestampSec - m_lastStableTimestampSec, kPoseStabilizerMinDtSec, kPoseStabilizerMaxDtSec);
    }

    core::ports::PoseEstimate predicted = m_lastStablePose;
    if (m_haveLastStablePose) {
        ClampVelocityVector(m_stableVelX, m_stableVelY, m_stableVelZ);
        predicted.x += m_stableVelX * static_cast<float>(dt);
        predicted.y += m_stableVelY * static_cast<float>(dt);
        predicted.z += m_stableVelZ * static_cast<float>(dt);
        predicted.valid = true;
    }

    const bool rawIdentity = pose.valid && IsIdentityPose(pose);
    bool usePrediction = !pose.valid || rawIdentity;
    if (!usePrediction && m_haveLastStablePose) {
        const float rawStep = PoseTranslationDistance(pose, m_lastStablePose);
        const float maxStep =
            std::min(kPoseStabilizerMaxSpeedMps * static_cast<float>(dt), kPoseStabilizerMaxStepMeters);
        const bool rawPoseStuck = trackingState != ORB_SLAM3::Tracking::OK && rawStep < 1.0e-5f;
        usePrediction = rawPoseStuck || rawStep > maxStep;
        if (usePrediction && rawStep > maxStep && rawStep > 1.0e-6f) {
            const float scale = maxStep / rawStep;
            predicted.x = m_lastStablePose.x + (pose.x - m_lastStablePose.x) * scale;
            predicted.y = m_lastStablePose.y + (pose.y - m_lastStablePose.y) * scale;
            predicted.z = m_lastStablePose.z + (pose.z - m_lastStablePose.z) * scale;
            predicted.qw = m_lastStablePose.qw;
            predicted.qx = m_lastStablePose.qx;
            predicted.qy = m_lastStablePose.qy;
            predicted.qz = m_lastStablePose.qz;
            predicted.valid = true;
        }
    }

    if (usePrediction && m_haveLastStablePose) {
        pose = predicted;
        poseValid = true;
        m_stableVelX *= kPoseStabilizerPredictedVelocityDecay;
        m_stableVelY *= kPoseStabilizerPredictedVelocityDecay;
        m_stableVelZ *= kPoseStabilizerPredictedVelocityDecay;
    } else if (pose.valid) {
        if (m_haveLastStablePose) {
            float measuredVelX = (pose.x - m_lastStablePose.x) / static_cast<float>(dt);
            float measuredVelY = (pose.y - m_lastStablePose.y) / static_cast<float>(dt);
            float measuredVelZ = (pose.z - m_lastStablePose.z) / static_cast<float>(dt);
            ClampVelocityVector(measuredVelX, measuredVelY, measuredVelZ);
            LimitPoseRotationStep(m_lastStablePose, pose);
            m_stableVelX = (1.0f - kPoseStabilizerVelocityAlpha) * m_stableVelX +
                           kPoseStabilizerVelocityAlpha * measuredVelX;
            m_stableVelY = (1.0f - kPoseStabilizerVelocityAlpha) * m_stableVelY +
                           kPoseStabilizerVelocityAlpha * measuredVelY;
            m_stableVelZ = (1.0f - kPoseStabilizerVelocityAlpha) * m_stableVelZ +
                           kPoseStabilizerVelocityAlpha * measuredVelZ;
        }
        poseValid = true;
    } else {
        poseValid = false;
    }

    if (poseValid && pose.valid) {
        m_lastStablePose = pose;
        m_haveLastStablePose = true;
        m_lastStableTimestampSec = timestampSec;
    }
}

} // namespace smartdrone::adapters::slam
