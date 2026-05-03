#include "adapters/slam/orbslam3_engine.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <utility>

#include <sophus/se3.hpp>

#include "ImuTypes.h"
#include "TrackedVisualData.h"
#include "adapters/slam/orbslam3_mode_strategy.h"
#include "adapters/slam/orbslam3_mode_common.h"

namespace smartdrone::adapters::slam {

OrbSlam3Engine::OrbSlam3Engine(std::unique_ptr<ORB_SLAM3::System> system, OrbInputMode inputMode, bool useImu,
                               std::string settingsPath)
    : m_system(std::move(system)), m_modeState(std::make_unique<SlamModeSharedState>()), m_inputMode(inputMode),
      m_useImu(useImu), m_modeStrategy(CreateSlamModeStrategy(FeatureFrontend::Orb)),
      m_settingsPath(std::move(settingsPath))
{
    m_modeState->LoadStereoCalibration(m_settingsPath);
}

OrbSlam3Engine::~OrbSlam3Engine() = default;

bool OrbSlam3Engine::Start()
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

void OrbSlam3Engine::SetOperationMode(core::domain::SlamOperationMode mode)
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

void OrbSlam3Engine::SetFeatureFrontend(FeatureFrontend frontend)
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

void OrbSlam3Engine::Stop()
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

bool OrbSlam3Engine::ShutdownAndSaveOrbTrajectoryEuRoC(const std::string &path)
{
    if (!m_system || path.empty()) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    m_system->SaveTrajectoryEuRoC(path);
    return true;
}

void OrbSlam3Engine::StabilizeOutputPose(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
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
