#include "adapters/slam/openvins/openvins_slam_engine.h"

#include <iostream>
#include <utility>

#include "adapters/slam/engine/slam_output_utils.h"
#include "adapters/slam/engine/slam_pose_utils.h"
#include "core/ports/slam_tracking_state.h"

namespace SmartDrone::Adapters::Slam {

OpenVinsSlamEngine::OpenVinsSlamEngine(std::string settingsPath)
    : m_runtime(std::move(settingsPath))
{
}

OpenVinsSlamEngine::~OpenVinsSlamEngine() = default;

bool OpenVinsSlamEngine::Start()
{
    m_lastPose = {};
    m_haveLastPose = false;
    return m_runtime.Available();
}

void OpenVinsSlamEngine::Stop()
{
    m_lastPose = {};
    m_haveLastPose = false;
    m_runtime.Shutdown();
}

Core::Ports::SlamOutput OpenVinsSlamEngine::Process(
    const Core::Ports::SlamInputBatch &input, bool extractFeatures,
    bool extractPointCloud)
{
    (void)extractFeatures;
    (void)extractPointCloud;

    Core::Ports::SlamOutput out = MakeOkSlamOutput(input);
    Core::Ports::SlamTrackRequest request;
    request.input = &input;
    request.inputMode = Core::Ports::SlamInputMode::Stereo;
    request.useImu = true;
    const Sophus::SE3f tcw = m_runtime.TrackRaw(request);
    out.trackingState = m_runtime.TrackingState();
    if (!m_runtime.HasTrackingInitialized()) {
        return BuildBootstrapOutput(input);
    }
    if (!TrackingStateCanPublishPose(out.trackingState)) {
        if (FillContinuityPose(input, out)) {
            return out;
        }
        MarkSlamOutputPoseLost(out, out.trackingState);
        return out;
    }
    out.pose = PoseFromTwc(tcw.inverse());
    out.poseValid = out.pose.valid;
    if (out.poseValid) {
        m_lastPose = out.pose;
        m_haveLastPose = true;
    }
    return out;
}

Core::Ports::SlamOutput OpenVinsSlamEngine::BuildBootstrapOutput(
    const Core::Ports::SlamInputBatch &input) const
{
    Core::Ports::SlamOutput out{};
    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    out.trackingState = Core::Ports::SLAM_TRACKING_NOT_INITIALIZED;
    static int bootstrapLogCount = 0;
    if (bootstrapLogCount < 5) {
        ++bootstrapLogCount;
        std::cerr << "[openvins_engine] bootstrap frame=" << input.frameId
                  << " tracking=" << out.trackingState
                  << " poseValid=" << out.poseValid
                  << " pose.valid=" << out.pose.valid << "\n";
    }
    return out;
}

bool OpenVinsSlamEngine::FillContinuityPose(
    const Core::Ports::SlamInputBatch &input,
    Core::Ports::SlamOutput &out)
{
    if (!m_haveLastPose) {
        return false;
    }
    out.pose = m_lastPose;
    out.pose.valid = true;
    out.poseValid = true;
    out.trackingState = Core::Ports::SLAM_TRACKING_RECENTLY_LOST;
    static int continuityLogCount = 0;
    if (continuityLogCount < 5) {
        ++continuityLogCount;
        std::cerr << "[openvins_engine] continuity frame=" << input.frameId
                  << " tracking=" << out.trackingState
                  << " poseValid=" << out.poseValid << "\n";
    }
    return true;
}

void OpenVinsSlamEngine::SetOperationMode(Core::Domain::SlamOperationMode mode)
{
    m_runtime.SetOperationMode(mode);
}

void OpenVinsSlamEngine::SetFeatureFrontend(FeatureFrontend frontend)
{
    (void)frontend;
}

void OpenVinsSlamEngine::SetVisualFeatureFrontend(
    Core::Ports::IVisualFeatureFrontend *frontend)
{
    (void)frontend;
}

void OpenVinsSlamEngine::SetVisualFeatureInputSizeLimit(int maxWidth,
                                                        int maxHeight)
{
    (void)maxWidth;
    (void)maxHeight;
}

void OpenVinsSlamEngine::SetStereoVoLoopClosure(bool enabled, float scale,
                                                float relaxation)
{
    (void)enabled;
    (void)scale;
    (void)relaxation;
}

void OpenVinsSlamEngine::SetStereoVoPerFrameAcceleration(
    std::string acceleration)
{
    (void)acceleration;
}

bool OpenVinsSlamEngine::ShutdownAndSaveTrajectoryEuRoC(
    const std::string &path)
{
    return m_runtime.ShutdownAndSaveTrajectoryEuRoC(path);
}

void OpenVinsSlamEngine::RequestBackendStop()
{
    m_runtime.RequestBackendStop();
}

bool OpenVinsSlamEngine::BackendStopped() const
{
    return m_runtime.BackendStopped();
}

void OpenVinsSlamEngine::StepBackend()
{
    m_runtime.StepBackend();
}

} // namespace SmartDrone::Adapters::Slam
