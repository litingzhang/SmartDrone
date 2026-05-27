#include "adapters/slam/openvins/openvins_runtime.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <sophus/se3.hpp>

#include "adapters/slam/openvins/openvins_runtime_driver.h"
#include "core/ports/slam_tracking_state.h"

namespace SmartDrone::Adapters::Slam {

namespace {

Core::Ports::TrackedVisualSummary EmptyTrackedVisualSummary()
{
    return {};
}

Sophus::SE3f IdentityPose()
{
    return Sophus::SE3f();
}

struct TrajectorySample {
    double timestampSec{0.0};
    Sophus::SE3f twc{};
};

void ConvertGrayImage(const cv::Mat &src, cv::Mat &dst)
{
    if (src.empty()) {
        dst.release();
        return;
    }
    if (src.type() == CV_8UC1) {
        dst = src;
        return;
    }
    cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
}

} // namespace

struct OpenVinsRuntime::Impl {
    std::unique_ptr<OpenVinsRuntimeDriver> driver;
    bool lastPoseLost{true};
    double lastTimestampSec{0.0};
    Sophus::SE3f lastTwc{};
    std::vector<TrajectorySample> trajectory;
};

OpenVinsRuntime::OpenVinsRuntime(std::string settingsPath)
    : m_impl(std::make_unique<Impl>())
{
    m_impl->driver = CreateOpenVinsRuntimeDriver(settingsPath);
}

OpenVinsRuntime::~OpenVinsRuntime() = default;

bool OpenVinsRuntime::Available() const
{
    return m_impl != nullptr && m_impl->driver != nullptr &&
           m_impl->driver->Available();
}

void OpenVinsRuntime::SetOperationMode(Core::Domain::SlamOperationMode mode)
{
    (void)mode;
}

void OpenVinsRuntime::RequestBackendStop()
{
}

bool OpenVinsRuntime::BackendStopped() const
{
    return true;
}

void OpenVinsRuntime::StepBackend()
{
}

void OpenVinsRuntime::Shutdown()
{
    if (m_impl == nullptr) {
        return;
    }
    if (m_impl->driver != nullptr) {
        m_impl->driver->Shutdown();
    }
    m_impl->lastPoseLost = true;
    m_impl->lastTimestampSec = 0.0;
    m_impl->lastTwc = IdentityPose();
    m_impl->trajectory.clear();
}

bool OpenVinsRuntime::ShutdownAndSaveTrajectoryEuRoC(const std::string &path)
{
    if (m_impl == nullptr || path.empty() || m_impl->trajectory.empty()) {
        return false;
    }

    std::ofstream stream(path);
    if (!stream.is_open()) {
        return false;
    }

    stream << std::fixed << std::setprecision(9);
    for (const TrajectorySample &sample : m_impl->trajectory) {
        const Eigen::Vector3f t = sample.twc.translation();
        const Eigen::Quaternionf q(sample.twc.so3().unit_quaternion());
        const int64_t timestampNs =
            static_cast<int64_t>(std::llround(sample.timestampSec * 1e9));
        stream << timestampNs << ' ' << t.x() << ' ' << t.y() << ' ' << t.z()
               << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' '
               << q.w() << '\n';
    }
    stream.flush();
    const bool ok = stream.good();
    if (ok) {
        Shutdown();
    }
    return ok;
}

Sophus::SE3f OpenVinsRuntime::TrackRaw(
    const Core::Ports::SlamTrackRequest &request)
{
    const bool validInput = request.input != nullptr &&
                            request.inputMode ==
                                Core::Ports::SlamInputMode::Stereo &&
                            request.useImu;
    if (!validInput || !Available()) {
        return IdentityPose();
    }

    const Sophus::SE3f tcw = m_impl->driver->TrackRaw(request);
    if (!m_impl->driver->Initialized()) {
        m_impl->lastPoseLost = true;
        return IdentityPose();
    }

    m_impl->lastTwc = tcw.inverse();
    m_impl->lastTimestampSec = request.input->frameTimeSec;
    m_impl->lastPoseLost = false;
    RecordTrajectorySample();
    return tcw;
}

void OpenVinsRuntime::RecordTrajectorySample()
{
    if (m_impl == nullptr) {
        return;
    }
    const TrajectorySample sample{m_impl->lastTimestampSec, m_impl->lastTwc};
    if (m_impl->trajectory.empty()) {
        m_impl->trajectory.push_back(sample);
        return;
    }
    TrajectorySample &last = m_impl->trajectory.back();
    if (sample.timestampSec > last.timestampSec) {
        m_impl->trajectory.push_back(sample);
        return;
    }
    if (sample.timestampSec == last.timestampSec) {
        last = sample;
    }
}

Sophus::SE3f OpenVinsRuntime::TrackPreparedStereoWithFeatures(
    const Core::Ports::PreparedStereoFeatureTrackRequest &request)
{
    Core::Ports::SlamTrackRequest rawRequest;
    rawRequest.input = request.input;
    rawRequest.inputMode = Core::Ports::SlamInputMode::Stereo;
    rawRequest.useImu = request.useImu;
    return TrackRaw(rawRequest);
}

bool OpenVinsRuntime::PrepareStereoImagesForTracking(
    const Core::Ports::StereoPreprocessRequest &request,
    Core::Ports::StereoPreprocessResult &result) const
{
    result = {};
    if (request.left == nullptr || request.right == nullptr) {
        return false;
    }
    ConvertGrayImage(*request.left, result.leftPrepared);
    ConvertGrayImage(*request.right, result.rightPrepared);
    return !result.leftPrepared.empty() && !result.rightPrepared.empty();
}

int OpenVinsRuntime::TrackingState() const
{
    if (!Available()) {
        return Core::Ports::SLAM_TRACKING_NO_IMAGES_YET;
    }
    if (!HasTrackingInitialized()) {
        return Core::Ports::SLAM_TRACKING_NOT_INITIALIZED;
    }
    return m_impl->lastPoseLost ? Core::Ports::SLAM_TRACKING_RECENTLY_LOST
                                : Core::Ports::SLAM_TRACKING_OK;
}

int OpenVinsRuntime::TrackedMapPointCount() const
{
    return 0;
}

bool OpenVinsRuntime::IsTrackingInitializing() const
{
    const int state = TrackingState();
    return state == Core::Ports::SLAM_TRACKING_NO_IMAGES_YET ||
           state == Core::Ports::SLAM_TRACKING_NOT_INITIALIZED;
}

bool OpenVinsRuntime::IsTrackingRecovering() const
{
    const int state = TrackingState();
    return state == Core::Ports::SLAM_TRACKING_RECENTLY_LOST ||
           state == Core::Ports::SLAM_TRACKING_LOST;
}

bool OpenVinsRuntime::HasTrackingInitialized() const
{
    return m_impl != nullptr && m_impl->driver != nullptr &&
           m_impl->driver->Initialized();
}

const Core::Ports::IVisualDescriptorProvider *
OpenVinsRuntime::LeftDescriptorProvider()
{
    return nullptr;
}

const Core::Ports::IVisualDescriptorProvider *
OpenVinsRuntime::RightDescriptorProvider()
{
    return nullptr;
}

bool OpenVinsRuntime::GetLatestFrameTrajectoryPoseEuRoC(
    Sophus::SE3f &twc, double *timestamp, bool *lost) const
{
    if (!HasTrackingInitialized() || m_impl->lastPoseLost) {
        return false;
    }
    twc = m_impl->lastTwc;
    if (timestamp != nullptr) {
        *timestamp = m_impl->lastTimestampSec;
    }
    if (lost != nullptr) {
        *lost = false;
    }
    return true;
}

Core::Ports::SlamMapSummary OpenVinsRuntime::GetMapSummary() const
{
    return {};
}

Core::Ports::SlamBackendStats OpenVinsRuntime::GetBackendStats() const
{
    Core::Ports::SlamBackendStats stats{};
    stats.frame.trackingState = TrackingState();
    return stats;
}

Core::Ports::TrackedVisualSummary OpenVinsRuntime::GetTrackedVisualSummary() const
{
    return EmptyTrackedVisualSummary();
}

Core::Ports::TrackedFeatureSnapshot OpenVinsRuntime::ExtractTrackedFeatures(
    int leftImageWidth, int leftImageHeight, int rightImageWidth,
    int rightImageHeight)
{
    (void)leftImageWidth;
    (void)leftImageHeight;
    (void)rightImageWidth;
    (void)rightImageHeight;
    return {};
}

Core::Ports::TrackedPointCloudSnapshot OpenVinsRuntime::ExtractTrackedPointCloud(
    size_t maxPointCloudPoints)
{
    (void)maxPointCloudPoints;
    return {};
}

Core::Ports::TrackedVisualData OpenVinsRuntime::ExtractTrackedVisualData(
    const Core::Ports::VisualMapSnapshotRequest &request)
{
    (void)request;
    return {};
}

Core::Ports::VisualMapSnapshot OpenVinsRuntime::ExtractVisualMapSnapshot(
    const Core::Ports::VisualMapSnapshotRequest &request)
{
    (void)request;
    return {};
}

void OpenVinsRuntime::LogStereoFeatureDiagnostics(
    uint64_t frameId,
    const Core::Ports::StereoFeatureObservationPacket &observations) const
{
    (void)frameId;
    (void)observations;
}

bool OpenVinsRuntime::Optimize(
    const Core::Ports::SlamBackendOptimizationRequest &request,
    Core::Ports::SlamBackendOptimizationResult &result)
{
    (void)request;
    result = {};
    return false;
}

bool OpenVinsRuntime::ApplyMappingOperation(
    const Core::Ports::SlamBackendMappingRequest &request,
    Core::Ports::SlamBackendMappingResult &result)
{
    (void)request;
    result = {};
    return false;
}

bool OpenVinsRuntime::ApplyLoopClosureOperation(
    const Core::Ports::SlamBackendLoopClosureRequest &request,
    Core::Ports::SlamBackendLoopClosureResult &result)
{
    (void)request;
    result = {};
    return false;
}

} // namespace SmartDrone::Adapters::Slam
