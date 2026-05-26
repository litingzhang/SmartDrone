#include "adapters/slam/openvins/openvins_runtime.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <sophus/se3.hpp>

#include "core/ports/slam_tracking_state.h"

#if defined(SMART_DRONE_HAS_OPENVINS)
#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/sensor_data.h"
#endif

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

} // namespace

struct OpenVinsRuntime::Impl {
    struct TrajectorySample {
        double timestampSec{0.0};
        Sophus::SE3f twc{};
    };

#if defined(SMART_DRONE_HAS_OPENVINS)
    std::unique_ptr<ov_msckf::VioManager> manager;
#endif
    bool available{false};
    bool initialized{false};
    bool lastPoseLost{true};
    double lastTimestampSec{0.0};
    Sophus::SE3f lastTwc{};
    std::vector<TrajectorySample> trajectory;
};

OpenVinsRuntime::OpenVinsRuntime(std::string settingsPath)
    : m_impl(std::make_unique<Impl>())
{
#if defined(SMART_DRONE_HAS_OPENVINS)
    if (!settingsPath.empty()) {
        auto parser = std::make_shared<ov_core::YamlParser>(settingsPath, false);
        if (parser != nullptr && parser->successful()) {
            ov_msckf::VioManagerOptions options;
            options.print_and_load(parser);
#if defined(SMART_DRONE_OPENVINS_DISABLE_CERES)
            options.init_options.init_dyn_use = false;
#endif
            m_impl->manager = std::make_unique<ov_msckf::VioManager>(options);
            m_impl->available = m_impl->manager != nullptr;
        }
    }
#else
    (void)settingsPath;
#endif
}

OpenVinsRuntime::~OpenVinsRuntime() = default;

bool OpenVinsRuntime::Available() const
{
    return m_impl != nullptr && m_impl->available;
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
#if defined(SMART_DRONE_HAS_OPENVINS)
    m_impl->manager.reset();
#endif
    m_impl->available = false;
    m_impl->initialized = false;
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
    for (const Impl::TrajectorySample &sample : m_impl->trajectory) {
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
    if (request.input == nullptr || request.inputMode != Core::Ports::SlamInputMode::Stereo ||
        !request.useImu || !Available()) {
        return IdentityPose();
    }
#if defined(SMART_DRONE_HAS_OPENVINS)
    ov_core::CameraData cameraData;
    cameraData.timestamp = request.input->frameTimeSec;
    cameraData.sensor_ids = {0, 1};
    cameraData.images = {request.input->stereo.left.gray, request.input->stereo.right.gray};
    cameraData.masks = {
        cv::Mat::zeros(request.input->stereo.left.gray.rows,
                       request.input->stereo.left.gray.cols, CV_8UC1),
        cv::Mat::zeros(request.input->stereo.right.gray.rows,
                       request.input->stereo.right.gray.cols, CV_8UC1)};
    if (cameraData.images[0].empty() || cameraData.images[1].empty()) {
        std::cerr << "[openvins] empty stereo image left="
                  << cameraData.images[0].cols << "x" << cameraData.images[0].rows
                  << " right=" << cameraData.images[1].cols << "x" << cameraData.images[1].rows
                  << " frame_id=" << request.input->frameId << "\n";
    }

    for (const Core::Ports::ImuReading &reading : request.input->imu) {
        ov_core::ImuData imuData;
        imuData.timestamp = static_cast<double>(reading.timestampNs) * 1e-9;
        imuData.wm << static_cast<double>(reading.gx), static_cast<double>(reading.gy),
            static_cast<double>(reading.gz);
        imuData.am << static_cast<double>(reading.ax), static_cast<double>(reading.ay),
            static_cast<double>(reading.az);
        m_impl->manager->feed_measurement_imu(imuData);
    }

    m_impl->manager->feed_measurement_camera(cameraData);
    m_impl->initialized = m_impl->manager->initialized();
    if (!m_impl->initialized) {
        m_impl->lastPoseLost = true;
        return IdentityPose();
    }

    const std::shared_ptr<ov_msckf::State> state = m_impl->manager->get_state();
    if (state == nullptr || state->_imu == nullptr) {
        m_impl->lastPoseLost = true;
        return IdentityPose();
    }

    const Eigen::Vector4d qGtoI = state->_imu->quat();
    const Eigen::Vector3d pIinG = state->_imu->pos();
    Eigen::Quaterniond qGtoIQuat(qGtoI(3), qGtoI(0), qGtoI(1), qGtoI(2));
    qGtoIQuat.normalize();
    const Eigen::Quaternionf qTwc = qGtoIQuat.conjugate().cast<float>();
    const Eigen::Vector3f tTwc = pIinG.cast<float>();
    m_impl->lastTwc = Sophus::SE3f(Sophus::SO3f(qTwc), tTwc);
    m_impl->lastTimestampSec = request.input->frameTimeSec;
    m_impl->lastPoseLost = false;
    if (!m_impl->trajectory.empty() &&
        m_impl->lastTimestampSec <= m_impl->trajectory.back().timestampSec) {
        if (m_impl->lastTimestampSec == m_impl->trajectory.back().timestampSec) {
            m_impl->trajectory.back() = {m_impl->lastTimestampSec,
                                         m_impl->lastTwc};
        }
    } else {
        m_impl->trajectory.push_back({m_impl->lastTimestampSec, m_impl->lastTwc});
    }
    return m_impl->lastTwc.inverse();
#else
    return IdentityPose();
#endif
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
    const auto convertGray = [](const cv::Mat &src, cv::Mat &dst) {
        if (src.empty()) {
            dst.release();
            return;
        }
        if (src.type() == CV_8UC1) {
            dst = src;
            return;
        }
        cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
    };
    convertGray(*request.left, result.leftPrepared);
    convertGray(*request.right, result.rightPrepared);
    return !result.leftPrepared.empty() && !result.rightPrepared.empty();
}

int OpenVinsRuntime::TrackingState() const
{
    if (!Available()) {
        return Core::Ports::SLAM_TRACKING_NO_IMAGES_YET;
    }
    if (!m_impl->initialized) {
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
    return m_impl != nullptr && m_impl->initialized;
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
    uint64_t frameId, const Core::Ports::StereoFeatureObservationPacket &observations) const
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
