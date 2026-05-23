#include "adapters/slam/orb/orb_slam3_runtime.h"

#include <memory>
#include <utility>
#include <vector>

#include "System.h"
#include "adapters/slam/orb/orb_visual_components.h"
#include "core/ports/slam_tracking_state.h"

namespace SmartDrone::Adapters::Slam {

namespace {

ORB_SLAM3::System::eSensor ResolveOrbSensor(SensorMode sensorMode)
{
    switch (sensorMode) {
    case SensorMode::MonoImu:
        return ORB_SLAM3::System::IMU_MONOCULAR;
    case SensorMode::Mono:
        return ORB_SLAM3::System::MONOCULAR;
    case SensorMode::StereoImu:
        return ORB_SLAM3::System::IMU_STEREO;
    case SensorMode::Stereo:
    default:
        return ORB_SLAM3::System::STEREO;
    }
}

std::vector<ORB_SLAM3::IMU::Point>
ToOrbImuPoints(const std::vector<Core::Ports::ImuReading> &readings)
{
    std::vector<ORB_SLAM3::IMU::Point> out;
    out.reserve(readings.size());
    for (const Core::Ports::ImuReading &reading : readings) {
        out.emplace_back(cv::Point3f(reading.ax, reading.ay, reading.az),
                         cv::Point3f(reading.gx, reading.gy, reading.gz),
                         static_cast<double>(reading.timestampNs) * 1e-9);
    }
    return out;
}

} // namespace

struct OrbSlam3Runtime::Impl {
    std::unique_ptr<ORB_SLAM3::System> system;
};

OrbSlam3Runtime::OrbSlam3Runtime(std::string vocabularyPath,
                                 std::string settingsPath,
                                 SensorMode sensorMode, bool useViewer)
    : m_impl(std::make_unique<Impl>())
{
    m_impl->system = std::make_unique<ORB_SLAM3::System>(
        std::move(vocabularyPath), std::move(settingsPath),
        ResolveOrbSensor(sensorMode), useViewer);
}

OrbSlam3Runtime::~OrbSlam3Runtime() = default;

bool OrbSlam3Runtime::Available() const
{
    return m_impl && m_impl->system;
}

void OrbSlam3Runtime::StepBackend()
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system != nullptr) {
        system->StepBackend();
    }
}

void OrbSlam3Runtime::SetOperationMode(Core::Domain::SlamOperationMode mode)
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr || m_operationMode == mode) {
        return;
    }

    const bool localizationOnly =
        mode == Core::Domain::SlamOperationMode::Localization ||
        mode == Core::Domain::SlamOperationMode::Relocalization ||
        mode == Core::Domain::SlamOperationMode::TrackingOnly;
    if (localizationOnly) {
        system->ActivateLocalizationMode();
    } else {
        system->DeactivateLocalizationMode();
    }
    m_operationMode = mode;
}

void OrbSlam3Runtime::Shutdown()
{
    if (m_impl && m_impl->system) {
        m_impl->system->Shutdown();
    }
}

bool OrbSlam3Runtime::ShutdownAndSaveTrajectoryEuRoC(const std::string &path)
{
    if (!m_impl || !m_impl->system || path.empty()) {
        return false;
    }
    m_impl->system->Shutdown();
    m_impl->system->SaveTrajectoryEuRoC(path);
    return true;
}

Sophus::SE3f OrbSlam3Runtime::TrackRaw(const Core::Ports::SlamInputBatch &input,
                                       Core::Ports::SlamInputMode inputMode,
                                       bool useImu)
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr) {
        return Sophus::SE3f();
    }

    const bool monoMode = inputMode != Core::Ports::SlamInputMode::Stereo;
    const cv::Mat &monoImage =
        (inputMode == Core::Ports::SlamInputMode::MonoRight)
            ? input.stereo.right.gray
            : input.stereo.left.gray;

    if (useImu) {
        const std::vector<ORB_SLAM3::IMU::Point> orbImu = ToOrbImuPoints(input.imu);
        if (monoMode) {
            return system->TrackMonocular(monoImage, input.frameTimeSec, orbImu);
        }
        return system->TrackStereo(input.stereo.left.gray, input.stereo.right.gray,
                                   input.frameTimeSec, orbImu);
    }

    if (monoMode) {
        return system->TrackMonocular(monoImage, input.frameTimeSec);
    }
    return system->TrackStereo(input.stereo.left.gray, input.stereo.right.gray,
                               input.frameTimeSec);
}

Sophus::SE3f
OrbSlam3Runtime::TrackRaw(const Core::Ports::SlamTrackRequest &request)
{
    if (request.input == nullptr) {
        return Sophus::SE3f();
    }
    return TrackRaw(*request.input, request.inputMode, request.useImu);
}

Sophus::SE3f OrbSlam3Runtime::TrackPreparedStereoWithFeatures(
    const Core::Ports::PreparedStereoFeatureTrackRequest &request)
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr || request.input == nullptr ||
        request.leftPrepared == nullptr || request.rightPrepared == nullptr ||
        request.observations == nullptr) {
        return Sophus::SE3f();
    }
    if (request.useImu) {
        const std::vector<ORB_SLAM3::IMU::Point> orbImu =
            ToOrbImuPoints(request.input->imu);
        return system->TrackStereoPreparedWithFeatures(
            *request.leftPrepared, *request.rightPrepared, *request.observations,
            request.input->frameTimeSec, orbImu);
    }
    return system->TrackStereoPreparedWithFeatures(
        *request.leftPrepared, *request.rightPrepared, *request.observations,
        request.input->frameTimeSec);
}

bool OrbSlam3Runtime::PrepareStereoImagesForTracking(
    const cv::Mat &left, const cv::Mat &right, cv::Mat &leftPrepared,
    cv::Mat &rightPrepared) const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    return system != nullptr && system->PrepareStereoImagesForTracking(
                                    left, right, leftPrepared, rightPrepared);
}

bool OrbSlam3Runtime::PrepareStereoImagesForTracking(
    const Core::Ports::StereoPreprocessRequest &request,
    Core::Ports::StereoPreprocessResult &result) const
{
    result = {};
    if (request.left == nullptr || request.right == nullptr) {
        return false;
    }
    return PrepareStereoImagesForTracking(*request.left, *request.right,
                                          result.leftPrepared,
                                          result.rightPrepared);
}

int OrbSlam3Runtime::TrackingState() const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    return system != nullptr ? system->GetTrackingState()
                             : Core::Ports::kSlamTrackingNoImagesYet;
}

int OrbSlam3Runtime::TrackedMapPointCount() const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    return system != nullptr ? static_cast<int>(system->GetTrackedMapPointCount())
                             : 0;
}

bool OrbSlam3Runtime::IsTrackingInitializing() const
{
    const int state = TrackingState();
    return state == Core::Ports::kSlamTrackingNoImagesYet ||
           state == Core::Ports::kSlamTrackingNotInitialized;
}

bool OrbSlam3Runtime::IsTrackingRecovering() const
{
    const int state = TrackingState();
    return state == Core::Ports::kSlamTrackingRecentlyLost ||
           state == Core::Ports::kSlamTrackingLost;
}

bool OrbSlam3Runtime::HasTrackingInitialized() const
{
    return !IsTrackingInitializing();
}

const Core::Ports::IVisualDescriptorProvider *
OrbSlam3Runtime::LeftDescriptorProvider()
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    auto *extractor = system != nullptr ? system->GetLeftORBExtractor() : nullptr;
    if (extractor == nullptr) {
        m_leftDescriptorProvider.reset();
        return nullptr;
    }
    m_leftDescriptorProvider = CreateOrbDescriptorProvider(extractor);
    return m_leftDescriptorProvider.get();
}

const Core::Ports::IVisualDescriptorProvider *
OrbSlam3Runtime::RightDescriptorProvider()
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    auto *extractor =
        system != nullptr ? system->GetRightORBExtractor() : nullptr;
    if (extractor == nullptr) {
        m_rightDescriptorProvider.reset();
        return nullptr;
    }
    m_rightDescriptorProvider = CreateOrbDescriptorProvider(extractor);
    return m_rightDescriptorProvider.get();
}

bool OrbSlam3Runtime::GetLatestFrameTrajectoryPoseEuRoC(Sophus::SE3f &twc,
                                                        double *timestamp,
                                                        bool *lost) const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    return system != nullptr &&
           system->GetLatestFrameTrajectoryPoseEuRoC(twc, timestamp, lost);
}

Core::Ports::SlamMapSummary OrbSlam3Runtime::GetMapSummary() const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr) {
        return {};
    }
    return system->GetMapSummary();
}

Core::Ports::SlamBackendStats OrbSlam3Runtime::GetBackendStats() const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr) {
        return {};
    }
    return system->GetBackendStats();
}

Core::Ports::TrackedVisualSummary
OrbSlam3Runtime::GetTrackedVisualSummary() const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr) {
        return {};
    }
    return system->GetTrackedVisualSummary();
}

Core::Ports::TrackedFeatureSnapshot
OrbSlam3Runtime::ExtractTrackedFeatures(int leftImageWidth, int leftImageHeight,
                                        int rightImageWidth,
                                        int rightImageHeight)
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr) {
        return {};
    }
    return system->ExtractTrackedFeatures(leftImageWidth, leftImageHeight,
                                          rightImageWidth, rightImageHeight);
}

Core::Ports::TrackedPointCloudSnapshot
OrbSlam3Runtime::ExtractTrackedPointCloud(size_t maxPointCloudPoints)
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr) {
        return {};
    }
    return system->ExtractTrackedPointCloud(maxPointCloudPoints);
}

Core::Ports::TrackedVisualData OrbSlam3Runtime::ExtractTrackedVisualData(
    const Core::Ports::VisualMapSnapshotRequest &request)
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr) {
        return {};
    }
    return system->ExtractTrackedVisualData(
        request.leftImageWidth, request.leftImageHeight,
        request.rightImageWidth, request.rightImageHeight,
        request.includePointCloud, request.maxPointCloudPoints);
}

Core::Ports::VisualMapSnapshot OrbSlam3Runtime::ExtractVisualMapSnapshot(
    const Core::Ports::VisualMapSnapshotRequest &request)
{
    Core::Ports::VisualMapSnapshot out;
    out.summary = GetTrackedVisualSummary();
    if (request.includeFeatures) {
        out.features =
            ExtractTrackedFeatures(request.leftImageWidth, request.leftImageHeight,
                                   request.rightImageWidth,
                                   request.rightImageHeight);
    }
    if (request.includePointCloud) {
        out.pointCloud = ExtractTrackedPointCloud(request.maxPointCloudPoints);
    }
    return out;
}

void OrbSlam3Runtime::LogStereoFeatureDiagnostics(
    uint64_t frameId,
    const Core::Ports::StereoFeatureObservationPacket &observations) const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr) {
        return;
    }
    system->LogStereoFeatureDfx(frameId, observations);
}

bool OrbSlam3Runtime::Optimize(
    const Core::Ports::SlamBackendOptimizationRequest &request,
    Core::Ports::SlamBackendOptimizationResult &result)
{
    result = {};
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    return system != nullptr && system->OptimizeBackend(request, result);
}

bool OrbSlam3Runtime::ApplyMappingOperation(
    const Core::Ports::SlamBackendMappingRequest &request,
    Core::Ports::SlamBackendMappingResult &result)
{
    result = {};
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    return system != nullptr &&
           system->ApplyLocalMappingOperation(request, result);
}

bool OrbSlam3Runtime::ApplyLoopClosureOperation(
    const Core::Ports::SlamBackendLoopClosureRequest &request,
    Core::Ports::SlamBackendLoopClosureResult &result)
{
    result = {};
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    return system != nullptr &&
           system->ApplyLoopClosingOperation(request, result);
}

} // namespace SmartDrone::Adapters::Slam
