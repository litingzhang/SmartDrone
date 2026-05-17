#include "adapters/slam/orb_slam3_backend.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "System.h"
#include "Tracking.h"
#include "TrackedVisualData.h"
#include "adapters/slam/slam_engine_adapter.h"
#include "adapters/slam/slam_env.h"
#include "adapters/slam/orb_feature_utils.h"
#include "core/ports/slam_tracking_state.h"

namespace smartdrone::adapters::slam {

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

bool TrackingStateCanPublishExternalDfx()
{
    return EnvFlagEnabled("SMART_DRONE_SP_LG_TRACK_DFX", false);
}

bool ExternalStereoStateDfxEnabled()
{
    return EnvFlagEnabled("SMART_DRONE_SP_LG_STATE_DFX", false) ||
           EnvFlagEnabled("SMART_DRONE_EXTERNAL_STEREO_TRACK_DFX", false);
}

int ExternalStereoStabilizingFrameWindow()
{
    return EnvIntValueClamped("SMART_DRONE_EXTERNAL_STEREO_STABILIZING_FRAME_WINDOW", 1200, 0, 100000);
}

uint32_t ExternalStereoBootstrapKeyframeLimit()
{
    return static_cast<uint32_t>(
        EnvIntValueClamped("SMART_DRONE_EXTERNAL_STEREO_BOOTSTRAP_KF_LIMIT", 12, 0, 1000));
}

uint32_t ExternalStereoStabilizingKeyframeLimit()
{
    return static_cast<uint32_t>(
        EnvIntValueClamped("SMART_DRONE_EXTERNAL_STEREO_STABILIZING_KF_LIMIT", 120, 0, 1000));
}

std::vector<ORB_SLAM3::IMU::Point> ToOrbImuPoints(const std::vector<core::ports::ImuReading> &readings)
{
    std::vector<ORB_SLAM3::IMU::Point> out;
    out.reserve(readings.size());
    for (const core::ports::ImuReading &reading : readings) {
        out.emplace_back(cv::Point3f(reading.ax, reading.ay, reading.az),
                         cv::Point3f(reading.gx, reading.gy, reading.gz),
                         static_cast<double>(reading.timestampNs) * 1e-9);
    }
    return out;
}

ORB_SLAM3::ExternalStereoFrameData ToOrbExternalStereoFrameData(
    const ExternalStereoObservationPacket &observations)
{
    ORB_SLAM3::ExternalStereoFrameData data;
    data.leftKeypoints = observations.leftKeypoints;
    data.rightKeypoints = observations.rightKeypoints;
    data.leftDescriptors = observations.leftDescriptors;
    data.rightDescriptors = observations.rightDescriptors;
    data.leftToRightMatch = observations.leftToRightMatch;
    data.matchedStereoPairs = observations.matchedStereoPairs;
    return data;
}

} // namespace

struct OrbSlam3Backend::Impl {
    std::unique_ptr<ORB_SLAM3::System> system;
};

OrbSlam3Backend::OrbSlam3Backend(std::string vocabularyPath, std::string settingsPath,
                                 SensorMode sensorMode, bool useViewer)
    : m_impl(std::make_unique<Impl>())
{
    m_impl->system = std::make_unique<ORB_SLAM3::System>(
        std::move(vocabularyPath), std::move(settingsPath), ResolveOrbSensor(sensorMode), useViewer);
}

OrbSlam3Backend::~OrbSlam3Backend() = default;

bool OrbSlam3Backend::Available() const
{
    return m_impl && m_impl->system;
}

void OrbSlam3Backend::SetOperationMode(core::domain::SlamOperationMode mode)
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr || m_operationMode == mode) {
        return;
    }

    const bool localizationOnly = mode == core::domain::SlamOperationMode::Localization ||
                                  mode == core::domain::SlamOperationMode::Relocalization ||
                                  mode == core::domain::SlamOperationMode::TrackingOnly;
    if (localizationOnly) {
        system->ActivateLocalizationMode();
    } else {
        system->DeactivateLocalizationMode();
    }
    m_operationMode = mode;
}

void OrbSlam3Backend::Shutdown()
{
    if (m_impl && m_impl->system) {
        m_impl->system->Shutdown();
    }
}

bool OrbSlam3Backend::ShutdownAndSaveTrajectoryEuRoC(const std::string &path)
{
    if (!m_impl || !m_impl->system || path.empty()) {
        return false;
    }
    m_impl->system->Shutdown();
    m_impl->system->SaveTrajectoryEuRoC(path);
    return true;
}

Sophus::SE3f OrbSlam3Backend::TrackRaw(const core::ports::SlamInputBatch &input,
                                       SlamInputMode inputMode, bool useImu)
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr) {
        return Sophus::SE3f();
    }

    const bool monoMode = inputMode != SlamInputMode::Stereo;
    const cv::Mat &monoImage =
        (inputMode == SlamInputMode::MonoRight) ? input.stereo.right.gray : input.stereo.left.gray;

    if (useImu) {
        const std::vector<ORB_SLAM3::IMU::Point> orbImu = ToOrbImuPoints(input.imu);
        if (monoMode) {
            return system->TrackMonocular(monoImage, input.frameTimeSec, orbImu);
        }
        return system->TrackStereo(input.stereo.left.gray, input.stereo.right.gray, input.frameTimeSec,
                                   orbImu);
    }

    if (monoMode) {
        return system->TrackMonocular(monoImage, input.frameTimeSec);
    }
    return system->TrackStereo(input.stereo.left.gray, input.stereo.right.gray, input.frameTimeSec);
}

Sophus::SE3f OrbSlam3Backend::TrackPreparedStereoWithFeatures(
    const core::ports::SlamInputBatch &input,
    const cv::Mat &leftPrepared,
    const cv::Mat &rightPrepared,
    const ExternalStereoObservationPacket &observations,
    bool useImu)
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr) {
        return Sophus::SE3f();
    }
    ORB_SLAM3::ExternalStereoFrameData externalData = ToOrbExternalStereoFrameData(observations);
    if (useImu) {
        const std::vector<ORB_SLAM3::IMU::Point> orbImu = ToOrbImuPoints(input.imu);
        return system->TrackStereoPreparedWithFeatures(leftPrepared, rightPrepared, externalData,
                                                       input.frameTimeSec, orbImu);
    }
    return system->TrackStereoPreparedWithFeatures(leftPrepared, rightPrepared, externalData,
                                                   input.frameTimeSec);
}

bool OrbSlam3Backend::PrepareStereoImagesForTracking(const cv::Mat &left, const cv::Mat &right,
                                                     cv::Mat &leftPrepared, cv::Mat &rightPrepared) const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    return system != nullptr && system->PrepareStereoImagesForTracking(left, right, leftPrepared, rightPrepared);
}

int OrbSlam3Backend::TrackingState() const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    return system != nullptr ? system->GetTrackingState() : core::ports::kSlamTrackingNoImagesYet;
}

int OrbSlam3Backend::TrackedMapPointCount() const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    return system != nullptr ? static_cast<int>(system->GetTrackedMapPointCount()) : 0;
}

bool OrbSlam3Backend::IsTrackingInitializing() const
{
    const int state = TrackingState();
    return state == core::ports::kSlamTrackingNoImagesYet ||
           state == core::ports::kSlamTrackingNotInitialized;
}

bool OrbSlam3Backend::IsTrackingRecovering() const
{
    const int state = TrackingState();
    return state == core::ports::kSlamTrackingRecentlyLost ||
           state == core::ports::kSlamTrackingLost;
}

bool OrbSlam3Backend::HasTrackingInitialized() const
{
    return !IsTrackingInitializing();
}

const ExternalDescriptorProvider *OrbSlam3Backend::LeftDescriptorProvider()
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    ORB_SLAM3::Tracking *tracker = system != nullptr ? system->GetTracker() : nullptr;
    ORB_SLAM3::ORBextractor *extractor = tracker != nullptr ? tracker->GetLeftORBExtractor() : nullptr;
    if (extractor == nullptr) {
        m_leftDescriptorProvider.reset();
        return nullptr;
    }
    m_leftDescriptorProvider = std::make_unique<OrbExternalDescriptorProvider>(extractor);
    return m_leftDescriptorProvider.get();
}

const ExternalDescriptorProvider *OrbSlam3Backend::RightDescriptorProvider()
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    ORB_SLAM3::Tracking *tracker = system != nullptr ? system->GetTracker() : nullptr;
    ORB_SLAM3::ORBextractor *extractor = tracker != nullptr ? tracker->GetRightORBExtractor() : nullptr;
    if (extractor == nullptr) {
        m_rightDescriptorProvider.reset();
        return nullptr;
    }
    m_rightDescriptorProvider = std::make_unique<OrbExternalDescriptorProvider>(extractor);
    return m_rightDescriptorProvider.get();
}

bool OrbSlam3Backend::GetLatestFrameTrajectoryPoseEuRoC(Sophus::SE3f &twc, double *timestamp,
                                                        bool *lost) const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    return system != nullptr && system->GetLatestFrameTrajectoryPoseEuRoC(twc, timestamp, lost);
}

void OrbSlam3Backend::CopyMapSummaryToOutput(core::ports::SlamOutput &out) const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr) {
        return;
    }
    out.mapId = system->GetCurrentMapId();
    out.matchesInliers = system->GetMatchesInliers();
    out.trackedMapPointCount = static_cast<uint32_t>(system->GetTrackedMapPointCount());
    out.localMapPointCount = static_cast<uint32_t>(system->GetLocalMapPointCount());
    out.localMapPointHash = system->GetLocalMapPointHash();
    out.matchedMapPointHashBeforePoseOptimization =
        system->GetMatchedMapPointHashBeforePoseOptimization();
    out.trackedMapPointHash = system->GetTrackedMapPointHash();
}

void OrbSlam3Backend::CopyTrackingStatsToOutput(core::ports::SlamOutput &out) const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr) {
        return;
    }

    if (ORB_SLAM3::Tracking *tracker = system->GetTracker()) {
        out.orbExtractMs = tracker->mCurrentFrame.mTimeORB_Ext;
        out.orbStereoMatchMs = tracker->mCurrentFrame.mTimeStereoMatch;
    }

    out.trackingState = system->GetTrackingState();
    CopyMapSummaryToOutput(out);

    const ORB_SLAM3::LocalMappingWaitStats localMappingWaitStats =
        system->GetLastLocalMappingWaitStats();
    out.localMappingWaitMs = localMappingWaitStats.waitMs;
    out.localMappingWaitQueueBefore = localMappingWaitStats.queueBefore;
    out.localMappingWaitQueueAfter = localMappingWaitStats.queueAfter;
    out.localMappingWaitTimeoutMs = localMappingWaitStats.timeoutMs;
    out.localMappingWaitRequested = localMappingWaitStats.requested;
    out.localMappingWaitTimedOut = localMappingWaitStats.timedOut;
    out.localMappingAcceptingBefore = localMappingWaitStats.acceptingBefore;
    out.localMappingAcceptingAfter = localMappingWaitStats.acceptingAfter;

    ORB_SLAM3::Tracking *tracker = system->GetTracker();
    if (tracker == nullptr) {
        return;
    }

    out.closeMapPointCount = static_cast<uint32_t>(std::max(0, tracker->mCurrentFrame.mnCloseMPs));
    out.orbFrameId = static_cast<uint64_t>(tracker->mCurrentFrame.mnId);
    out.referenceKeyFrameId = tracker->mCurrentFrame.mpReferenceKF != nullptr
                                  ? static_cast<int64_t>(tracker->mCurrentFrame.mpReferenceKF->mnId)
                                  : -1;
    ORB_SLAM3::KeyFrame *lastKeyFrame = tracker->GetLastKeyFrame();
    out.lastKeyFrameId = lastKeyFrame != nullptr ? static_cast<int64_t>(lastKeyFrame->mnId) : -1;
    out.lastKeyFrameFrameId =
        lastKeyFrame != nullptr ? static_cast<int64_t>(lastKeyFrame->mnFrameId) : -1;

    ORB_SLAM3::Map *currentMap = nullptr;
    if (tracker->mCurrentFrame.mpReferenceKF != nullptr) {
        currentMap = tracker->mCurrentFrame.mpReferenceKF->GetMap();
    } else if (lastKeyFrame != nullptr) {
        currentMap = lastKeyFrame->GetMap();
    }
    out.keyFramesInMap = currentMap != nullptr ? static_cast<uint32_t>(currentMap->KeyFramesInMap()) : 0U;
    out.externalStereoInitFrameId = tracker->mnExternalStereoInitFrameId;
    out.externalStereoInjected = tracker->mCurrentFrame.mbExternalStereoInjected;
    if (out.externalStereoInjected && out.externalStereoInitFrameId >= 0 &&
        tracker->mCurrentFrame.mnId >= static_cast<unsigned long>(out.externalStereoInitFrameId)) {
        const int framesSinceInit =
            static_cast<int>(tracker->mCurrentFrame.mnId -
                             static_cast<unsigned long>(out.externalStereoInitFrameId));
        const int stabilizingFrameWindow = ExternalStereoStabilizingFrameWindow();
        const uint32_t bootstrapKeyframeLimit = ExternalStereoBootstrapKeyframeLimit();
        const uint32_t stabilizingKeyframeLimit = ExternalStereoStabilizingKeyframeLimit();
        out.externalStereoStabilizing =
            framesSinceInit <= stabilizingFrameWindow && out.keyFramesInMap <= stabilizingKeyframeLimit;
        out.externalStereoBootstrap =
            out.externalStereoStabilizing && out.keyFramesInMap <= bootstrapKeyframeLimit;
    }

    if (ExternalStereoStateDfxEnabled() && out.externalStereoInjected) {
        std::cerr << "[sp_lg_state_dfx] frame_id=" << out.frameId
                  << " orb_frame=" << out.orbFrameId
                  << " ref_kf=" << out.referenceKeyFrameId
                  << " last_kf=" << out.lastKeyFrameId
                  << " last_kf_frame=" << out.lastKeyFrameFrameId
                  << " kfs=" << out.keyFramesInMap
                  << " close=" << out.closeMapPointCount
                  << " inliers=" << out.matchesInliers
                  << " tracked=" << out.trackedMapPointCount
                  << " local=" << out.localMapPointCount
                  << " external_init_frame=" << out.externalStereoInitFrameId
                  << " external_bootstrap=" << (out.externalStereoBootstrap ? "Y" : "N")
                  << " external_stabilizing=" << (out.externalStereoStabilizing ? "Y" : "N")
                  << "\n";
    }
}

TrackedVisualSnapshot OrbSlam3Backend::ExtractTrackedVisualSnapshot(
    int leftImageWidth, int leftImageHeight, int rightImageWidth, int rightImageHeight,
    bool includePointCloud, size_t maxPointCloudPoints)
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr) {
        return {};
    }
    ORB_SLAM3::TrackedVisualData visual =
        system->ExtractTrackedVisualData(leftImageWidth, leftImageHeight, rightImageWidth,
                                         rightImageHeight, includePointCloud,
                                         maxPointCloudPoints);
    TrackedVisualSnapshot out{};
    out.matchesInliers = visual.matchesInliers;
    out.trackedMapPointCount = visual.trackedMapPointCount;
    out.localMapPointCount = visual.localMapPointCount;
    out.localMapPointHash = visual.localMapPointHash;
    out.matchedMapPointHashBeforePoseOptimization = visual.matchedMapPointHashBeforePoseOptimization;
    out.trackedMapPointHash = visual.trackedMapPointHash;
    out.pointCloudXyz = std::move(visual.pointCloudXyz);
    out.leftFeatures = std::move(visual.leftFeatures);
    out.rightFeatures = std::move(visual.rightFeatures);
    return out;
}

void OrbSlam3Backend::LogExternalStereoDfx(
    uint64_t frameId, const ExternalStereoObservationPacket &observations) const
{
    ORB_SLAM3::System *system = m_impl ? m_impl->system.get() : nullptr;
    if (system == nullptr || !TrackingStateCanPublishExternalDfx()) {
        return;
    }
    ORB_SLAM3::ExternalStereoFrameData externalData = ToOrbExternalStereoFrameData(observations);

    int validDisparityCount = 0;
    double disparitySum = 0.0;
    double minDisparity = std::numeric_limits<double>::infinity();
    double maxDisparity = 0.0;
    const size_t pairCount = std::min(externalData.leftKeypoints.size(),
                                      externalData.rightKeypoints.size());
    for (size_t i = 0; i < pairCount; ++i) {
        const double disparity =
            static_cast<double>(externalData.leftKeypoints[i].pt.x -
                                externalData.rightKeypoints[i].pt.x);
        if (!(disparity > 0.0) || !std::isfinite(disparity)) {
            continue;
        }
        ++validDisparityCount;
        disparitySum += disparity;
        minDisparity = std::min(minDisparity, disparity);
        maxDisparity = std::max(maxDisparity, disparity);
    }
    const double meanDisparity =
        validDisparityCount > 0 ? disparitySum / static_cast<double>(validDisparityCount) : 0.0;
    const ORB_SLAM3::Tracking *tracker = system->GetTracker();
    const ORB_SLAM3::Frame *frame = tracker != nullptr ? &tracker->mCurrentFrame : nullptr;
    std::cerr << "[sp_lg_track_dfx] frame_id=" << frameId
              << " injected_left=" << externalData.leftKeypoints.size()
              << " injected_right=" << externalData.rightKeypoints.size()
              << " descriptor_rows=" << externalData.leftDescriptors.rows
              << "/" << externalData.rightDescriptors.rows
              << " valid_disparity=" << validDisparityCount
              << " disparity_mean=" << meanDisparity
              << " disparity_min=" << (validDisparityCount > 0 ? minDisparity : 0.0)
              << " disparity_max=" << maxDisparity
              << " frame_N=" << (frame != nullptr ? frame->N : 0)
              << " close_mps=" << (frame != nullptr ? frame->mnCloseMPs : 0)
              << " state=" << system->GetTrackingState()
              << " matches_inliers=" << system->GetMatchesInliers()
              << " tracked_map_points=" << system->GetTrackedMapPointCount()
              << " local_map_points=" << system->GetLocalMapPointCount()
              << " map_id=" << system->GetCurrentMapId()
              << "\n";
}

} // namespace smartdrone::adapters::slam
