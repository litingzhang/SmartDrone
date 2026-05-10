#include "adapters/slam/slam_tracking_backend.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

#include <sophus/se3.hpp>

#include "KeyFrame.h"
#include "Map.h"
#include "TrackedVisualData.h"
#include "adapters/slam/slam_engine_access.h"
#include "adapters/slam/slam_mode_common.h"

namespace smartdrone::adapters::slam {

namespace {

bool EnvFlag(const char *name, bool fallback)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    const std::string text(value);
    return !(text == "0" || text == "false" || text == "FALSE" || text == "off" || text == "OFF" ||
             text == "no" || text == "NO");
}

void LogExternalStereoDfx(uint64_t frameId, const ORB_SLAM3::ExternalStereoFrameData &externalData,
                          ORB_SLAM3::System &system)
{
    if (!EnvFlag("SMART_DRONE_SP_LG_TRACK_DFX", false)) {
        return;
    }

    int validDisparityCount = 0;
    double disparitySum = 0.0;
    double minDisparity = std::numeric_limits<double>::infinity();
    double maxDisparity = 0.0;
    const size_t pairCount = std::min(externalData.leftKeypoints.size(), externalData.rightKeypoints.size());
    for (size_t i = 0; i < pairCount; ++i) {
        const double disparity =
            static_cast<double>(externalData.leftKeypoints[i].pt.x - externalData.rightKeypoints[i].pt.x);
        if (!(disparity > 0.0) || !std::isfinite(disparity)) {
            continue;
        }
        ++validDisparityCount;
        disparitySum += disparity;
        minDisparity = std::min(minDisparity, disparity);
        maxDisparity = std::max(maxDisparity, disparity);
    }
    const double meanDisparity = validDisparityCount > 0 ? disparitySum / static_cast<double>(validDisparityCount) : 0.0;
    const ORB_SLAM3::Tracking *tracker = system.GetTracker();
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
              << " state=" << system.GetTrackingState()
              << " matches_inliers=" << system.GetMatchesInliers()
              << " tracked_map_points=" << system.GetTrackedMapPointCount()
              << " local_map_points=" << system.GetLocalMapPointCount()
              << " map_id=" << system.GetCurrentMapId()
              << "\n";
}

bool TrackingStateCanPublishPose(int trackingState)
{
    return trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::RECENTLY_LOST ||
           trackingState == ORB_SLAM3::Tracking::OK_KLT;
}

bool ExternalStereoStateDfxEnabled()
{
    return EnvFlag("SMART_DRONE_SP_LG_STATE_DFX", false) ||
           EnvFlag("SMART_DRONE_EXTERNAL_STEREO_TRACK_DFX", false);
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

} // namespace

core::ports::SlamOutput RunSlamTrackingBackend(SlamEngineAdapter &engine,
                                                  const core::ports::SlamInputBatch &input,
                                                  bool extractFeatures, bool extractPointCloud,
                                                  const ExternalStereoTrackRequest *externalRequest)
{
    core::ports::SlamOutput out{};
    ORB_SLAM3::System *system = SlamEngineAccess::System(engine);
    if (system == nullptr) {
        return out;
    }

    SlamModeSharedState &state = SlamEngineAccess::ModeState(engine);
    const SlamInputMode inputMode = SlamEngineAccess::InputMode(engine);
    const bool useImu = SlamEngineAccess::UseImu(engine);
    const bool monoMode = (inputMode != SlamInputMode::Stereo);
    const cv::Mat &monoImage =
        (inputMode == SlamInputMode::MonoRight) ? input.stereo.right.gray : input.stereo.left.gray;

    const bool trackWithExternalStereo =
        externalRequest != nullptr && externalRequest->enabled && !monoMode && !externalRequest->leftPrepared.empty() &&
        !externalRequest->rightPrepared.empty();
    if (!trackWithExternalStereo && state.m_lastSuperPointImageCount == 0) {
        state.ResetExternalFeatureStats();
    }

    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    out.usedSuperPointFrontend = false;
    state.CopyExternalFeatureStatsToOutput(out);

    Sophus::SE3f tcw;
    if (trackWithExternalStereo) {
        out.inputPrepareMs = externalRequest->inputPrepareMs;
        out.frontendMs = externalRequest->frontendMs;
        out.stereoPairMs = externalRequest->stereoPairMs;
        out.externalPackMs = externalRequest->externalPackMs;
        out.monoAugmentMs = externalRequest->monoAugmentMs;
        state.m_lastSuperPointExternalHash = externalRequest->externalHash;

        const auto trackStartTp = std::chrono::steady_clock::now();
        if (useImu) {
            tcw = system->TrackStereoPreparedWithFeatures(externalRequest->leftPrepared, externalRequest->rightPrepared,
                                                          externalRequest->externalData, input.frameTimeSec,
                                                          input.imu);
        } else {
            tcw = system->TrackStereoPreparedWithFeatures(externalRequest->leftPrepared, externalRequest->rightPrepared,
                                                          externalRequest->externalData, input.frameTimeSec);
        }
        const auto trackEndTp = std::chrono::steady_clock::now();
        LogExternalStereoDfx(input.frameId, externalRequest->externalData, *system);
        if (externalRequest->recordTotalMs) {
            state.m_lastSuperPointTotalMs =
                std::chrono::duration<double, std::milli>(trackEndTp - externalRequest->totalStartTp).count();
        }
        out.orbTrackMs = std::chrono::duration<double, std::milli>(trackEndTp - trackStartTp).count();
        out.frontendMs = state.m_lastSuperPointFrontendMs > 0.0 ? state.m_lastSuperPointFrontendMs : out.frontendMs;
        state.CopyExternalFeatureStatsToOutput(out);
        out.usedSuperPointFrontend = true;
        out.leftFeatures = externalRequest->leftFeaturePoints;
        out.rightFeatures = externalRequest->rightFeaturePoints;
    } else {
        const auto orbTrackStartTp = std::chrono::steady_clock::now();
        if (useImu) {
            if (monoMode) {
                tcw = system->TrackMonocular(monoImage, input.frameTimeSec, input.imu);
            } else {
                tcw = system->TrackStereo(input.stereo.left.gray, input.stereo.right.gray, input.frameTimeSec,
                                          input.imu);
            }
        } else {
            if (monoMode) {
                tcw = system->TrackMonocular(monoImage, input.frameTimeSec);
            } else {
                tcw = system->TrackStereo(input.stereo.left.gray, input.stereo.right.gray, input.frameTimeSec);
            }
        }
        const auto orbTrackEndTp = std::chrono::steady_clock::now();
        out.orbTrackMs = std::chrono::duration<double, std::milli>(orbTrackEndTp - orbTrackStartTp).count();
    }

    if (ORB_SLAM3::Tracking *tracker = system->GetTracker()) {
        out.orbExtractMs = tracker->mCurrentFrame.mTimeORB_Ext;
        out.orbStereoMatchMs = tracker->mCurrentFrame.mTimeStereoMatch;
    }

    out.trackingState = system->GetTrackingState();
    out.mapId = system->GetCurrentMapId();
    out.matchesInliers = system->GetMatchesInliers();
    out.trackedMapPointCount = static_cast<uint32_t>(system->GetTrackedMapPointCount());
    out.localMapPointCount = static_cast<uint32_t>(system->GetLocalMapPointCount());
    out.localMapPointHash = system->GetLocalMapPointHash();
    out.matchedMapPointHashBeforePoseOptimization = system->GetMatchedMapPointHashBeforePoseOptimization();
    out.trackedMapPointHash = system->GetTrackedMapPointHash();
    const ORB_SLAM3::LocalMappingWaitStats localMappingWaitStats = system->GetLastLocalMappingWaitStats();
    out.localMappingWaitMs = localMappingWaitStats.waitMs;
    out.localMappingWaitQueueBefore = localMappingWaitStats.queueBefore;
    out.localMappingWaitQueueAfter = localMappingWaitStats.queueAfter;
    out.localMappingWaitTimeoutMs = localMappingWaitStats.timeoutMs;
    out.localMappingWaitRequested = localMappingWaitStats.requested;
    out.localMappingWaitTimedOut = localMappingWaitStats.timedOut;
    out.localMappingAcceptingBefore = localMappingWaitStats.acceptingBefore;
    out.localMappingAcceptingAfter = localMappingWaitStats.acceptingAfter;
    if (ORB_SLAM3::Tracking *tracker = system->GetTracker()) {
        out.closeMapPointCount = static_cast<uint32_t>(std::max(0, tracker->mCurrentFrame.mnCloseMPs));
        out.orbFrameId = static_cast<uint64_t>(tracker->mCurrentFrame.mnId);
        out.referenceKeyFrameId = tracker->mCurrentFrame.mpReferenceKF != nullptr
                                      ? static_cast<int64_t>(tracker->mCurrentFrame.mpReferenceKF->mnId)
                                      : -1;
        ORB_SLAM3::KeyFrame *lastKeyFrame = tracker->GetLastKeyFrame();
        out.lastKeyFrameId = lastKeyFrame != nullptr ? static_cast<int64_t>(lastKeyFrame->mnId) : -1;
        out.lastKeyFrameFrameId = lastKeyFrame != nullptr ? static_cast<int64_t>(lastKeyFrame->mnFrameId) : -1;
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
                static_cast<int>(tracker->mCurrentFrame.mnId - static_cast<unsigned long>(out.externalStereoInitFrameId));
            const int stabilizingFrameWindow = ExternalStereoStabilizingFrameWindow();
            const uint32_t bootstrapKeyframeLimit = ExternalStereoBootstrapKeyframeLimit();
            const uint32_t stabilizingKeyframeLimit = ExternalStereoStabilizingKeyframeLimit();
            out.externalStereoStabilizing =
                framesSinceInit <= stabilizingFrameWindow && out.keyFramesInMap <= stabilizingKeyframeLimit;
            out.externalStereoBootstrap =
                out.externalStereoStabilizing && out.keyFramesInMap <= bootstrapKeyframeLimit;
        }
        if (ExternalStereoStateDfxEnabled() && out.externalStereoInjected) {
            std::cerr << "[sp_lg_state_dfx] frame_id=" << input.frameId
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

    Sophus::SE3f twc = tcw.inverse();
    bool trajectoryPoseLost = false;
    if (EnvFlag("SMART_DRONE_ORB_LIVE_EUROC_TRAJECTORY_POSE", false)) {
        Sophus::SE3f trajectoryTwc;
        if (system->GetLatestFrameTrajectoryPoseEuRoC(trajectoryTwc, nullptr, &trajectoryPoseLost)) {
            twc = trajectoryTwc;
        }
    }
    const Eigen::Vector3f t = twc.translation();
    const Eigen::Quaternionf q(twc.so3().unit_quaternion());
    const bool finitePose = std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()) &&
                            std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) &&
                            std::isfinite(q.z());
    out.poseValid = finitePose && TrackingStateCanPublishPose(out.trackingState) && !trajectoryPoseLost;
    out.pose.valid = out.poseValid;
    out.pose.x = t.x();
    out.pose.y = t.y();
    out.pose.z = t.z();
    out.pose.qw = q.w();
    out.pose.qx = q.x();
    out.pose.qy = q.y();
    out.pose.qz = q.z();

    SlamEngineAccess::GateRealtimePoseQuality(engine, out, input.frameTimeSec);
    out.pose.valid = out.poseValid;
    if (EnvFlag("SMART_DRONE_REALTIME_POSE_CONTINUITY", true)) {
        SlamEngineAccess::MaintainRealtimePoseContinuity(engine, out.pose, out.poseValid, input.frameTimeSec,
                                                         out.trackingState);
        if (out.poseValid && !TrackingStateCanPublishPose(out.trackingState) &&
            !IsIdentityPose(out.pose) &&
            EnvFlag("SMART_DRONE_REALTIME_POSE_CONTINUITY_MARK_RECENTLY_LOST", true)) {
            out.trackingState = ORB_SLAM3::Tracking::RECENTLY_LOST;
        }
        out.pose.valid = out.poseValid;
    }

    if (EnvFlagEnabled("SMART_DRONE_POSE_STABILIZER", false)) {
        SlamEngineAccess::StabilizeOutputPose(engine, out.pose, out.poseValid, input.frameTimeSec,
                                                  out.trackingState);
    }

    const bool needVisualExtraction = extractPointCloud || extractFeatures;
    if (!needVisualExtraction) {
        return out;
    }

    const int leftWidth = monoMode ? monoImage.cols : input.stereo.left.gray.cols;
    const int leftHeight = monoMode ? monoImage.rows : input.stereo.left.gray.rows;
    ORB_SLAM3::TrackedVisualData visual =
        system->ExtractTrackedVisualData(leftWidth, leftHeight, monoMode ? 0 : input.stereo.right.gray.cols,
                                         monoMode ? 0 : input.stereo.right.gray.rows, extractPointCloud, 120);
    out.matchesInliers = visual.matchesInliers;
    out.trackedMapPointCount = static_cast<uint32_t>(visual.trackedMapPointCount);
    out.localMapPointCount = static_cast<uint32_t>(visual.localMapPointCount);
    out.localMapPointHash = visual.localMapPointHash;
    out.matchedMapPointHashBeforePoseOptimization = visual.matchedMapPointHashBeforePoseOptimization;
    out.trackedMapPointHash = visual.trackedMapPointHash;
    if (extractFeatures && !out.usedSuperPointFrontend) {
        if (inputMode == SlamInputMode::MonoRight) {
            out.rightFeatures = std::move(visual.leftFeatures);
        } else {
            out.leftFeatures = std::move(visual.leftFeatures);
            out.rightFeatures = std::move(visual.rightFeatures);
        }
    }
    if (extractPointCloud) {
        out.pointCloudXyz = std::move(visual.pointCloudXyz);
    }
    return out;
}

} // namespace smartdrone::adapters::slam
