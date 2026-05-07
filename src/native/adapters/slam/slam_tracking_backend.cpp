#include "adapters/slam/slam_tracking_backend.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

#include <sophus/se3.hpp>

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

    if (const ORB_SLAM3::Tracking *tracker = system->GetTracker()) {
        out.orbExtractMs = tracker->mCurrentFrame.mTimeORB_Ext;
        out.orbStereoMatchMs = tracker->mCurrentFrame.mTimeStereoMatch;
    }

    out.trackingState = system->GetTrackingState();
    out.mapId = system->GetCurrentMapId();
    out.matchesInliers = system->GetMatchesInliers();
    out.trackedMapPointCount = static_cast<uint32_t>(system->GetTrackedMapPointCount());
    out.localMapPointCount = static_cast<uint32_t>(system->GetLocalMapPointCount());

    const Sophus::SE3f twc = tcw.inverse();
    const Eigen::Vector3f t = twc.translation();
    const Eigen::Quaternionf q(twc.so3().unit_quaternion());
    out.poseValid = std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()) && std::isfinite(q.w()) &&
                    std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());
    out.pose.valid = out.poseValid;
    out.pose.x = t.x();
    out.pose.y = t.y();
    out.pose.z = t.z();
    out.pose.qw = q.w();
    out.pose.qx = q.x();
    out.pose.qy = q.y();
    out.pose.qz = q.z();

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
