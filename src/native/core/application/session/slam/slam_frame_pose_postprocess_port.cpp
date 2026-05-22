#include "core/application/session/slam/slam_frame_pose_postprocess_port.h"

#include <atomic>
#include <chrono>
#include <iostream>
#include <utility>

#include <Eigen/Geometry>

#include "core/application/config/runtime_app_types.h"
#include "core/application/session/slam/slam_processing_support.h"
#include "core/application/session/slam/slam_runtime_control_port.h"
#include "core/application/session/slam/slam_settings_loader.h"
#include "core/application/state/live_pose_state.h"

namespace SmartDrone::Core::Application {
namespace {

constexpr uint64_t kPoseAxisLogEveryNFrames = 30;

uint8_t ComposeResetCounter(uint8_t sessionBase, uint8_t continuityCounter)
{
    return static_cast<uint8_t>(sessionBase + continuityCounter);
}

uint16_t ComposeResetMapCount(uint16_t sessionBase,
                              uint16_t continuityResetMapCount)
{
    return static_cast<uint16_t>(sessionBase + continuityResetMapCount);
}

struct RuntimeTbcOverride {
    float tx{0.0f};
    float ty{0.0f};
    float tz{0.0f};
    float rollDeg{0.0f};
    float pitchDeg{0.0f};
    float yawDeg{0.0f};
};

Sophus::SE3f BuildBodyToCamFromRuntimeOverride(
    const RuntimeTbcOverride &overrideValue)
{
    constexpr float kDegToRad = 0.017453292519943295769f;
    const float rollRad = overrideValue.rollDeg * kDegToRad;
    const float pitchRad = overrideValue.pitchDeg * kDegToRad;
    const float yawRad = overrideValue.yawDeg * kDegToRad;
    const Eigen::AngleAxisf rollRotation(rollRad, Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf pitchRotation(pitchRad, Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf yawRotation(yawRad, Eigen::Vector3f::UnitZ());
    const Eigen::Quaternionf q = yawRotation * pitchRotation * rollRotation;
    return Sophus::SE3f(
        Sophus::SO3f(q),
        Eigen::Vector3f(overrideValue.tx, overrideValue.ty, overrideValue.tz));
}

Sophus::SE3f BuildBodyToCamPitchDelta(float pitchDeg)
{
    constexpr float kDegToRad = 0.017453292519943295769f;
    const Eigen::AngleAxisf pitchRotation(pitchDeg * kDegToRad,
                                          Eigen::Vector3f::UnitY());
    return Sophus::SE3f(Sophus::SO3f(Eigen::Quaternionf(pitchRotation)),
                        Eigen::Vector3f::Zero());
}

} // namespace

SlamFramePosePostprocessPort::SlamFramePosePostprocessPort(
    SlamFramePosePostprocessContext &context,
    SlamFramePosePostprocessState &state,
    SlamFrameSharedState &sharedState)
    : m_ctx(context), m_state(state), m_sharedState(sharedState)
{
}

SlamFrameStepResult SlamFramePosePostprocessPort::PostprocessTrackedFrame(
    std::shared_ptr<SlamTrackedFrameData> tracked,
    SlamPublishedFrameData &published)
{
    if (!tracked || !tracked->frame) {
        return SlamFrameStepResult::Continue;
    }

    const auto postStartTp = std::chrono::steady_clock::now();
    const TrackingContext tracking = ResolveTrackingContext(*tracked);
    const Sophus::SE3f twcRaw = ResolveRawPose(*tracked, tracking);
    const StereoExtrinsicsContext extrinsics = ResolveStereoExtrinsics();
    const PosePostprocessor::ProcessRequest poseRequest = BuildPoseRequest(
        twcRaw, tracking, tracked->frame->captureTimestampNs, extrinsics);
    const auto poseResult = m_ctx.posePostprocessor.ProcessPose(poseRequest);
    MaybeLogPoseAxis(*tracked, tracking, twcRaw, poseResult);
    const auto postEndTp = std::chrono::steady_clock::now();

    UpdateAutoSlamMode(*tracked, tracking, poseResult,
                       tracked->frame->frameGapMs);
    const PostprocessArtifacts artifacts = BuildPostprocessArtifacts(
        *tracked, tracking, poseResult, postStartTp, postEndTp);
    FillPublishedFrame(std::move(tracked), artifacts, published);
    return SlamFrameStepResult::Continue;
}

SlamFramePosePostprocessPort::TrackingContext
SlamFramePosePostprocessPort::ResolveTrackingContext(
    const SlamTrackedFrameData &tracked)
{
    const bool debugRightOnlyFeatures = tracked.frame->debugRightOnlyFeatures;
    TrackingContext tracking{};
    tracking.state = debugRightOnlyFeatures ? Ports::kSlamTrackingLost
                                            : tracked.slamOutput.trackingState;
    tracking.usable = !debugRightOnlyFeatures &&
                      Ports::IsSlamTrackingPoseUsable(tracking.state);
    tracking.mapId = debugRightOnlyFeatures ? 0UL : tracked.slamOutput.mapId;
    m_sharedState.lastTrackingState.store(tracking.state);
    m_sharedState.lastTrackingUsable.store(tracking.usable);
    if (tracking.mapId != PosePostprocessor::ContinuityMapper::kInvalidMapId &&
        tracking.mapId != m_state.lastRawMapId) {
        m_state.lastRawMapId = tracking.mapId;
    }
    return tracking;
}

Sophus::SE3f SlamFramePosePostprocessPort::ResolveRawPose(
    const SlamTrackedFrameData &tracked,
    const TrackingContext &tracking)
{
    Sophus::SE3f twcRaw =
        m_state.haveLastValidTwcRaw ? m_state.lastValidTwcRaw : Sophus::SE3f();
    if (!tracking.usable && !tracked.slamOutput.poseValid) {
        return twcRaw;
    }
    if (tracked.frame->debugRightOnlyFeatures ||
        !tracked.slamOutput.poseValid) {
        return twcRaw;
    }

    const auto &pose = tracked.slamOutput.pose;
    const Eigen::Quaternionf rawQ(pose.qw, pose.qx, pose.qy, pose.qz);
    twcRaw = Sophus::SE3f(Sophus::SO3f(rawQ),
                          Eigen::Vector3f(pose.x, pose.y, pose.z));
    m_state.lastValidTwcRaw = twcRaw;
    m_state.haveLastValidTwcRaw = true;
    return twcRaw;
}

SlamFramePosePostprocessPort::StereoExtrinsicsContext
SlamFramePosePostprocessPort::ResolveStereoExtrinsics() const
{
    StereoExtrinsicsContext extrinsics{m_ctx.stereoBodyExtrinsics.loaded,
                                       m_ctx.stereoBodyExtrinsics.Tbc};
    if (m_ctx.useImu || m_ctx.monoMode ||
        !m_ctx.tuning.useCustomTbc.load(std::memory_order_relaxed)) {
        return extrinsics;
    }

    const float pitchDeg =
        m_ctx.tuning.tbcPitchDeg.load(std::memory_order_relaxed);
    extrinsics.loaded = true;
    if (m_ctx.stereoBodyExtrinsics.loaded) {
        extrinsics.bodyToCamera =
            m_ctx.stereoBodyExtrinsics.Tbc * BuildBodyToCamPitchDelta(pitchDeg);
        return extrinsics;
    }

    const RuntimeTbcOverride overrideValue{
        m_ctx.tuning.tbcTx.load(std::memory_order_relaxed),
        m_ctx.tuning.tbcTy.load(std::memory_order_relaxed),
        m_ctx.tuning.tbcTz.load(std::memory_order_relaxed),
        m_ctx.tuning.tbcRollDeg.load(std::memory_order_relaxed),
        pitchDeg,
        m_ctx.tuning.tbcYawDeg.load(std::memory_order_relaxed)};
    extrinsics.bodyToCamera = BuildBodyToCamFromRuntimeOverride(overrideValue);
    return extrinsics;
}

PosePostprocessor::ProcessRequest
SlamFramePosePostprocessPort::BuildPoseRequest(
    const Sophus::SE3f &twcRaw, const TrackingContext &tracking,
    int64_t captureTimestampNs,
    const StereoExtrinsicsContext &extrinsics) const
{
    PosePostprocessor::ProcessRequest request{};
    request.twcRaw = twcRaw;
    request.useImu = m_ctx.useImu;
    request.trackingUsable = tracking.usable;
    request.mapId = tracking.mapId;
    request.stereoExtrinsicsLoaded = extrinsics.loaded;
    request.stereoBodyExtrinsics = extrinsics.bodyToCamera;
    request.stereoReferencePoseSet = &m_state.stereoReferencePoseSet;
    request.stereoReferencePose = &m_state.stereoReferencePose;
    request.frameNs = captureTimestampNs;
    request.readRangeSensor = m_ctx.readRangeSensor;
    return request;
}

void SlamFramePosePostprocessPort::MaybeLogPoseAxis(
    const SlamTrackedFrameData &tracked, const TrackingContext &tracking,
    const Sophus::SE3f &twcRaw,
    const PosePostprocessor::Result &poseResult) const
{
    if (m_ctx.useImu || m_ctx.monoMode || !tracked.slamOutput.poseValid ||
        (tracked.slamOutput.frameId % kPoseAxisLogEveryNFrames) != 0) {
        return;
    }

    const Eigen::Vector3f camT = twcRaw.translation();
    const auto &dbg = poseResult.debug;
    const Eigen::Vector3f frdT(poseResult.poseEstimate.x,
                               poseResult.poseEstimate.y,
                               poseResult.poseEstimate.z);
    std::cerr << "[pose_axis] frame=" << tracked.slamOutput.frameId
              << " cam_t=" << camT.x() << "," << camT.y() << ","
              << camT.z()
              << " body_t=" << dbg.bodyX << "," << dbg.bodyY << ","
              << dbg.bodyZ << " local_t=" << dbg.localX << ","
              << dbg.localY << "," << dbg.localZ
              << " frd_t=" << frdT.x() << "," << frdT.y() << ","
              << frdT.z()
              << " tbc=" << (dbg.stereoExtrinsicsApplied ? 1 : 0)
              << " ref=" << (dbg.referenceApplied ? 1 : 0)
              << " tracking=" << (tracking.usable ? 1 : 0)
              << " q=" << poseResult.poseEstimate.qw << ","
              << poseResult.poseEstimate.qx << ","
              << poseResult.poseEstimate.qy << ","
              << poseResult.poseEstimate.qz << "\n";
}

void SlamFramePosePostprocessPort::UpdateAutoSlamMode(
    const SlamTrackedFrameData &tracked, const TrackingContext &tracking,
    const PosePostprocessor::Result &poseResult, double frameGapMs)
{
    if (m_sharedState.requestedSlamMode.load() !=
        SmartDrone::Core::Domain::SlamOperationMode::Auto) {
        return;
    }

    const auto autoEffectiveMode = m_ctx.autoSlamModeController.Observe(
        tracking.usable, poseResult.quality, frameGapMs,
        tracked.slamOutput.leftFeatures.size(),
        tracked.slamOutput.rightFeatures.size());
    if (autoEffectiveMode == m_sharedState.effectiveSlamMode.load()) {
        return;
    }

    m_sharedState.effectiveSlamMode.store(autoEffectiveMode);
    if (m_ctx.slamControl != nullptr) {
        m_ctx.slamControl->SetOperationMode(autoEffectiveMode);
    }
    m_ctx.livePose.SetSlamMode(ToRuntimeSlamModeValue(autoEffectiveMode));
    std::cerr << "[slam_auto] effective_mode -> "
              << SmartDrone::Core::Domain::ToString(autoEffectiveMode)
              << " quality=" << static_cast<int>(poseResult.quality)
              << " state=" << tracking.state
              << " featL=" << tracked.slamOutput.leftFeatures.size()
              << " featR=" << tracked.slamOutput.rightFeatures.size()
              << " frame_gap_ms=" << frameGapMs << "\n";
}

SlamFramePosePostprocessPort::PostprocessArtifacts
SlamFramePosePostprocessPort::BuildPostprocessArtifacts(
    const SlamTrackedFrameData &tracked, const TrackingContext &tracking,
    const PosePostprocessor::Result &poseResult,
    std::chrono::steady_clock::time_point postStartTp,
    std::chrono::steady_clock::time_point postEndTp) const
{
    PostprocessArtifacts artifacts{};
    artifacts.poseResult = poseResult;
    artifacts.postStartTp = postStartTp;
    artifacts.postEndTp = postEndTp;
    artifacts.trackingState = tracking.state;
    artifacts.trackingUsable = tracking.usable;
    artifacts.effectiveResetCounter =
        ComposeResetCounter(m_state.sessionResetCounterBase,
                            poseResult.resetCounter);
    artifacts.effectiveResetMapCount =
        ComposeResetMapCount(m_state.sessionResetMapCountBase,
                             poseResult.resetMapCount);
    artifacts.pointCount =
        tracked.frame->sendMap ? tracked.slamOutput.pointCloudXyz.size() / 3
                               : 0;
    return artifacts;
}

void SlamFramePosePostprocessPort::FillPublishedFrame(
    std::shared_ptr<SlamTrackedFrameData> tracked,
    const PostprocessArtifacts &artifacts,
    SlamPublishedFrameData &published) const
{
    published.frame = std::move(tracked);
    published.poseResult = artifacts.poseResult;
    published.cloudStartTp = artifacts.postEndTp;
    published.cloudEndTp = artifacts.postEndTp;
    published.udpStartTp = artifacts.postEndTp;
    published.udpEndTp = artifacts.postEndTp;
    published.postStartTp = artifacts.postStartTp;
    published.postEndTp = artifacts.postEndTp;
    published.livePoseStartTp = artifacts.postEndTp;
    published.livePoseEndTp = artifacts.postEndTp;
    published.publishStartTp = artifacts.postEndTp;
    published.publishEndTp = artifacts.postEndTp;
    published.pointCount = artifacts.pointCount;
    published.trackingState = artifacts.trackingState;
    published.trackingUsable = artifacts.trackingUsable;
    published.effectiveResetCounter = artifacts.effectiveResetCounter;
    published.effectiveResetMapCount = artifacts.effectiveResetMapCount;
}

} // namespace SmartDrone::Core::Application
