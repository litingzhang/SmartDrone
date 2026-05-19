#include "core/application/state/live_pose_state.h"

#include <utility>

namespace smartdrone::core::application {

void LivePoseState::UpdatePeer(const UdpPeer &peer)
{
    if (!peer.valid) {
        return;
    }

    std::lock_guard<std::mutex> lock(mu);
    latestPeer = peer;
    hasPeer = true;
}

void LivePoseState::SetRuntimeMode(uint8_t mode)
{
    std::lock_guard<std::mutex> lock(mu);
    runtimeMode = mode;
    if (mode != RUNTIME_MODE_SLAM) {
        poseValid = false;
        slamMode = RUNTIME_SLAM_MODE_MAPPING;
        trackingState = 0xFF;
        poseQuality = LivePoseQuality::Lost;
    }
    dirty = true;
}

void LivePoseState::SetSlamMode(uint8_t mode)
{
    std::lock_guard<std::mutex> lock(mu);
    slamMode = mode;
    dirty = true;
}

void LivePoseState::SetVehicleFlightState(bool armedIn, uint8_t px4MainModeIn, uint8_t px4SubModeIn)
{
    std::lock_guard<std::mutex> lock(mu);
    armed = armedIn;
    px4MainMode = px4MainModeIn;
    px4SubMode = px4SubModeIn;
    dirty = true;
}

void LivePoseState::UpdatePose(const LivePoseUpdate &update)
{
    std::lock_guard<std::mutex> lock(mu);
    runtimeMode = update.runtimeMode;
    trackingState = update.trackingState;
    poseQuality = update.quality;
    resetCounter = update.resetCounter;
    resetMapCount = update.resetMapCount;
    x = update.pose.x;
    y = update.pose.y;
    z = update.pose.z;
    qw = update.pose.qw;
    qx = update.pose.qx;
    qy = update.pose.qy;
    qz = update.pose.qz;
    poseValid = update.poseValid;
    dirty = true;
}

void LivePoseState::UpdatePointCloud(std::vector<float> xyz)
{
    std::lock_guard<std::mutex> lock(mu);
    pointCloudXyz = std::make_shared<const std::vector<float>>(std::move(xyz));
    ++pointCloudSeq;
    dirty = true;
}

bool LivePoseState::ConsumeSnapshot(Snapshot &out)
{
    std::lock_guard<std::mutex> lock(mu);
    if (!hasPeer || !dirty) {
        return false;
    }

    out.hasPeer = hasPeer;
    out.peer = latestPeer;
    out.poseValid = poseValid;
    out.runtimeMode = runtimeMode;
    out.slamMode = slamMode;
    out.trackingState = trackingState;
    out.armed = armed;
    out.px4MainMode = px4MainMode;
    out.px4SubMode = px4SubMode;
    out.poseQuality = poseQuality;
    out.resetCounter = resetCounter;
    out.resetMapCount = resetMapCount;
    out.x = x;
    out.y = y;
    out.z = z;
    out.qw = qw;
    out.qx = qx;
    out.qy = qy;
    out.qz = qz;
    out.seq = ++txSeq;
    out.pointCloudXyz = pointCloudXyz;
    out.pointCloudSeq = pointCloudSeq;
    dirty = false;
    return true;
}

bool LivePoseState::ReadSnapshot(Snapshot &out) const
{
    std::lock_guard<std::mutex> lock(mu);
    if (!hasPeer) {
        return false;
    }

    out.hasPeer = hasPeer;
    out.peer = latestPeer;
    out.poseValid = poseValid;
    out.runtimeMode = runtimeMode;
    out.slamMode = slamMode;
    out.trackingState = trackingState;
    out.armed = armed;
    out.px4MainMode = px4MainMode;
    out.px4SubMode = px4SubMode;
    out.poseQuality = poseQuality;
    out.resetCounter = resetCounter;
    out.resetMapCount = resetMapCount;
    out.x = x;
    out.y = y;
    out.z = z;
    out.qw = qw;
    out.qx = qx;
    out.qy = qy;
    out.qz = qz;
    out.seq = txSeq;
    out.pointCloudXyz = pointCloudXyz;
    out.pointCloudSeq = pointCloudSeq;
    return true;
}

} // namespace smartdrone::core::application
