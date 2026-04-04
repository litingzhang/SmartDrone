#include "core/application/live_pose_state.h"

#include <utility>

namespace smartdrone::core::application {

void LivePoseState::UpdatePeer(const UdpPeer& peer)
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
        odomQuality = OdomQualityMode::LOST;
    }
    dirty = true;
}

void LivePoseState::SetSlamMode(uint8_t mode)
{
    std::lock_guard<std::mutex> lock(mu);
    slamMode = mode;
    dirty = true;
}

void LivePoseState::UpdatePose(
    uint8_t mode,
    uint8_t tracking,
    uint16_t resetCounterIn,
    uint16_t resetMapCountIn,
    const Px4MavlinkGateway::Pose& p,
    OdomQualityMode quality)
{
    std::lock_guard<std::mutex> lock(mu);
    runtimeMode = mode;
    trackingState = tracking;
    odomQuality = quality;
    resetCounter = resetCounterIn;
    resetMapCount = resetMapCountIn;
    x = p.x;
    y = p.y;
    z = p.z;
    qw = p.qw;
    qx = p.qx;
    qy = p.qy;
    qz = p.qz;
    poseValid = true;
    dirty = true;
}

void LivePoseState::UpdatePointCloud(std::vector<float> xyz)
{
    std::lock_guard<std::mutex> lock(mu);
    pointCloudXyz = std::make_shared<const std::vector<float>>(std::move(xyz));
    ++pointCloudSeq;
    dirty = true;
}

bool LivePoseState::ConsumeSnapshot(Snapshot& out)
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
    out.odomQuality = odomQuality;
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

bool LivePoseState::ReadSnapshot(Snapshot& out) const
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
    out.odomQuality = odomQuality;
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

}  // namespace smartdrone::core::application
