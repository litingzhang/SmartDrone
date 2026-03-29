#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

#include "adapters/telemetry/px4_mavlink_gateway.hpp"
#include "common/tlv/tlv_protocol.hpp"
#include "common/tlv/udp_server.hpp"

namespace smartdrone::core::application {

struct LivePoseState {
    struct Snapshot {
        bool hasPeer{false};
        UdpPeer peer{};
        bool poseValid{false};
        uint8_t runtimeMode{RUNTIME_MODE_IDLE};
        uint8_t trackingState{0xFF};
        OdomQualityMode odomQuality{OdomQualityMode::LOST};
        uint16_t resetCounter{0};
        uint16_t resetMapCount{0};
        float x{0.0f}, y{0.0f}, z{0.0f};
        float qw{1.0f}, qx{0.0f}, qy{0.0f}, qz{0.0f};
        uint32_t seq{0};
        std::vector<float> pointCloudXyz;
        uint32_t pointCloudSeq{0};
    };

    void UpdatePeer(const UdpPeer& peer)
    {
        if (!peer.valid) return;
        std::lock_guard<std::mutex> lock(mu);
        latestPeer = peer;
        hasPeer = true;
    }

    void SetRuntimeMode(uint8_t mode)
    {
        std::lock_guard<std::mutex> lock(mu);
        runtimeMode = mode;
        if (mode != RUNTIME_MODE_SLAM) {
            poseValid = false;
            trackingState = 0xFF;
            odomQuality = OdomQualityMode::LOST;
        }
        dirty = true;
    }

    void UpdatePose(uint8_t mode, uint8_t tracking, uint16_t resetCounterIn, uint16_t resetMapCountIn,
                    const Px4MavlinkGateway::Pose& p, OdomQualityMode quality)
    {
        std::lock_guard<std::mutex> lock(mu);
        runtimeMode = mode;
        trackingState = tracking;
        odomQuality = quality;
        resetCounter = resetCounterIn;
        resetMapCount = resetMapCountIn;
        x = p.x; y = p.y; z = p.z;
        qw = p.qw; qx = p.qx; qy = p.qy; qz = p.qz;
        poseValid = true;
        dirty = true;
    }

    void UpdatePointCloud(std::vector<float> xyz)
    {
        std::lock_guard<std::mutex> lock(mu);
        pointCloudXyz = std::move(xyz);
        ++pointCloudSeq;
        dirty = true;
    }

    bool ConsumeSnapshot(Snapshot& out)
    {
        std::lock_guard<std::mutex> lock(mu);
        if (!hasPeer || !dirty) return false;
        out.hasPeer = hasPeer;
        out.peer = latestPeer;
        out.poseValid = poseValid;
        out.runtimeMode = runtimeMode;
        out.trackingState = trackingState;
        out.odomQuality = odomQuality;
        out.resetCounter = resetCounter;
        out.resetMapCount = resetMapCount;
        out.x = x; out.y = y; out.z = z;
        out.qw = qw; out.qx = qx; out.qy = qy; out.qz = qz;
        out.seq = ++txSeq;
        out.pointCloudXyz = pointCloudXyz;
        out.pointCloudSeq = pointCloudSeq;
        dirty = false;
        return true;
    }

    bool ReadSnapshot(Snapshot& out) const
    {
        std::lock_guard<std::mutex> lock(mu);
        if (!hasPeer) return false;
        out.hasPeer = hasPeer;
        out.peer = latestPeer;
        out.poseValid = poseValid;
        out.runtimeMode = runtimeMode;
        out.trackingState = trackingState;
        out.odomQuality = odomQuality;
        out.resetCounter = resetCounter;
        out.resetMapCount = resetMapCount;
        out.x = x; out.y = y; out.z = z;
        out.qw = qw; out.qx = qx; out.qy = qy; out.qz = qz;
        out.seq = txSeq;
        out.pointCloudXyz = pointCloudXyz;
        out.pointCloudSeq = pointCloudSeq;
        return true;
    }

    mutable std::mutex mu;
    UdpPeer latestPeer{};
    bool hasPeer{false};
    bool poseValid{false};
    uint8_t runtimeMode{RUNTIME_MODE_IDLE};
    uint8_t trackingState{0xFF};
    OdomQualityMode odomQuality{OdomQualityMode::LOST};
    uint16_t resetCounter{0};
    uint16_t resetMapCount{0};
    float x{0.0f}, y{0.0f}, z{0.0f};
    float qw{1.0f}, qx{0.0f}, qy{0.0f}, qz{0.0f};
    std::vector<float> pointCloudXyz;
    uint32_t pointCloudSeq{0};
    uint32_t txSeq{1};
    bool dirty{false};
};

}  // namespace smartdrone::core::application
