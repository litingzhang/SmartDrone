#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "common/tlv/tlv_protocol.h"
#include "common/tlv/udp_server.h"

namespace smartdrone::core::application {

struct LivePoseState {
    struct Snapshot {
        bool hasPeer{false};
        UdpPeer peer{};
        bool poseValid{false};
        uint8_t runtimeMode{RUNTIME_MODE_IDLE};
        uint8_t slamMode{RUNTIME_SLAM_MODE_MAPPING};
        uint8_t trackingState{0xFF};
        OdomQualityMode odomQuality{OdomQualityMode::LOST};
        uint16_t resetCounter{0};
        uint16_t resetMapCount{0};
        float x{0.0f}, y{0.0f}, z{0.0f};
        float qw{1.0f}, qx{0.0f}, qy{0.0f}, qz{0.0f};
        uint32_t seq{0};
        std::shared_ptr<const std::vector<float>> pointCloudXyz;
        uint32_t pointCloudSeq{0};
    };

    void UpdatePeer(const UdpPeer &peer);
    void SetRuntimeMode(uint8_t mode);
    void SetSlamMode(uint8_t mode);
    void UpdatePose(uint8_t mode, uint8_t tracking, uint16_t resetCounterIn, uint16_t resetMapCountIn,
                    const Px4MavlinkGateway::Pose &p, OdomQualityMode quality);
    void UpdatePointCloud(std::vector<float> xyz);
    bool ConsumeSnapshot(Snapshot &out);
    bool ReadSnapshot(Snapshot &out) const;

    mutable std::mutex mu;
    UdpPeer latestPeer{};
    bool hasPeer{false};
    bool poseValid{false};
    uint8_t runtimeMode{RUNTIME_MODE_IDLE};
    uint8_t slamMode{RUNTIME_SLAM_MODE_MAPPING};
    uint8_t trackingState{0xFF};
    OdomQualityMode odomQuality{OdomQualityMode::LOST};
    uint16_t resetCounter{0};
    uint16_t resetMapCount{0};
    float x{0.0f}, y{0.0f}, z{0.0f};
    float qw{1.0f}, qx{0.0f}, qy{0.0f}, qz{0.0f};
    std::shared_ptr<const std::vector<float>> pointCloudXyz;
    uint32_t pointCloudSeq{0};
    uint32_t txSeq{1};
    bool dirty{false};
};

} // namespace smartdrone::core::application
