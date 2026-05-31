#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "common/tlv/tlv_protocol.h"
#include "common/tlv/udp_server.h"
#include "core/application/state/live_pose_types.h"

namespace SmartDrone::Core::Application {

class LivePoseState {
  public:
    struct Snapshot {
        bool hasPeer{false};
        UdpPeer peer{};
        bool poseValid{false};
        uint8_t runtimeMode{RUNTIME_MODE_IDLE};
        uint8_t slamMode{RUNTIME_SLAM_MODE_MAPPING};
        uint8_t trackingState{0xFF};
        bool armed{false};
        uint8_t px4MainMode{0};
        uint8_t px4SubMode{0};
        LivePoseQuality poseQuality{LivePoseQuality::Lost};
        uint16_t resetCounter{0};
        uint16_t resetMapCount{0};
        float x{0.0f}, y{0.0f}, z{0.0f};
        float qw{1.0f}, qx{0.0f}, qy{0.0f}, qz{0.0f};
        uint32_t seq{0};
        std::shared_ptr<const std::vector<float>> pointCloudXyz;
        uint32_t pointCloudSeq{0};
        uint64_t pointCloudUpdateUs{0};
        AvoidanceTelemetry avoidance{};
    };

    LivePoseState();
    ~LivePoseState();

    void UpdatePeer(const UdpPeer &peer);
    void SetRuntimeMode(uint8_t mode);
    void SetSlamMode(uint8_t mode);
    void SetVehicleFlightState(bool armedIn, uint8_t px4MainModeIn, uint8_t px4SubModeIn);
    void SetAvoidanceTelemetry(const AvoidanceTelemetry &telemetry);
    void UpdatePose(const LivePoseUpdate &update);
    void UpdatePointCloud(std::vector<float> xyz);
    bool ConsumeSnapshot(Snapshot &out);
    bool ReadSnapshot(Snapshot &out) const;

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Core::Application
