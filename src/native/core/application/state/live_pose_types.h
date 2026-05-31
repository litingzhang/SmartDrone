#pragma once

#include <cstdint>

#include "common/tlv/tlv_protocol.h"

namespace SmartDrone::Core::Application {

enum class LivePoseQuality : uint8_t {
    Good = 0,
    Weak = 1,
    Lost = 2,
};

enum class AvoidanceHoldReason : uint8_t {
    None = 0,
    ObstacleAhead = 1,
    ObstacleNear = 2,
    PointCloudUnavailable = 3,
    PointCloudStale = 4,
    PointCloudSparse = 5,
};

struct LivePoseValue {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float qw{1.0f};
    float qx{0.0f};
    float qy{0.0f};
    float qz{0.0f};
};

struct LivePoseUpdate {
    uint8_t runtimeMode{RUNTIME_MODE_IDLE};
    uint8_t trackingState{0xFF};
    uint16_t resetCounter{0};
    uint16_t resetMapCount{0};
    LivePoseValue pose{};
    LivePoseQuality quality{LivePoseQuality::Lost};
    bool poseValid{false};
};

struct AvoidanceTelemetry {
    bool enabled{false};
    bool activeGoal{false};
    bool holding{false};
    AvoidanceHoldReason holdReason{AvoidanceHoldReason::None};
    float nearestObstacleM{0.0f};
    uint32_t holdCount{0};
    uint32_t pointCloudAgeMs{0};
};

} // namespace SmartDrone::Core::Application
