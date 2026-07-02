#include "core/domain/runtime_mode.h"

namespace SmartDrone::Core::Domain {

const char *ToString(RuntimeMode mode)
{
    switch (mode) {
    case RuntimeMode::Idle:
        return "idle";
    case RuntimeMode::Slam:
        return "slam";
    case RuntimeMode::Calib:
        return "calib";
    case RuntimeMode::Playback:
        return "playback";
    default:
        return "unknown";
    }
}

const char *ToString(PerceptionMode mode)
{
    switch (mode) {
    case PerceptionMode::Stereo:
        return "stereo";
    case PerceptionMode::StereoImu:
        return "stereo-imu";
    case PerceptionMode::Mono:
        return "mono";
    case PerceptionMode::MonoImu:
        return "mono-imu";
    case PerceptionMode::Rgbd:
        return "rgbd";
    default:
        return "unknown";
    }
}

const char *ToString(SlamOperationMode mode)
{
    switch (mode) {
    case SlamOperationMode::Mapping:
        return "mapping";
    case SlamOperationMode::Localization:
        return "localization";
    case SlamOperationMode::Relocalization:
        return "relocalization";
    case SlamOperationMode::TrackingOnly:
        return "tracking-only";
    case SlamOperationMode::Auto:
        return "auto";
    default:
        return "unknown";
    }
}

const char *ToString(Px4PoseOutputMode mode)
{
    switch (mode) {
    case Px4PoseOutputMode::None:
        return "none";
    case Px4PoseOutputMode::Position:
        return "position";
    case Px4PoseOutputMode::PositionVelocity:
        return "position_velocity";
    default:
        return "unknown";
    }
}

} // namespace SmartDrone::Core::Domain
