#pragma once

#include <cstdint>

namespace smartdrone::core::domain {

enum class RuntimeMode : uint8_t {
    Idle = 0,
    Slam = 1,
    Calib = 2,
    Playback = 3,
};

enum class PerceptionMode : uint8_t {
    Stereo = 0,
    StereoImu = 1,
    Mono = 2,
    MonoImu = 3,
    Rgbd = 4,
};

enum class SlamOperationMode : uint8_t {
    Mapping = 0,
    Localization = 1,
    Relocalization = 2,
    TrackingOnly = 3,
    Auto = 4,
};

struct RuntimeSelection {
    RuntimeMode runtimeMode{RuntimeMode::Idle};
    PerceptionMode perceptionMode{PerceptionMode::Stereo};
    SlamOperationMode slamMode{SlamOperationMode::Mapping};
};

inline const char* ToString(RuntimeMode mode)
{
    switch (mode) {
        case RuntimeMode::Idle: return "idle";
        case RuntimeMode::Slam: return "slam";
        case RuntimeMode::Calib: return "calib";
        case RuntimeMode::Playback: return "playback";
        default: return "unknown";
    }
}

inline const char* ToString(PerceptionMode mode)
{
    switch (mode) {
        case PerceptionMode::Stereo: return "stereo";
        case PerceptionMode::StereoImu: return "stereo-imu";
        case PerceptionMode::Mono: return "mono";
        case PerceptionMode::MonoImu: return "mono-imu";
        case PerceptionMode::Rgbd: return "rgbd";
        default: return "unknown";
    }
}

inline const char* ToString(SlamOperationMode mode)
{
    switch (mode) {
        case SlamOperationMode::Mapping: return "mapping";
        case SlamOperationMode::Localization: return "localization";
        case SlamOperationMode::Relocalization: return "relocalization";
        case SlamOperationMode::TrackingOnly: return "tracking-only";
        case SlamOperationMode::Auto: return "auto";
        default: return "unknown";
    }
}

}  // namespace smartdrone::core::domain
