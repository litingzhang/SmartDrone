#pragma once

#include <cstdint>

namespace SmartDrone::core::domain {

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

const char *ToString(RuntimeMode mode);
const char *ToString(PerceptionMode mode);
const char *ToString(SlamOperationMode mode);

} // namespace SmartDrone::core::domain
