#pragma once

#include <cstdint>

namespace SmartDrone::Core::Domain {

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

enum class SensorMode : uint8_t {
    Stereo = 0,
    StereoImu = 1,
    Mono = 2,
    MonoImu = 3,
};

enum class SlamBackend : uint8_t {
    Klt = 0,
    OrbSlam3 = 1,
    DpvoTensorRt = 2,
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

} // namespace SmartDrone::Core::Domain

using SensorMode = SmartDrone::Core::Domain::SensorMode;
using SlamBackend = SmartDrone::Core::Domain::SlamBackend;
