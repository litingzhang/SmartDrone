#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

#include "core/domain/imu_sample.h"
#include "core/ports/camera_provider.h"

namespace SmartDrone::Core::Application {

struct CalibResourceReady {
    bool ready{false};
};

struct CalibTick {
    std::uint64_t sequence{0};
};

struct CalibStereoFrame {
    SmartDrone::Core::Ports::StereoFrame stereo;
};

struct CalibSavePair {
    std::shared_ptr<CalibStereoFrame> frame;
    std::int64_t pairNs{0};
    std::string name;
    std::filesystem::path fnL;
    std::filesystem::path fnR;
};

struct CalibCaptureDone {
    bool sessionOk{true};
};

struct CalibStopRequest {
    bool sessionOk{false};
};

struct CalibImuSample {
    bool ok{true};
    ImuSample sample{};
};

struct CalibPreviewStatus {
    bool sessionOk{true};
};

struct CalibFlushRequest {
    bool sessionOk{true};
};

struct CalibStorageFlushed {
    bool sessionOk{true};
};

struct CalibStatus {
    bool sessionOk{true};
    bool completed{false};
};

} // namespace SmartDrone::Core::Application
