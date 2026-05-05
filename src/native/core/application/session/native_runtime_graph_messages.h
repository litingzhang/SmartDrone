#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

#include "common/runtime_graph/runtime_graph.h"
#include "core/application/session/slam_frame_processor.h"
#include "core/application/session/slam_session_runtime.h"
#include "core/ports/camera_provider.h"

namespace smartdrone::core::application {

struct NativeSlamResourceReady {
    bool ready{false};
};

struct NativeSlamTick {
    std::uint64_t sequence{0};
};

struct NativeSlamFrameReady {
    std::shared_ptr<SlamSessionRuntime> runtime;
};

struct NativeSlamPreparedFrame {
    std::shared_ptr<SlamSessionRuntime> runtime;
    std::shared_ptr<SlamFrameProcessor::PreparedFrame> frame;
};

struct NativeSlamTrackedFrame {
    std::shared_ptr<SlamSessionRuntime> runtime;
    std::shared_ptr<SlamFrameProcessor::TrackedFrame> frame;
};

struct NativeSlamPublishedFrame {
    std::shared_ptr<SlamSessionRuntime> runtime;
    std::shared_ptr<SlamFrameProcessor::PublishedFrame> frame;
};

struct NativeSlamStatus {
    bool sessionOk{true};
    bool abortRequested{false};
};

struct CalibResourceReady {
    bool ready{false};
};

struct NativeCalibTick {
    std::uint64_t sequence{0};
};

struct CalibStereoFrame {
    smartdrone::core::ports::StereoFrame stereo;
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

struct CalibStorageStatus {
    bool ok{true};
};

struct CalibImuStatus {
    bool ok{true};
};

struct CalibPreviewStatus {
    bool ok{true};
};

struct CalibFlushRequest {
    bool sessionOk{true};
};

struct NativeCalibStatus {
    bool sessionOk{true};
    bool completed{false};
};

} // namespace smartdrone::core::application
