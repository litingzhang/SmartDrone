#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

#include "common/epg/epg.h"
#include "core/ports/camera_provider.h"

namespace smartdrone::core::application {

struct SlamResourceReady {
    bool ready{false};
};

struct SlamTick {
    std::uint64_t sequence{0};
};

struct SlamImuReady {
    bool ready{false};
};

struct SlamFrameReady {
    std::uint64_t sessionId{0};
};

struct SlamPreparedFramePayload {
    std::shared_ptr<void> handle;
};

struct SlamTrackedFramePayload {
    std::shared_ptr<void> handle;
};

struct SlamPublishedFramePayload {
    std::shared_ptr<void> handle;
};

struct SlamPreparedFrame {
    std::uint64_t sessionId{0};
    std::shared_ptr<SlamPreparedFramePayload> frame;
};

struct SlamTrackedFrame {
    std::uint64_t sessionId{0};
    std::shared_ptr<SlamTrackedFramePayload> frame;
};

struct SlamPublishedFrame {
    std::uint64_t sessionId{0};
    std::shared_ptr<SlamPublishedFramePayload> frame;
};

struct SlamStatus {
    bool sessionOk{true};
    bool abortRequested{false};
};

struct CalibResourceReady {
    bool ready{false};
};

struct CalibTick {
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

struct CalibStopRequest {
    bool sessionOk{false};
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

struct CalibStatus {
    bool sessionOk{true};
    bool completed{false};
};

} // namespace smartdrone::core::application
