#pragma once

#include <cstdint>
#include <memory>

#include "core/application/session/slam/slam_frame_stage_data.h"

namespace SmartDrone::Core::Application {

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

struct ISlamPreparedFramePayload {
    virtual ~ISlamPreparedFramePayload() = default;
    virtual std::shared_ptr<SlamPreparedFrameData> Frame() const = 0;
};

struct ISlamTrackedFramePayload {
    virtual ~ISlamTrackedFramePayload() = default;
    virtual std::shared_ptr<SlamTrackedFrameData> Frame() const = 0;
};

struct ISlamPublishedFramePayload {
    virtual ~ISlamPublishedFramePayload() = default;
    virtual std::shared_ptr<SlamPublishedFrameData> Frame() const = 0;
};

struct SlamPreparedFrame {
    std::uint64_t sessionId{0};
    std::shared_ptr<ISlamPreparedFramePayload> frame;
};

struct SlamKltPreparedFrame {
    std::uint64_t sessionId{0};
    std::shared_ptr<ISlamPreparedFramePayload> frame;
};

struct SlamDpvoPreparedFrame {
    std::uint64_t sessionId{0};
    std::shared_ptr<ISlamPreparedFramePayload> frame;
};

struct SlamOrbPreparedFrame {
    std::uint64_t sessionId{0};
    std::shared_ptr<ISlamPreparedFramePayload> frame;
};

struct SlamVisualFeaturePreparedFrame {
    std::uint64_t sessionId{0};
    std::shared_ptr<ISlamPreparedFramePayload> frame;
};

struct SlamTrackedFrame {
    std::uint64_t sessionId{0};
    std::shared_ptr<ISlamTrackedFramePayload> frame;
};

struct SlamPublishedFrame {
    std::uint64_t sessionId{0};
    std::shared_ptr<ISlamPublishedFramePayload> frame;
};

struct SlamPreviewReady {
    std::uint64_t sessionId{0};
    std::shared_ptr<ISlamPublishedFramePayload> frame;
};

struct SlamStatus {
    bool sessionOk{true};
    bool abortRequested{false};
};

} // namespace SmartDrone::Core::Application
