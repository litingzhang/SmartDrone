#pragma once

#include <memory>

#include "core/application/session/epg/messages/slam_epg_messages.h"

namespace SmartDrone::Core::Application {

struct SlamTaskStepResult {
    bool sessionAvailable{false};
    bool sessionOk{true};
    bool abortRequested{false};
    std::uint64_t resourceWaitUs{0};
};

struct SlamPrepareFrameResult : SlamTaskStepResult {
    std::shared_ptr<ISlamPreparedFramePayload> frame;
};

struct SlamTrackFrameResult : SlamTaskStepResult {
    std::shared_ptr<ISlamTrackedFramePayload> frame;
};

struct SlamPublishFrameResult : SlamTaskStepResult {
    std::shared_ptr<ISlamPublishedFramePayload> frame;
};

} // namespace SmartDrone::Core::Application
