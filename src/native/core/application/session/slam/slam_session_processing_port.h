#pragma once

#include <memory>

#include "core/application/session/epg/messages/slam_epg_messages.h"
#include "core/application/session/epg/slam_session_task_results.h"

namespace smartdrone::core::application {

class SlamSessionRuntime;

class SlamSessionProcessingPort final {
  public:
    SlamTaskStepResult StepBackend(SlamSessionRuntime &runtime);
    SlamPrepareFrameResult AcquireAndPrepareFrame(SlamSessionRuntime &runtime);
    SlamTrackFrameResult TrackPreparedFrame(
        SlamSessionRuntime &runtime,
        std::shared_ptr<ISlamPreparedFramePayload> frame);
    SlamPublishFrameResult PostprocessTrackedFrame(
        SlamSessionRuntime &runtime,
        std::shared_ptr<ISlamTrackedFramePayload> frame);
    SlamTaskStepResult EmitPointCloud(SlamSessionRuntime &runtime,
                                      ISlamPublishedFramePayload &frame);
    SlamTaskStepResult EmitDfx(SlamSessionRuntime &runtime,
                               ISlamPublishedFramePayload &frame);
    SlamTaskStepResult EmitUdp(SlamSessionRuntime &runtime,
                               ISlamPublishedFramePayload &frame);
    SlamTaskStepResult EmitMavlink(SlamSessionRuntime &runtime,
                                   ISlamPublishedFramePayload &frame);
    SlamTaskStepResult EmitLivePose(SlamSessionRuntime &runtime,
                                    ISlamPublishedFramePayload &frame);
};

} // namespace smartdrone::core::application
