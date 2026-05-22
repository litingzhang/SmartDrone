#include "core/application/session/slam/slam_session_processing_port.h"

#include <utility>

#include "core/application/session/slam/slam_frame_stage_data.h"
#include "core/application/session/slam/slam_frame_step_result.h"
#include "core/application/session/slam/slam_session_runtime.h"

namespace SmartDrone::Core::Application {
namespace {

struct SlamPreparedFramePayload final : ISlamPreparedFramePayload {
    explicit SlamPreparedFramePayload(
        std::shared_ptr<SlamPreparedFrameData> inputFrame)
        : frame(std::move(inputFrame))
    {
    }

    std::shared_ptr<SlamPreparedFrameData> frame;

    std::shared_ptr<SlamPreparedFrameData> Frame() const override
    {
        return frame;
    }
};

struct SlamTrackedFramePayload final : ISlamTrackedFramePayload {
    explicit SlamTrackedFramePayload(
        std::shared_ptr<SlamTrackedFrameData> inputFrame)
        : frame(std::move(inputFrame))
    {
    }

    std::shared_ptr<SlamTrackedFrameData> frame;

    std::shared_ptr<SlamTrackedFrameData> Frame() const override
    {
        return frame;
    }
};

struct SlamPublishedFramePayload final : ISlamPublishedFramePayload {
    explicit SlamPublishedFramePayload(
        std::shared_ptr<SlamPublishedFrameData> inputFrame)
        : frame(std::move(inputFrame))
    {
    }

    std::shared_ptr<SlamPublishedFrameData> frame;

    std::shared_ptr<SlamPublishedFrameData> Frame() const override
    {
        return frame;
    }
};

std::shared_ptr<ISlamPreparedFramePayload> MakePreparedPayload(
    std::shared_ptr<SlamPreparedFrameData> frame)
{
    return std::make_shared<SlamPreparedFramePayload>(std::move(frame));
}

std::shared_ptr<ISlamTrackedFramePayload> MakeTrackedPayload(
    std::shared_ptr<SlamTrackedFrameData> frame)
{
    return std::make_shared<SlamTrackedFramePayload>(std::move(frame));
}

std::shared_ptr<ISlamPublishedFramePayload> MakePublishedPayload(
    std::shared_ptr<SlamPublishedFrameData> frame)
{
    return std::make_shared<SlamPublishedFramePayload>(std::move(frame));
}

std::shared_ptr<SlamPreparedFrameData> ExtractPreparedFrame(
    const std::shared_ptr<ISlamPreparedFramePayload> &payload)
{
    return payload ? payload->Frame() : nullptr;
}

std::shared_ptr<SlamTrackedFrameData> ExtractTrackedFrame(
    const std::shared_ptr<ISlamTrackedFramePayload> &payload)
{
    return payload ? payload->Frame() : nullptr;
}

SlamTaskStepResult MakeStepResult(SlamFrameStageResult result)
{
    return {
        true,
        result.sessionOk,
        result.stepResult == SlamFrameStepResult::SessionAbort,
    };
}

} // namespace

SlamTaskStepResult SlamSessionProcessingPort::StepBackend(
    SlamSessionRuntime &runtime)
{
    return MakeStepResult(runtime.StepBackend());
}

SlamPrepareFrameResult SlamSessionProcessingPort::AcquireAndPrepareFrame(
    SlamSessionRuntime &runtime)
{
    SlamPrepareFrameResult output;
    auto frame = std::make_shared<SlamPreparedFrameData>();
    const auto result = runtime.AcquireAndPrepareFrame(*frame);
    output.sessionAvailable = true;
    output.sessionOk = result.sessionOk;
    output.abortRequested =
        result.stepResult == SlamFrameStepResult::SessionAbort;
    if (!output.abortRequested && frame->slamInput.frameId != 0) {
        output.frame = MakePreparedPayload(std::move(frame));
    }
    return output;
}

SlamTrackFrameResult SlamSessionProcessingPort::TrackPreparedFrame(
    SlamSessionRuntime &runtime,
    std::shared_ptr<ISlamPreparedFramePayload> frame)
{
    SlamTrackFrameResult output;
    auto prepared = ExtractPreparedFrame(frame);
    if (!prepared) {
        return output;
    }

    auto tracked = std::make_shared<SlamTrackedFrameData>();
    const auto result =
        runtime.TrackPreparedFrame(std::move(prepared), *tracked);
    output = SlamTrackFrameResult{
        MakeStepResult(result),
        !tracked->frame ? nullptr : MakeTrackedPayload(std::move(tracked)),
    };
    return output;
}

SlamPublishFrameResult SlamSessionProcessingPort::PostprocessTrackedFrame(
    SlamSessionRuntime &runtime,
    std::shared_ptr<ISlamTrackedFramePayload> frame)
{
    SlamPublishFrameResult output;
    auto tracked = ExtractTrackedFrame(frame);
    if (!tracked) {
        return output;
    }

    auto published = std::make_shared<SlamPublishedFrameData>();
    const auto result =
        runtime.PostprocessTrackedFrame(std::move(tracked), *published);
    output = SlamPublishFrameResult{
        MakeStepResult(result),
        !published->frame ? nullptr : MakePublishedPayload(std::move(published)),
    };
    return output;
}

SlamTaskStepResult SlamSessionProcessingPort::EmitPointCloud(
    SlamSessionRuntime &runtime,
    ISlamPublishedFramePayload &frame)
{
    auto published = frame.Frame();
    if (!published) {
        return {};
    }
    return MakeStepResult(runtime.EmitPointCloud(*published));
}

SlamTaskStepResult SlamSessionProcessingPort::EmitDfx(
    SlamSessionRuntime &runtime,
    ISlamPublishedFramePayload &frame)
{
    auto published = frame.Frame();
    if (!published) {
        return {};
    }
    return MakeStepResult(runtime.EmitDfx(*published));
}

SlamTaskStepResult SlamSessionProcessingPort::EmitUdp(
    SlamSessionRuntime &runtime,
    ISlamPublishedFramePayload &frame)
{
    auto published = frame.Frame();
    if (!published) {
        return {};
    }
    return MakeStepResult(runtime.EmitUdp(*published));
}

SlamTaskStepResult SlamSessionProcessingPort::EmitMavlink(
    SlamSessionRuntime &runtime,
    ISlamPublishedFramePayload &frame)
{
    auto published = frame.Frame();
    if (!published) {
        return {};
    }
    return MakeStepResult(runtime.EmitMavlink(*published));
}

SlamTaskStepResult SlamSessionProcessingPort::EmitLivePose(
    SlamSessionRuntime &runtime,
    ISlamPublishedFramePayload &frame)
{
    auto published = frame.Frame();
    if (!published) {
        return {};
    }
    return MakeStepResult(runtime.EmitLivePose(*published));
}

} // namespace SmartDrone::Core::Application
