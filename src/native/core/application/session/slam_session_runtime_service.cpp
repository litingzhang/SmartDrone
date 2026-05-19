#include "core/application/session/slam_session_runtime_service.h"

#include <utility>

namespace smartdrone::core::application {
namespace {

template <class PayloadType, class FrameType>
std::shared_ptr<PayloadType> MakeFramePayload(std::shared_ptr<FrameType> frame)
{
    auto payload = std::make_shared<PayloadType>();
    payload->handle = std::move(frame);
    return payload;
}

template <class FrameType, class PayloadType>
std::shared_ptr<FrameType> FrameFromPayload(
    const std::shared_ptr<PayloadType> &payload)
{
    if (!payload || !payload->handle) {
        return nullptr;
    }
    return std::static_pointer_cast<FrameType>(payload->handle);
}

} // namespace

SlamSessionRuntimeService::SlamSessionRuntimeService(
    SlamSessionGraphRuntimeConfig config)
    : m_cfg(std::move(config.cfg)),
      m_tuning(config.tuning),
      m_telemetry(config.telemetry),
      m_posePublisher(config.posePublisher),
      m_stop(config.stop),
      m_livePose(config.livePose),
      m_runningFlag(config.runningFlag)
{
}

SlamSessionRuntimeService::~SlamSessionRuntimeService()
{
    Stop();
}

bool SlamSessionRuntimeService::EnsureStarted()
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (m_started) {
        return true;
    }
    if (m_startFailed) {
        return false;
    }

    m_runtime = std::make_shared<SlamSessionRuntime>(SlamSessionRuntimeConfig{
        m_cfg,
        m_tuning,
        m_telemetry,
        m_posePublisher,
        m_livePose,
        m_stop,
        m_runningFlag,
    });
    m_stopped.store(false, std::memory_order_release);
    if (!m_runtime->Start()) {
        m_runtime.reset();
        m_startFailed = true;
        m_stopped.store(true, std::memory_order_release);
        return false;
    }

    m_started = true;
    ++m_sessionId;
    return true;
}

bool SlamSessionRuntimeService::StartFailed() const
{
    std::lock_guard<std::mutex> lock(m_mu);
    return m_startFailed;
}

bool SlamSessionRuntimeService::Stopped() const
{
    return m_stopped.load(std::memory_order_acquire);
}

std::uint64_t SlamSessionRuntimeService::SessionId() const
{
    std::lock_guard<std::mutex> lock(m_mu);
    return m_started ? m_sessionId : 0;
}

void SlamSessionRuntimeService::Stop()
{
    std::shared_ptr<SlamSessionRuntime> runtime;
    {
        std::lock_guard<std::mutex> lock(m_mu);
        runtime = std::move(m_runtime);
        m_started = false;
    }
    if (runtime) {
        runtime->Stop();
    }
    m_stopped.store(true, std::memory_order_release);
}

bool SlamSessionRuntimeService::StepImuPoll()
{
    auto runtime = Runtime();
    if (!runtime) {
        return true;
    }
    return runtime->StepImuPoll();
}

bool SlamSessionRuntimeService::ImuReady() const
{
    auto runtime = Runtime();
    return runtime ? runtime->ImuReady() : false;
}

SlamTaskStepResult SlamSessionRuntimeService::StepBackend()
{
    auto runtime = Runtime();
    if (!runtime) {
        return {};
    }

    std::lock_guard<std::mutex> lock(m_processorMu);
    return MakeStepResult(
        *runtime,
        runtime->FrameProcessor().StepBackend());
}

SlamPrepareFrameResult SlamSessionRuntimeService::AcquireAndPrepareFrame(
    std::uint64_t sessionId)
{
    SlamPrepareFrameResult output;
    auto runtime = Runtime(sessionId);
    if (!runtime) {
        return output;
    }

    std::lock_guard<std::mutex> lock(m_processorMu);
    auto frame = std::make_shared<SlamFrameProcessor::PreparedFrame>();
    const auto result =
        runtime->FrameProcessor().AcquireAndPrepareFrame(runtime->sessionOk,
                                                         *frame);
    output.sessionAvailable = true;
    output.sessionOk = runtime->sessionOk;
    output.abortRequested =
        result == SlamFrameProcessor::StepResult::SessionAbort;
    if (!output.abortRequested && frame->slamInput.frameId != 0) {
        output.frame = MakeFramePayload<SlamPreparedFramePayload>(
            std::move(frame));
    }
    return output;
}

SlamTrackFrameResult SlamSessionRuntimeService::TrackPreparedFrame(
    std::uint64_t sessionId,
    std::shared_ptr<SlamPreparedFramePayload> frame)
{
    SlamTrackFrameResult output;
    auto runtime = Runtime(sessionId);
    auto prepared = FrameFromPayload<SlamFrameProcessor::PreparedFrame>(frame);
    if (!runtime || !prepared) {
        return output;
    }

    std::lock_guard<std::mutex> lock(m_processorMu);
    auto tracked = std::make_shared<SlamFrameProcessor::TrackedFrame>();
    const auto result =
        runtime->FrameProcessor().TrackPreparedFrame(std::move(prepared),
                                                     *tracked);
    output = SlamTrackFrameResult{
        MakeStepResult(*runtime, result),
        !tracked->frame ? nullptr
                        : MakeFramePayload<SlamTrackedFramePayload>(
                              std::move(tracked)),
    };
    return output;
}

SlamPublishFrameResult SlamSessionRuntimeService::PostprocessTrackedFrame(
    std::uint64_t sessionId,
    std::shared_ptr<SlamTrackedFramePayload> frame)
{
    SlamPublishFrameResult output;
    auto runtime = Runtime(sessionId);
    auto tracked = FrameFromPayload<SlamFrameProcessor::TrackedFrame>(frame);
    if (!runtime || !tracked) {
        return output;
    }

    std::lock_guard<std::mutex> lock(m_processorMu);
    auto published = std::make_shared<SlamFrameProcessor::PublishedFrame>();
    const auto result =
        runtime->FrameProcessor().PostprocessTrackedFrame(std::move(tracked),
                                                          *published);
    output = SlamPublishFrameResult{
        MakeStepResult(*runtime, result),
        !published->frame ? nullptr
                          : MakeFramePayload<SlamPublishedFramePayload>(
                                std::move(published)),
    };
    return output;
}

SlamTaskStepResult SlamSessionRuntimeService::EmitPointCloud(
    std::uint64_t sessionId,
    SlamPublishedFramePayload &frame)
{
    auto runtime = Runtime(sessionId);
    auto published =
        std::static_pointer_cast<SlamFrameProcessor::PublishedFrame>(
            frame.handle);
    if (!runtime || !published) {
        return {};
    }

    std::lock_guard<std::mutex> lock(m_processorMu);
    return MakeStepResult(*runtime,
                          runtime->FrameProcessor().EmitPointCloud(*published));
}

SlamTaskStepResult SlamSessionRuntimeService::EmitDfx(
    std::uint64_t sessionId,
    SlamPublishedFramePayload &frame)
{
    auto runtime = Runtime(sessionId);
    auto published =
        std::static_pointer_cast<SlamFrameProcessor::PublishedFrame>(
            frame.handle);
    if (!runtime || !published) {
        return {};
    }

    std::lock_guard<std::mutex> lock(m_processorMu);
    return MakeStepResult(*runtime,
                          runtime->FrameProcessor().EmitDfx(*published));
}

SlamTaskStepResult SlamSessionRuntimeService::EmitUdp(
    std::uint64_t sessionId,
    SlamPublishedFramePayload &frame)
{
    auto runtime = Runtime(sessionId);
    auto published =
        std::static_pointer_cast<SlamFrameProcessor::PublishedFrame>(
            frame.handle);
    if (!runtime || !published) {
        return {};
    }

    std::lock_guard<std::mutex> lock(m_processorMu);
    return MakeStepResult(*runtime,
                          runtime->FrameProcessor().EmitUdp(*published));
}

SlamTaskStepResult SlamSessionRuntimeService::EmitMavlink(
    std::uint64_t sessionId,
    SlamPublishedFramePayload &frame)
{
    auto runtime = Runtime(sessionId);
    auto published =
        std::static_pointer_cast<SlamFrameProcessor::PublishedFrame>(
            frame.handle);
    if (!runtime || !published) {
        return {};
    }

    std::lock_guard<std::mutex> lock(m_processorMu);
    return MakeStepResult(*runtime,
                          runtime->FrameProcessor().EmitMavlink(*published));
}

SlamTaskStepResult SlamSessionRuntimeService::EmitLivePose(
    std::uint64_t sessionId,
    SlamPublishedFramePayload &frame)
{
    auto runtime = Runtime(sessionId);
    auto published =
        std::static_pointer_cast<SlamFrameProcessor::PublishedFrame>(
            frame.handle);
    if (!runtime || !published) {
        return {};
    }

    std::lock_guard<std::mutex> lock(m_processorMu);
    return MakeStepResult(*runtime,
                          runtime->FrameProcessor().EmitLivePose(*published));
}

std::shared_ptr<SlamSessionRuntime> SlamSessionRuntimeService::Runtime() const
{
    std::lock_guard<std::mutex> lock(m_mu);
    return m_runtime;
}

std::shared_ptr<SlamSessionRuntime> SlamSessionRuntimeService::Runtime(
    std::uint64_t sessionId) const
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (!m_started || sessionId == 0 || sessionId != m_sessionId) {
        return nullptr;
    }
    return m_runtime;
}

SlamTaskStepResult SlamSessionRuntimeService::MakeStepResult(
    const SlamSessionRuntime &runtime,
    SlamFrameProcessor::StepResult result) const
{
    return {
        true,
        runtime.sessionOk,
        result == SlamFrameProcessor::StepResult::SessionAbort,
    };
}

} // namespace smartdrone::core::application
