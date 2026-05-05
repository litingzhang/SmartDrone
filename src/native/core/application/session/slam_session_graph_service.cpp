#include "core/application/session/slam_session_graph_service.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "common/runtime_graph/runtime_graph.h"
#include "core/application/session/native_runtime_graph_messages.h"
#include "core/application/session/native_runtime_graph_registry.h"
#include "core/application/session/slam_frame_processor.h"
#include "core/application/session/slam_session_runtime.h"

namespace smartdrone::core::application {
namespace {

class NativeSlamRuntimeState final {
  public:
    NativeSlamRuntimeState(const UnifiedConfig &cfg, LiveRuntimeTuning &tuning, Px4MavlinkGateway &mav,
                           std::atomic<bool> &stop, LivePoseState &livePose, std::atomic<bool> &runningFlag)
        : m_cfg(cfg), m_tuning(tuning), m_mav(mav), m_stop(stop), m_livePose(livePose), m_runningFlag(runningFlag)
    {
    }

    bool EnsureStarted()
    {
        std::lock_guard<std::mutex> lock(m_mu);
        if (m_started) {
            return true;
        }
        if (m_startFailed) {
            return false;
        }

        m_runtime = std::make_shared<SlamSessionRuntime>(m_cfg, m_tuning, m_mav, m_livePose, m_stop, m_runningFlag);
        if (!m_runtime->Start()) {
            m_runtime.reset();
            m_startFailed = true;
            return false;
        }
        m_started = true;
        return true;
    }

    std::shared_ptr<SlamSessionRuntime> Runtime() const
    {
        std::lock_guard<std::mutex> lock(m_mu);
        return m_runtime;
    }

    bool StartFailed() const
    {
        std::lock_guard<std::mutex> lock(m_mu);
        return m_startFailed;
    }

    void Stop()
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
    }

  private:
    const UnifiedConfig &m_cfg;
    LiveRuntimeTuning &m_tuning;
    Px4MavlinkGateway &m_mav;
    std::atomic<bool> &m_stop;
    LivePoseState &m_livePose;
    std::atomic<bool> &m_runningFlag;
    mutable std::mutex m_mu;
    std::shared_ptr<SlamSessionRuntime> m_runtime;
    bool m_started{false};
    bool m_startFailed{false};
};

class NativeSlamResourceTask final : public smartdrone::runtime_graph::ITask {
  public:
    NativeSlamResourceTask(std::shared_ptr<NativeSlamRuntimeState> state, std::atomic<bool> &stop,
                           std::atomic<bool> &runningFlag)
        : m_state(std::move(state)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    ~NativeSlamResourceTask() override { m_state->Stop(); }

    void OnTick(smartdrone::runtime_graph::TaskContext &context) override
    {
        if (m_readyEmitted || !m_runningFlag.load() || m_stop.load()) {
            return;
        }
        if (!m_state->EnsureStarted()) {
            m_stop.store(true);
            return;
        }
        auto ready = context.Make<NativeSlamResourceReady>();
        ready->ready = true;
        if (context.Push("ready", std::move(ready))) {
            m_readyEmitted = true;
        }
    }

  private:
    std::shared_ptr<NativeSlamRuntimeState> m_state;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    bool m_readyEmitted{false};
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(
    NativeSlamResourceTask, "NativeSlamResourceTask",
    std::vector<smartdrone::runtime_graph::PortSpec>{},
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("ready", "NativeSlamResourceReady")})

class NativeSlamClockTask final : public smartdrone::runtime_graph::ITask {
  public:
    NativeSlamClockTask(std::atomic<bool> &stop, std::atomic<bool> &runningFlag)
        : m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(smartdrone::runtime_graph::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        auto tick = context.Make<NativeSlamTick>();
        tick->sequence = ++m_sequence;
        context.Push("tick", std::move(tick));
    }

  private:
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    std::uint64_t m_sequence{0};
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(
    NativeSlamClockTask, "NativeSlamClockTask",
    std::vector<smartdrone::runtime_graph::PortSpec>{},
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("tick", "NativeSlamTick")})

class NativeSlamImuGateTask final : public smartdrone::runtime_graph::ITask {
  public:
    NativeSlamImuGateTask(std::shared_ptr<NativeSlamRuntimeState> state, std::atomic<bool> &stop,
                          std::atomic<bool> &runningFlag)
        : m_state(std::move(state)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(smartdrone::runtime_graph::TaskContext &context) override
    {
        if (auto ready = context.TryPopLatest<NativeSlamResourceReady>("ready")) {
            m_resourceReady = ready->ready;
        }
        const auto tick = context.TryPopLatest<NativeSlamTick>("tick");
        if (m_state->StartFailed()) {
            PushStatus(context, false, true);
            return;
        }
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        if (!m_resourceReady) {
            return;
        }
        if (!tick) {
            return;
        }

        auto runtime = m_state->Runtime();
        if (!runtime) {
            return;
        }
        if (!runtime->WaitForImuReady()) {
            return;
        }

        auto frameReady = context.Make<NativeSlamFrameReady>();
        frameReady->runtime = std::move(runtime);
        context.Push("frame_ready", std::move(frameReady));
    }

  private:
    void PushStatus(smartdrone::runtime_graph::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<NativeSlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push("status", std::move(status));
    }

    std::shared_ptr<NativeSlamRuntimeState> m_state;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    bool m_resourceReady{false};
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(
    NativeSlamImuGateTask, "NativeSlamImuGateTask",
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("ready", "NativeSlamResourceReady"),
        SMARTDRONE_RUNTIME_GRAPH_PORT("tick", "NativeSlamTick")},
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("frame_ready", "NativeSlamFrameReady"),
        SMARTDRONE_RUNTIME_GRAPH_PORT("status", "NativeSlamStatus")})

class NativeSlamAcquireTask final : public smartdrone::runtime_graph::ITask {
  public:
    NativeSlamAcquireTask(std::shared_ptr<std::mutex> processorMu, std::atomic<bool> &stop,
                          std::atomic<bool> &runningFlag)
        : m_processorMu(std::move(processorMu)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(smartdrone::runtime_graph::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto frameReady = context.TryPopLatest<NativeSlamFrameReady>("frame_ready");
        if (!frameReady || !frameReady->runtime) {
            return;
        }

        auto &runtime = *frameReady->runtime;
        std::lock_guard<std::mutex> lock(*m_processorMu);
        SlamFrameProcessor &frameProcessor = runtime.FrameProcessor();
        auto preparedFrame = std::make_shared<SlamFrameProcessor::PreparedFrame>();
        const auto result = frameProcessor.AcquireAndPrepareFrame(runtime.sessionOk, *preparedFrame);
        if (result == SlamFrameProcessor::StepResult::SessionAbort) {
            PushStatus(context, runtime.sessionOk, true);
            return;
        }
        if (preparedFrame->slamInput.frameId == 0) {
            return;
        }

        auto prepared = context.Make<NativeSlamPreparedFrame>();
        prepared->runtime = frameReady->runtime;
        prepared->frame = std::move(preparedFrame);
        context.Push("prepared", std::move(prepared));
    }

  private:
    void PushStatus(smartdrone::runtime_graph::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<NativeSlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push("status", std::move(status));
    }

    std::shared_ptr<std::mutex> m_processorMu;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(
    NativeSlamAcquireTask, "NativeSlamAcquireTask",
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("frame_ready", "NativeSlamFrameReady")},
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("prepared", "NativeSlamPreparedFrame"),
        SMARTDRONE_RUNTIME_GRAPH_PORT("status", "NativeSlamStatus")})

class NativeSlamTrackingTask final : public smartdrone::runtime_graph::ITask {
  public:
    NativeSlamTrackingTask(std::shared_ptr<std::mutex> processorMu, std::atomic<bool> &stop,
                           std::atomic<bool> &runningFlag)
        : m_processorMu(std::move(processorMu)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(smartdrone::runtime_graph::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto prepared = context.TryPopLatest<NativeSlamPreparedFrame>("prepared");
        if (!prepared || !prepared->runtime || !prepared->frame) {
            return;
        }

        auto &runtime = *prepared->runtime;
        std::lock_guard<std::mutex> lock(*m_processorMu);
        SlamFrameProcessor &frameProcessor = runtime.FrameProcessor();
        auto trackedFrame = std::make_shared<SlamFrameProcessor::TrackedFrame>();
        const auto result = frameProcessor.TrackPreparedFrame(std::move(prepared->frame), *trackedFrame);
        if (result == SlamFrameProcessor::StepResult::SessionAbort) {
            PushStatus(context, runtime.sessionOk, true);
            return;
        }
        if (!trackedFrame->frame) {
            return;
        }

        auto tracked = context.Make<NativeSlamTrackedFrame>();
        tracked->runtime = prepared->runtime;
        tracked->frame = std::move(trackedFrame);
        context.Push("tracked", std::move(tracked));
    }

  private:
    void PushStatus(smartdrone::runtime_graph::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<NativeSlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push("status", std::move(status));
    }

    std::shared_ptr<std::mutex> m_processorMu;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(
    NativeSlamTrackingTask, "NativeSlamTrackingTask",
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("prepared", "NativeSlamPreparedFrame")},
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("tracked", "NativeSlamTrackedFrame"),
        SMARTDRONE_RUNTIME_GRAPH_PORT("status", "NativeSlamStatus")})

class NativeSlamPosePostprocessTask final : public smartdrone::runtime_graph::ITask {
  public:
    NativeSlamPosePostprocessTask(std::shared_ptr<std::mutex> processorMu, std::atomic<bool> &stop,
                          std::atomic<bool> &runningFlag)
        : m_processorMu(std::move(processorMu)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(smartdrone::runtime_graph::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto tracked = context.TryPopLatest<NativeSlamTrackedFrame>("tracked");
        if (!tracked || !tracked->runtime || !tracked->frame) {
            return;
        }

        auto &runtime = *tracked->runtime;
        std::lock_guard<std::mutex> lock(*m_processorMu);
        SlamFrameProcessor &frameProcessor = runtime.FrameProcessor();
        auto publishedFrame = std::make_shared<SlamFrameProcessor::PublishedFrame>();
        if (frameProcessor.PostprocessTrackedFrame(std::move(tracked->frame), *publishedFrame) ==
            SlamFrameProcessor::StepResult::SessionAbort) {
            PushStatus(context, runtime.sessionOk, true);
            return;
        }
        if (!publishedFrame->frame) {
            return;
        }

        auto published = context.Make<NativeSlamPublishedFrame>();
        published->runtime = tracked->runtime;
        published->frame = std::move(publishedFrame);
        context.Push("published", std::move(published));
    }

  private:
    void PushStatus(smartdrone::runtime_graph::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<NativeSlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push("status", std::move(status));
    }

    std::shared_ptr<std::mutex> m_processorMu;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(
    NativeSlamPosePostprocessTask, "NativeSlamPosePostprocessTask",
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("tracked", "NativeSlamTrackedFrame")},
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("published", "NativeSlamPublishedFrame"),
        SMARTDRONE_RUNTIME_GRAPH_PORT("status", "NativeSlamStatus")})

class NativeSlamPointCloudTask final : public smartdrone::runtime_graph::ITask {
  public:
    NativeSlamPointCloudTask(std::shared_ptr<std::mutex> processorMu, std::atomic<bool> &stop,
                             std::atomic<bool> &runningFlag)
        : m_processorMu(std::move(processorMu)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(smartdrone::runtime_graph::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto published = context.TryPopLatest<NativeSlamPublishedFrame>("published");
        if (!published || !published->runtime || !published->frame) {
            return;
        }

        auto &runtime = *published->runtime;
        {
            std::lock_guard<std::mutex> lock(*m_processorMu);
            SlamFrameProcessor &frameProcessor = runtime.FrameProcessor();
            if (frameProcessor.EmitPointCloud(*published->frame) == SlamFrameProcessor::StepResult::SessionAbort) {
                PushStatus(context, runtime.sessionOk, true);
                return;
            }
        }

        context.Push("published", published);
    }

  private:
    void PushStatus(smartdrone::runtime_graph::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<NativeSlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push("status", std::move(status));
    }

    std::shared_ptr<std::mutex> m_processorMu;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(
    NativeSlamPointCloudTask, "NativeSlamPointCloudTask",
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("published", "NativeSlamPublishedFrame")},
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("published", "NativeSlamPublishedFrame"),
        SMARTDRONE_RUNTIME_GRAPH_PORT("status", "NativeSlamStatus")})

class NativeSlamDfxTask final : public smartdrone::runtime_graph::ITask {
  public:
    NativeSlamDfxTask(std::shared_ptr<std::mutex> processorMu, std::atomic<bool> &stop,
                      std::atomic<bool> &runningFlag)
        : m_processorMu(std::move(processorMu)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(smartdrone::runtime_graph::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto published = context.TryPopLatest<NativeSlamPublishedFrame>("published");
        if (!published || !published->runtime || !published->frame) {
            return;
        }

        auto &runtime = *published->runtime;
        std::lock_guard<std::mutex> lock(*m_processorMu);
        SlamFrameProcessor &frameProcessor = runtime.FrameProcessor();
        if (frameProcessor.EmitDfx(*published->frame) == SlamFrameProcessor::StepResult::SessionAbort) {
            PushStatus(context, runtime.sessionOk, true);
        } else {
            PushStatus(context, runtime.sessionOk, false);
        }
    }

  private:
    void PushStatus(smartdrone::runtime_graph::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<NativeSlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push("status", std::move(status));
    }

    std::shared_ptr<std::mutex> m_processorMu;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(
    NativeSlamDfxTask, "NativeSlamDfxTask",
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("published", "NativeSlamPublishedFrame")},
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("status", "NativeSlamStatus")})

class NativeSlamUdpTask final : public smartdrone::runtime_graph::ITask {
  public:
    NativeSlamUdpTask(std::shared_ptr<std::mutex> processorMu, std::atomic<bool> &stop,
                      std::atomic<bool> &runningFlag)
        : m_processorMu(std::move(processorMu)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(smartdrone::runtime_graph::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto published = context.TryPopLatest<NativeSlamPublishedFrame>("published");
        if (!published || !published->runtime || !published->frame) {
            return;
        }

        auto &runtime = *published->runtime;
        {
            std::lock_guard<std::mutex> lock(*m_processorMu);
            SlamFrameProcessor &frameProcessor = runtime.FrameProcessor();
            if (frameProcessor.EmitUdp(*published->frame) == SlamFrameProcessor::StepResult::SessionAbort) {
                PushStatus(context, runtime.sessionOk, true);
                return;
            }
        }

        context.Push("published", published);
    }

  private:
    void PushStatus(smartdrone::runtime_graph::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<NativeSlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push("status", std::move(status));
    }

    std::shared_ptr<std::mutex> m_processorMu;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(
    NativeSlamUdpTask, "NativeSlamUdpTask",
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("published", "NativeSlamPublishedFrame")},
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("published", "NativeSlamPublishedFrame"),
        SMARTDRONE_RUNTIME_GRAPH_PORT("status", "NativeSlamStatus")})

class NativeSlamMavlinkTask final : public smartdrone::runtime_graph::ITask {
  public:
    NativeSlamMavlinkTask(std::shared_ptr<std::mutex> processorMu, std::atomic<bool> &stop,
                          std::atomic<bool> &runningFlag)
        : m_processorMu(std::move(processorMu)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(smartdrone::runtime_graph::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto published = context.TryPopLatest<NativeSlamPublishedFrame>("published");
        if (!published || !published->runtime || !published->frame) {
            return;
        }

        auto &runtime = *published->runtime;
        {
            std::lock_guard<std::mutex> lock(*m_processorMu);
            SlamFrameProcessor &frameProcessor = runtime.FrameProcessor();
            if (frameProcessor.EmitMavlink(*published->frame) == SlamFrameProcessor::StepResult::SessionAbort) {
                PushStatus(context, runtime.sessionOk, true);
                return;
            }
        }

        context.Push("published", published);
    }

  private:
    void PushStatus(smartdrone::runtime_graph::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<NativeSlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push("status", std::move(status));
    }

    std::shared_ptr<std::mutex> m_processorMu;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(
    NativeSlamMavlinkTask, "NativeSlamMavlinkTask",
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("published", "NativeSlamPublishedFrame")},
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("published", "NativeSlamPublishedFrame"),
        SMARTDRONE_RUNTIME_GRAPH_PORT("status", "NativeSlamStatus")})

class NativeSlamLivePoseTask final : public smartdrone::runtime_graph::ITask {
  public:
    NativeSlamLivePoseTask(std::shared_ptr<std::mutex> processorMu, std::atomic<bool> &stop,
                           std::atomic<bool> &runningFlag)
        : m_processorMu(std::move(processorMu)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(smartdrone::runtime_graph::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto published = context.TryPopLatest<NativeSlamPublishedFrame>("published");
        if (!published || !published->runtime || !published->frame) {
            return;
        }

        auto &runtime = *published->runtime;
        {
            std::lock_guard<std::mutex> lock(*m_processorMu);
            SlamFrameProcessor &frameProcessor = runtime.FrameProcessor();
            if (frameProcessor.EmitLivePose(*published->frame) == SlamFrameProcessor::StepResult::SessionAbort) {
                PushStatus(context, runtime.sessionOk, true);
                return;
            }
        }

        context.Push("published", published);
    }

  private:
    void PushStatus(smartdrone::runtime_graph::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<NativeSlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push("status", std::move(status));
    }

    std::shared_ptr<std::mutex> m_processorMu;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(
    NativeSlamLivePoseTask, "NativeSlamLivePoseTask",
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("published", "NativeSlamPublishedFrame")},
    std::vector<smartdrone::runtime_graph::PortSpec>{
        SMARTDRONE_RUNTIME_GRAPH_PORT("published", "NativeSlamPublishedFrame"),
        SMARTDRONE_RUNTIME_GRAPH_PORT("status", "NativeSlamStatus")})

class NativeSlamMonitorTask final : public smartdrone::runtime_graph::ITask {
  public:
    NativeSlamMonitorTask(std::atomic<bool> &stop, std::atomic<bool> &sessionOk) : m_stop(stop), m_sessionOk(sessionOk)
    {
    }

    void OnTick(smartdrone::runtime_graph::TaskContext &context) override
    {
        while (auto status = context.TryPop<NativeSlamStatus>("status")) {
            m_sessionOk.store(status->sessionOk, std::memory_order_relaxed);
            if (status->abortRequested) {
                m_stop.store(true);
            }
        }
    }

  private:
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_sessionOk;
};
SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(
    NativeSlamMonitorTask, "NativeSlamMonitorTask",
	    std::vector<smartdrone::runtime_graph::PortSpec>{
	        SMARTDRONE_RUNTIME_GRAPH_PORT("status", "NativeSlamStatus")},
	    std::vector<smartdrone::runtime_graph::PortSpec>{})

NativeRuntimeGraphTaskFactoryResolver MakeSlamGraphTaskFactoryResolver(
    const std::shared_ptr<NativeSlamRuntimeState> &runtimeState,
    const std::shared_ptr<std::mutex> &processorMu,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag,
    std::atomic<bool> &sessionOk)
{
    auto &catalog = smartdrone::runtime_graph::RuntimeGraphTypeCatalog::Global();
    return smartdrone::runtime_graph::RuntimeGraphTypeCatalog::MakeTaskFactoryResolver({
        catalog.MakeTaskFactoryEntry<NativeSlamResourceTask>([runtimeState, &stop, &runningFlag]() {
                    return std::unique_ptr<smartdrone::runtime_graph::ITask>(
                        new NativeSlamResourceTask(runtimeState, stop, runningFlag));
                }),
        catalog.MakeTaskFactoryEntry<NativeSlamClockTask>([&stop, &runningFlag]() {
                return std::unique_ptr<smartdrone::runtime_graph::ITask>(
                    new NativeSlamClockTask(stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<NativeSlamImuGateTask>([runtimeState, &stop, &runningFlag]() {
                    return std::unique_ptr<smartdrone::runtime_graph::ITask>(
                        new NativeSlamImuGateTask(runtimeState, stop, runningFlag));
                }),
        catalog.MakeTaskFactoryEntry<NativeSlamAcquireTask>([processorMu, &stop, &runningFlag]() {
                return std::unique_ptr<smartdrone::runtime_graph::ITask>(
                    new NativeSlamAcquireTask(processorMu, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<NativeSlamTrackingTask>([processorMu, &stop, &runningFlag]() {
                return std::unique_ptr<smartdrone::runtime_graph::ITask>(
                    new NativeSlamTrackingTask(processorMu, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<NativeSlamPosePostprocessTask>([processorMu, &stop, &runningFlag]() {
                return std::unique_ptr<smartdrone::runtime_graph::ITask>(
                    new NativeSlamPosePostprocessTask(processorMu, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<NativeSlamPointCloudTask>([processorMu, &stop, &runningFlag]() {
                return std::unique_ptr<smartdrone::runtime_graph::ITask>(
                    new NativeSlamPointCloudTask(processorMu, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<NativeSlamLivePoseTask>([processorMu, &stop, &runningFlag]() {
                return std::unique_ptr<smartdrone::runtime_graph::ITask>(
                    new NativeSlamLivePoseTask(processorMu, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<NativeSlamMavlinkTask>([processorMu, &stop, &runningFlag]() {
                return std::unique_ptr<smartdrone::runtime_graph::ITask>(
                    new NativeSlamMavlinkTask(processorMu, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<NativeSlamUdpTask>([processorMu, &stop, &runningFlag]() {
                return std::unique_ptr<smartdrone::runtime_graph::ITask>(
                    new NativeSlamUdpTask(processorMu, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<NativeSlamDfxTask>([processorMu, &stop, &runningFlag]() {
                return std::unique_ptr<smartdrone::runtime_graph::ITask>(
                    new NativeSlamDfxTask(processorMu, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<NativeSlamMonitorTask>([&stop, &sessionOk]() {
                return std::unique_ptr<smartdrone::runtime_graph::ITask>(
                    new NativeSlamMonitorTask(stop, sessionOk));
            }),
    });
}

} // namespace

bool RunSlamSessionGraph(const UnifiedConfig &cfg, LiveRuntimeTuning &tuning, Px4MavlinkGateway &mav,
                         std::atomic<bool> &stop, LivePoseState &livePose, std::atomic<bool> &runningFlag)
{
    std::atomic<bool> sessionOk{true};

    {
        smartdrone::runtime_graph::Registry registry;
        auto runtimeState =
            std::make_shared<NativeSlamRuntimeState>(cfg, tuning, mav, stop, livePose, runningFlag);
        auto processorMu = std::make_shared<std::mutex>();
        RegisterNativeRuntimeGraphTypes(
            registry, NativeRuntimeGraphDomain::SlamSession,
            MakeSlamGraphTaskFactoryResolver(runtimeState, processorMu, stop, runningFlag, sessionOk));

        smartdrone::runtime_graph::RuntimeGraph graph(registry);
        graph.Configure(CompileNativeRuntimeGraphConfig(NativeRuntimeGraphDomain::SlamSession, registry));
        graph.Start();

        while (runningFlag.load() && !stop.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        graph.Stop();
    }

    return sessionOk.load(std::memory_order_relaxed);
}

} // namespace smartdrone::core::application
