#include "core/application/session/slam_session_graph_service.h"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "common/epg/epg.h"
#include "core/application/runtime/epg_dfx_snapshot.h"
#include "core/application/runtime/runtime_aliases.h"
#include "core/application/session/epg_messages.h"
#include "core/application/session/epg_registry.h"
#include "core/application/runtime/epg_graph_lifecycle.h"
#include "core/application/session/slam_frame_processor.h"
#include "core/application/session/slam_session_runtime.h"

namespace smartdrone::core::application {
namespace {

constexpr const char *kSlamEpgDfxSnapshotPath = "/tmp/smartdrone_epg_slam.json";
constexpr std::chrono::milliseconds kSlamResourcePollInterval{100};

std::chrono::milliseconds SlamInputInterval(int slamInputFps, int cameraFps)
{
    const int clampedFps = ClampSlamInputFps(slamInputFps, cameraFps);
    const int intervalMs = std::max(1, (1000 + clampedFps - 1) / clampedFps);
    return std::chrono::milliseconds(intervalMs);
}

void OverrideTaskInterval(epg::GraphConfig &config, const std::string &taskName,
                          std::chrono::milliseconds interval)
{
    for (auto &task : config.tasks) {
        if (task.name == taskName) {
            task.trigger.interval = interval;
            return;
        }
    }
}

void OverrideTaskScheduling(epg::GraphConfig &config, const std::string &taskName,
                            bool realtime, int priority)
{
    for (auto &task : config.tasks) {
        if (task.name == taskName) {
            task.scheduling.realtime = realtime;
            task.scheduling.priority = priority;
            return;
        }
    }
}

void ApplySlamRuntimePacing(epg::GraphConfig &config, const UnifiedConfig &cfg)
{
    OverrideTaskInterval(config, "SlamResourceTask", kSlamResourcePollInterval);
    OverrideTaskInterval(config, "SlamClockTask",
                         SlamInputInterval(cfg.app.runtime.slamInputFps, cfg.app.camera.fps));
    OverrideTaskScheduling(config, "SlamImuPollTask", cfg.app.imu.rtImu, cfg.app.imu.rtPrio);
}

class SlamRuntimeState final {
  public:
    explicit SlamRuntimeState(SlamSessionGraphRuntimeConfig config)
        : m_cfg(std::move(config.cfg)),
          m_tuning(config.tuning),
          m_telemetry(config.telemetry),
          m_posePublisher(config.posePublisher),
          m_stop(config.stop),
          m_livePose(config.livePose),
          m_runningFlag(config.runningFlag)
    {
    }

    ~SlamRuntimeState() { Stop(); }

    bool EnsureStarted()
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

    bool Stopped() const { return m_stopped.load(std::memory_order_acquire); }

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
        m_stopped.store(true, std::memory_order_release);
    }

  private:
    UnifiedConfig m_cfg;
    LiveRuntimeTuning &m_tuning;
    smartdrone::core::ports::ISlamSessionTelemetryPort &m_telemetry;
    smartdrone::core::ports::IPosePublisher &m_posePublisher;
    std::atomic<bool> &m_stop;
    LivePoseState &m_livePose;
    std::atomic<bool> &m_runningFlag;
    mutable std::mutex m_mu;
    std::shared_ptr<SlamSessionRuntime> m_runtime;
    std::atomic<bool> m_stopped{true};
    bool m_started{false};
    bool m_startFailed{false};
};

class SlamResourceTask final : public epg::ITask {
  public:
    SlamResourceTask(std::shared_ptr<SlamRuntimeState> state, std::atomic<bool> &stop,
                           std::atomic<bool> &runningFlag)
        : m_state(std::move(state)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            m_state->Stop();
            return;
        }
        if (m_readyEmitted) {
            return;
        }
        if (!m_state->EnsureStarted()) {
            m_stop.store(true);
            return;
        }
        auto ready = context.Make<SlamResourceReady>();
        ready->ready = true;
        if (context.Push(0, std::move(ready))) {
            m_readyEmitted = true;
        }
    }

  private:
    std::shared_ptr<SlamRuntimeState> m_state;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    bool m_readyEmitted{false};
};
EPG_REGISTER_TASK_TYPE(SlamResourceTask, "SlamResourceTask")

class SlamClockTask final : public epg::ITask {
  public:
    SlamClockTask(std::atomic<bool> &stop, std::atomic<bool> &runningFlag)
        : m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        auto tick = context.Make<SlamTick>();
        tick->sequence = ++m_sequence;
        context.Push(0, std::move(tick));
    }

  private:
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    std::uint64_t m_sequence{0};
};
EPG_REGISTER_TASK_TYPE(SlamClockTask, "SlamClockTask")

class SlamImuPollTask final : public epg::ITask {
  public:
    SlamImuPollTask(std::shared_ptr<SlamRuntimeState> state, std::atomic<bool> &stop,
                    std::atomic<bool> &runningFlag)
        : m_state(std::move(state)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        (void)context;
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        auto runtime = m_state->Runtime();
        if (!runtime) {
            return;
        }
        if (!runtime->StepImuPoll()) {
            m_stop.store(true);
            return;
        }
        if (!runtime->ImuReady() || context.OutputSize(0) > 0) {
            return;
        }

        auto ready = context.Make<SlamImuReady>();
        ready->ready = true;
        context.Push(0, std::move(ready));
    }

  private:
    std::shared_ptr<SlamRuntimeState> m_state;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
EPG_REGISTER_TASK_TYPE(SlamImuPollTask, "SlamImuPollTask")

class SlamBackendTickTask final : public epg::ITask {
  public:
    SlamBackendTickTask(std::shared_ptr<SlamRuntimeState> state, std::shared_ptr<std::mutex> processorMu,
                        std::atomic<bool> &stop, std::atomic<bool> &runningFlag)
        : m_state(std::move(state)), m_processorMu(std::move(processorMu)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        (void)context;
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        auto runtime = m_state->Runtime();
        if (!runtime) {
            return;
        }
        std::lock_guard<std::mutex> lock(*m_processorMu);
        if (runtime->FrameProcessor().StepBackend() == SlamFrameProcessor::StepResult::SessionAbort) {
            m_stop.store(true);
        }
    }

  private:
    std::shared_ptr<SlamRuntimeState> m_state;
    std::shared_ptr<std::mutex> m_processorMu;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
EPG_REGISTER_TASK_TYPE(SlamBackendTickTask, "SlamBackendTickTask")

class SlamImuGateTask final : public epg::ITask {
  public:
    SlamImuGateTask(std::shared_ptr<SlamRuntimeState> state, std::atomic<bool> &stop,
                          std::atomic<bool> &runningFlag, LiveRuntimeTuning &tuning, int cameraFps)
        : m_state(std::move(state)), m_stop(stop), m_runningFlag(runningFlag), m_tuning(tuning), m_cameraFps(cameraFps)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (auto ready = context.TryPopLatest<SlamImuReady>(0)) {
            m_imuReady = ready->ready;
        }
        const auto tick = context.TryPopLatest<SlamTick>(1);
        if (m_state->StartFailed()) {
            PushStatus(context, false, true);
            return;
        }
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        if (!m_imuReady) {
            return;
        }
        if (!tick) {
            return;
        }
        if (context.OutputSize(0) > 0) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        const auto minInterval =
            SlamInputInterval(m_tuning.slamInputFps.load(std::memory_order_relaxed), m_cameraFps);
        if (m_lastFrameReadyTime.time_since_epoch().count() != 0 &&
            now - m_lastFrameReadyTime < minInterval) {
            return;
        }

        auto runtime = m_state->Runtime();
        if (!runtime) {
            return;
        }
        auto frameReady = context.Make<SlamFrameReady>();
        frameReady->runtime = std::move(runtime);
        if (context.Push(0, std::move(frameReady))) {
            m_lastFrameReadyTime = now;
        }
    }

  private:
    void PushStatus(epg::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<SlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push(1, std::move(status));
    }

    std::shared_ptr<SlamRuntimeState> m_state;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    LiveRuntimeTuning &m_tuning;
    int m_cameraFps{};
    std::chrono::steady_clock::time_point m_lastFrameReadyTime{};
    bool m_imuReady{false};
};
EPG_REGISTER_TASK_TYPE(SlamImuGateTask, "SlamImuGateTask")

class SlamAcquireTask final : public epg::ITask {
  public:
    SlamAcquireTask(std::shared_ptr<std::mutex> processorMu, std::atomic<bool> &stop,
                          std::atomic<bool> &runningFlag)
        : m_processorMu(std::move(processorMu)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto frameReady = context.TryPopLatest<SlamFrameReady>(0);
        if (!frameReady || !frameReady->runtime) {
            return;
        }
        if (context.OutputSize(0) > 0) {
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

        auto prepared = context.Make<SlamPreparedFrame>();
        prepared->runtime = frameReady->runtime;
        prepared->frame = std::move(preparedFrame);
        context.Push(0, std::move(prepared));
    }

  private:
    void PushStatus(epg::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<SlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push(1, std::move(status));
    }

    std::shared_ptr<std::mutex> m_processorMu;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
EPG_REGISTER_TASK_TYPE(SlamAcquireTask, "SlamAcquireTask")

class SlamTrackingTask final : public epg::ITask {
  public:
    SlamTrackingTask(std::shared_ptr<std::mutex> processorMu, std::atomic<bool> &stop,
                           std::atomic<bool> &runningFlag)
        : m_processorMu(std::move(processorMu)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto prepared = context.TryPopLatest<SlamPreparedFrame>(0);
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

        auto tracked = context.Make<SlamTrackedFrame>();
        tracked->runtime = prepared->runtime;
        tracked->frame = std::move(trackedFrame);
        context.Push(0, std::move(tracked));
    }

  private:
    void PushStatus(epg::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<SlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push(1, std::move(status));
    }

    std::shared_ptr<std::mutex> m_processorMu;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
EPG_REGISTER_TASK_TYPE(SlamTrackingTask, "SlamTrackingTask")

class SlamPosePostprocessTask final : public epg::ITask {
  public:
    SlamPosePostprocessTask(std::shared_ptr<std::mutex> processorMu, std::atomic<bool> &stop,
                          std::atomic<bool> &runningFlag)
        : m_processorMu(std::move(processorMu)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto tracked = context.TryPopLatest<SlamTrackedFrame>(0);
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

        auto published = context.Make<SlamPublishedFrame>();
        published->runtime = tracked->runtime;
        published->frame = std::move(publishedFrame);
        context.Push(0, std::move(published));
    }

  private:
    void PushStatus(epg::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<SlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push(1, std::move(status));
    }

    std::shared_ptr<std::mutex> m_processorMu;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
EPG_REGISTER_TASK_TYPE(SlamPosePostprocessTask, "SlamPosePostprocessTask")

class SlamPointCloudTask final : public epg::ITask {
  public:
    SlamPointCloudTask(std::shared_ptr<std::mutex> processorMu, std::atomic<bool> &stop,
                             std::atomic<bool> &runningFlag)
        : m_processorMu(std::move(processorMu)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto published = context.TryPopLatest<SlamPublishedFrame>(0);
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

        context.Push(0, published);
    }

  private:
    void PushStatus(epg::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<SlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push(1, std::move(status));
    }

    std::shared_ptr<std::mutex> m_processorMu;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
EPG_REGISTER_TASK_TYPE(SlamPointCloudTask, "SlamPointCloudTask")

class SlamDfxTask final : public epg::ITask {
  public:
    SlamDfxTask(std::shared_ptr<std::mutex> processorMu, std::atomic<bool> &stop,
                      std::atomic<bool> &runningFlag)
        : m_processorMu(std::move(processorMu)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto published = context.TryPopLatest<SlamPublishedFrame>(0);
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
    void PushStatus(epg::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<SlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push(1, std::move(status));
    }

    std::shared_ptr<std::mutex> m_processorMu;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
EPG_REGISTER_TASK_TYPE(SlamDfxTask, "SlamDfxTask")

class SlamUdpTask final : public epg::ITask {
  public:
    SlamUdpTask(std::atomic<bool> &stop, std::atomic<bool> &runningFlag)
        : m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto published = context.TryPopLatest<SlamPublishedFrame>(0);
        if (!published || !published->runtime || !published->frame) {
            return;
        }

        auto &runtime = *published->runtime;
        SlamFrameProcessor &frameProcessor = runtime.FrameProcessor();
        if (frameProcessor.EmitUdp(*published->frame) == SlamFrameProcessor::StepResult::SessionAbort) {
            PushStatus(context, runtime.sessionOk, true);
            return;
        }

        context.Push(0, published);
    }

  private:
    void PushStatus(epg::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<SlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push(1, std::move(status));
    }

    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
EPG_REGISTER_TASK_TYPE(SlamUdpTask, "SlamUdpTask")

class SlamMavlinkTask final : public epg::ITask {
  public:
    SlamMavlinkTask(std::atomic<bool> &stop, std::atomic<bool> &runningFlag)
        : m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto published = context.TryPopLatest<SlamPublishedFrame>(0);
        if (!published || !published->runtime || !published->frame) {
            return;
        }

        auto &runtime = *published->runtime;
        SlamFrameProcessor &frameProcessor = runtime.FrameProcessor();
        if (frameProcessor.EmitMavlink(*published->frame) == SlamFrameProcessor::StepResult::SessionAbort) {
            PushStatus(context, runtime.sessionOk, true);
            return;
        }

        context.Push(0, published);
    }

  private:
    void PushStatus(epg::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<SlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push(1, std::move(status));
    }

    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
EPG_REGISTER_TASK_TYPE(SlamMavlinkTask, "SlamMavlinkTask")

class SlamLivePoseTask final : public epg::ITask {
  public:
    SlamLivePoseTask(std::atomic<bool> &stop, std::atomic<bool> &runningFlag)
        : m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        const auto published = context.TryPopLatest<SlamPublishedFrame>(0);
        if (!published || !published->runtime || !published->frame) {
            return;
        }

        auto &runtime = *published->runtime;
        SlamFrameProcessor &frameProcessor = runtime.FrameProcessor();
        if (frameProcessor.EmitLivePose(*published->frame) == SlamFrameProcessor::StepResult::SessionAbort) {
            PushStatus(context, runtime.sessionOk, true);
            return;
        }

        context.Push(0, published);
    }

  private:
    void PushStatus(epg::TaskContext &context, bool sessionOk, bool abortRequested)
    {
        auto status = context.Make<SlamStatus>();
        status->sessionOk = sessionOk;
        status->abortRequested = abortRequested;
        context.Push(1, std::move(status));
    }

    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};
EPG_REGISTER_TASK_TYPE(SlamLivePoseTask, "SlamLivePoseTask")

class SlamMonitorTask final : public epg::ITask {
  public:
    SlamMonitorTask(std::atomic<bool> &stop, std::atomic<bool> &sessionOk) : m_stop(stop), m_sessionOk(sessionOk)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        while (auto status = context.TryPop<SlamStatus>(0)) {
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
EPG_REGISTER_TASK_TYPE(SlamMonitorTask, "SlamMonitorTask")

struct SlamTaskFactoryDeps {
    std::shared_ptr<SlamRuntimeState> runtimeState;
    std::shared_ptr<std::mutex> processorMu;
    std::atomic<bool> &stop;
    std::atomic<bool> &runningFlag;
    std::atomic<bool> &sessionOk;
    LiveRuntimeTuning &tuning;
    int cameraFps;
    std::shared_ptr<EpgGraphRef> graphRef;
};

using EpgTaskFactoryEntries = std::vector<epg::TypeCatalog::TaskFactoryEntry>;

void AddSlamStartupTaskFactories(EpgTaskFactoryEntries &entries,
                                 const epg::TypeCatalog &catalog,
                                 SlamTaskFactoryDeps deps)
{
    entries.push_back(
        catalog.MakeTaskFactoryEntry<SlamResourceTask>([deps]() {
            return std::unique_ptr<epg::ITask>(
                new SlamResourceTask(deps.runtimeState, deps.stop, deps.runningFlag));
        }));
    entries.push_back(
        catalog.MakeTaskFactoryEntry<SlamClockTask>([deps]() {
            return std::unique_ptr<epg::ITask>(
                new SlamClockTask(deps.stop, deps.runningFlag));
        }));
}

void AddSlamProcessingTaskFactories(EpgTaskFactoryEntries &entries,
                                    const epg::TypeCatalog &catalog,
                                    SlamTaskFactoryDeps deps)
{
    entries.push_back(
        catalog.MakeTaskFactoryEntry<SlamImuPollTask>([deps]() {
            return std::unique_ptr<epg::ITask>(
                new SlamImuPollTask(deps.runtimeState, deps.stop, deps.runningFlag));
        }));
    entries.push_back(
        catalog.MakeTaskFactoryEntry<SlamBackendTickTask>([deps]() {
            return std::unique_ptr<epg::ITask>(
                new SlamBackendTickTask(deps.runtimeState, deps.processorMu,
                                        deps.stop, deps.runningFlag));
        }));
    entries.push_back(
        catalog.MakeTaskFactoryEntry<SlamImuGateTask>([deps]() {
            return std::unique_ptr<epg::ITask>(
                new SlamImuGateTask(deps.runtimeState, deps.stop, deps.runningFlag,
                                    deps.tuning, deps.cameraFps));
        }));
    entries.push_back(
        catalog.MakeTaskFactoryEntry<SlamAcquireTask>([deps]() {
            return std::unique_ptr<epg::ITask>(
                new SlamAcquireTask(deps.processorMu, deps.stop, deps.runningFlag));
        }));
    entries.push_back(
        catalog.MakeTaskFactoryEntry<SlamTrackingTask>([deps]() {
            return std::unique_ptr<epg::ITask>(
                new SlamTrackingTask(deps.processorMu, deps.stop, deps.runningFlag));
        }));
}

void AddSlamOutputTaskFactories(EpgTaskFactoryEntries &entries,
                                const epg::TypeCatalog &catalog,
                                SlamTaskFactoryDeps deps)
{
    entries.push_back(
        catalog.MakeTaskFactoryEntry<SlamPosePostprocessTask>([deps]() {
            return std::unique_ptr<epg::ITask>(
                new SlamPosePostprocessTask(deps.processorMu, deps.stop,
                                            deps.runningFlag));
        }));
    entries.push_back(
        catalog.MakeTaskFactoryEntry<SlamPointCloudTask>([deps]() {
            return std::unique_ptr<epg::ITask>(
                new SlamPointCloudTask(deps.processorMu, deps.stop, deps.runningFlag));
        }));
    entries.push_back(
        catalog.MakeTaskFactoryEntry<SlamLivePoseTask>([deps]() {
            return std::unique_ptr<epg::ITask>(
                new SlamLivePoseTask(deps.stop, deps.runningFlag));
        }));
    entries.push_back(
        catalog.MakeTaskFactoryEntry<SlamMavlinkTask>([deps]() {
            return std::unique_ptr<epg::ITask>(
                new SlamMavlinkTask(deps.stop, deps.runningFlag));
        }));
    entries.push_back(
        catalog.MakeTaskFactoryEntry<SlamUdpTask>([deps]() {
            return std::unique_ptr<epg::ITask>(
                new SlamUdpTask(deps.stop, deps.runningFlag));
        }));
    entries.push_back(
        catalog.MakeTaskFactoryEntry<SlamDfxTask>([deps]() {
            return std::unique_ptr<epg::ITask>(
                new SlamDfxTask(deps.processorMu, deps.stop, deps.runningFlag));
        }));
}

void AddSlamMonitorTaskFactories(EpgTaskFactoryEntries &entries,
                                 const epg::TypeCatalog &catalog,
                                 SlamTaskFactoryDeps deps)
{
    entries.push_back(
        catalog.MakeTaskFactoryEntry<SlamMonitorTask>([deps]() {
            return std::unique_ptr<epg::ITask>(
                new SlamMonitorTask(deps.stop, deps.sessionOk));
        }));
    entries.push_back(
        catalog.MakeTaskFactoryEntry<EpgDfxSnapshotTask>(
            [deps]() {
                return std::unique_ptr<epg::ITask>(new EpgDfxSnapshotTask({
                    deps.graphRef,
                    "cluster_slam_session_graph",
                    kSlamEpgDfxSnapshotPath,
                }));
            }));
}

EpgTaskFactoryResolver MakeSlamGraphTaskFactoryResolver(SlamTaskFactoryDeps deps)
{
    auto &catalog = epg::TypeCatalog::Global();
    EpgTaskFactoryEntries entries;
    entries.reserve(15);
    AddSlamStartupTaskFactories(entries, catalog, deps);
    AddSlamProcessingTaskFactories(entries, catalog, deps);
    AddSlamOutputTaskFactories(entries, catalog, deps);
    AddSlamMonitorTaskFactories(entries, catalog, deps);
    return epg::TypeCatalog::MakeTaskFactoryResolver(std::move(entries));
}

} // namespace

class SlamSessionGraphRuntime::Impl final {
  public:
    explicit Impl(SlamSessionGraphRuntimeConfig config)
        : m_cfg(std::move(config.cfg)),
          m_tuning(config.tuning),
          m_telemetry(config.telemetry),
          m_posePublisher(config.posePublisher),
          m_stop(config.stop),
          m_livePose(config.livePose),
          m_runningFlag(config.runningFlag),
          m_lifecycle(EpgGraphLifecycleConfig{
              m_stop,
              [this]() { return ResourcesStopped(); },
              [this]() { StopResources(); },
              [this]() { ResetResources(); },
          })
    {
    }

    ~Impl() { Stop(); }

    bool Start()
    {
        if (m_lifecycle.HasGraph()) {
            return true;
        }
        m_lifecycle.ResetForStart();
        m_sessionOk.store(true, std::memory_order_relaxed);
        m_runtimeState = std::make_shared<SlamRuntimeState>(RuntimeConfig());
        m_processorMu = std::make_shared<std::mutex>();
        auto graphRef = std::make_shared<EpgGraphRef>();
        RegisterEpgTypes(m_registry, EpgDomain::SlamSession,
                         MakeSlamGraphTaskFactoryResolver({
                             m_runtimeState,
                             m_processorMu,
                             m_stop,
                             m_runningFlag,
                             m_sessionOk,
                             m_tuning,
                             m_cfg.app.camera.fps,
                             graphRef,
                         }));
        auto graph = std::make_unique<epg::EventPipelineGraph>(m_registry);
        graphRef->graph = graph.get();
        auto graphConfig = CompileEpgConfig(EpgDomain::SlamSession, m_registry);
        ApplySlamRuntimePacing(graphConfig, m_cfg);
        graph->Configure(graphConfig);
        graph->Start();
        m_lifecycle.AttachGraph(std::move(graph));
        return true;
    }

    void Step()
    {
        if (m_lifecycle.Done()) {
            return;
        }
        if (m_lifecycle.StopRequested()) {
            m_lifecycle.StepStop();
            return;
        }
        if (!m_lifecycle.HasGraph()) {
            return;
        }
        if (!m_runningFlag.load() || m_stop.load()) {
            RequestStop();
        }
    }

    void RequestStop()
    {
        m_lifecycle.RequestStop();
    }

    void Stop()
    {
        if (m_lifecycle.Done()) {
            return;
        }
        if (m_lifecycle.HasGraph()) {
            m_lifecycle.StopSynchronously();
        }
    }

    bool Done()
    {
        m_lifecycle.StepStop();
        return m_lifecycle.Done();
    }
    bool Ok() const { return m_sessionOk.load(std::memory_order_relaxed); }

  private:
    SlamSessionGraphRuntimeConfig RuntimeConfig()
    {
        return {
            m_cfg,
            m_tuning,
            m_telemetry,
            m_posePublisher,
            m_stop,
            m_livePose,
            m_runningFlag,
        };
    }

    bool ResourcesStopped() const
    {
        return m_runtimeState && m_runtimeState->Stopped();
    }

    void StopResources()
    {
        if (m_runtimeState) {
            m_runtimeState->Stop();
        }
    }

    void ResetResources()
    {
        m_runtimeState.reset();
        m_processorMu.reset();
    }

    UnifiedConfig m_cfg;
    LiveRuntimeTuning &m_tuning;
    smartdrone::core::ports::ISlamSessionTelemetryPort &m_telemetry;
    smartdrone::core::ports::IPosePublisher &m_posePublisher;
    std::atomic<bool> &m_stop;
    LivePoseState &m_livePose;
    std::atomic<bool> &m_runningFlag;
    epg::Registry m_registry;
    std::shared_ptr<SlamRuntimeState> m_runtimeState;
    std::shared_ptr<std::mutex> m_processorMu;
    EpgGraphLifecycle m_lifecycle;
    std::atomic<bool> m_sessionOk{true};
};

SlamSessionGraphRuntime::SlamSessionGraphRuntime(SlamSessionGraphRuntimeConfig config)
    : m_impl(new Impl(std::move(config)))
{
}

SlamSessionGraphRuntime::~SlamSessionGraphRuntime() = default;

bool SlamSessionGraphRuntime::Start() { return m_impl->Start(); }

void SlamSessionGraphRuntime::Step() { m_impl->Step(); }

void SlamSessionGraphRuntime::RequestStop() { m_impl->RequestStop(); }

void SlamSessionGraphRuntime::Stop() { m_impl->Stop(); }

bool SlamSessionGraphRuntime::Done() { return m_impl->Done(); }

bool SlamSessionGraphRuntime::Ok() const { return m_impl->Ok(); }

} // namespace smartdrone::core::application
