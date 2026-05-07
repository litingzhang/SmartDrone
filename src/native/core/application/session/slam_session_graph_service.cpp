#include "core/application/session/slam_session_graph_service.h"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "common/epg/epg.h"
#include "core/application/session/epg_messages.h"
#include "core/application/session/epg_registry.h"
#include "core/application/session/runtime_session_common.h"
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

void ApplySlamRuntimePacing(epg::GraphConfig &config, const UnifiedConfig &cfg)
{
    OverrideTaskInterval(config, "SlamResourceTask", kSlamResourcePollInterval);
    OverrideTaskInterval(config, "SlamClockTask",
                         SlamInputInterval(cfg.app.runtime.slamInputFps, cfg.app.camera.fps));
}

std::uint64_t EpgDfxNowMs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

void WriteEpgDfxSnapshotFile(const std::string &path, const std::string &json)
{
    const std::string tmpPath = path + ".tmp";
    {
        std::ofstream output(tmpPath, std::ios::out | std::ios::trunc);
        if (!output) {
            return;
        }
        output << json;
    }
    (void)std::rename(tmpPath.c_str(), path.c_str());
}

class SlamRuntimeState final {
  public:
    SlamRuntimeState(const UnifiedConfig &cfg, LiveRuntimeTuning &tuning, Px4MavlinkGateway &mav,
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

class SlamResourceTask final : public epg::ITask {
  public:
    SlamResourceTask(std::shared_ptr<SlamRuntimeState> state, std::atomic<bool> &stop,
                           std::atomic<bool> &runningFlag)
        : m_state(std::move(state)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    ~SlamResourceTask() override { m_state->Stop(); }

    void OnTick(epg::TaskContext &context) override
    {
        if (m_readyEmitted || !m_runningFlag.load() || m_stop.load()) {
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

class SlamImuGateTask final : public epg::ITask {
  public:
    SlamImuGateTask(std::shared_ptr<SlamRuntimeState> state, std::atomic<bool> &stop,
                          std::atomic<bool> &runningFlag, LiveRuntimeTuning &tuning, int cameraFps)
        : m_state(std::move(state)), m_stop(stop), m_runningFlag(runningFlag), m_tuning(tuning), m_cameraFps(cameraFps)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (auto ready = context.TryPopLatest<SlamResourceReady>(0)) {
            m_resourceReady = ready->ready;
        }
        const auto tick = context.TryPopLatest<SlamTick>(1);
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
        if (!runtime->WaitForImuReady()) {
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
    bool m_resourceReady{false};
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

EpgTaskFactoryResolver MakeSlamGraphTaskFactoryResolver(
    const std::shared_ptr<SlamRuntimeState> &runtimeState,
    const std::shared_ptr<std::mutex> &processorMu,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag,
    std::atomic<bool> &sessionOk,
    LiveRuntimeTuning &tuning,
    int cameraFps)
{
    auto &catalog = epg::TypeCatalog::Global();
    return epg::TypeCatalog::MakeTaskFactoryResolver({
        catalog.MakeTaskFactoryEntry<SlamResourceTask>([runtimeState, &stop, &runningFlag]() {
                    return std::unique_ptr<epg::ITask>(
                        new SlamResourceTask(runtimeState, stop, runningFlag));
                }),
        catalog.MakeTaskFactoryEntry<SlamClockTask>([&stop, &runningFlag]() {
                return std::unique_ptr<epg::ITask>(
                    new SlamClockTask(stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<SlamImuGateTask>([runtimeState, &stop, &runningFlag, &tuning, cameraFps]() {
                    return std::unique_ptr<epg::ITask>(
                        new SlamImuGateTask(runtimeState, stop, runningFlag, tuning, cameraFps));
                }),
        catalog.MakeTaskFactoryEntry<SlamAcquireTask>([processorMu, &stop, &runningFlag]() {
                return std::unique_ptr<epg::ITask>(
                    new SlamAcquireTask(processorMu, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<SlamTrackingTask>([processorMu, &stop, &runningFlag]() {
                return std::unique_ptr<epg::ITask>(
                    new SlamTrackingTask(processorMu, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<SlamPosePostprocessTask>([processorMu, &stop, &runningFlag]() {
                return std::unique_ptr<epg::ITask>(
                    new SlamPosePostprocessTask(processorMu, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<SlamPointCloudTask>([processorMu, &stop, &runningFlag]() {
                return std::unique_ptr<epg::ITask>(
                    new SlamPointCloudTask(processorMu, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<SlamLivePoseTask>([&stop, &runningFlag]() {
                return std::unique_ptr<epg::ITask>(
                    new SlamLivePoseTask(stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<SlamMavlinkTask>([&stop, &runningFlag]() {
                return std::unique_ptr<epg::ITask>(
                    new SlamMavlinkTask(stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<SlamUdpTask>([&stop, &runningFlag]() {
                return std::unique_ptr<epg::ITask>(
                    new SlamUdpTask(stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<SlamDfxTask>([processorMu, &stop, &runningFlag]() {
                return std::unique_ptr<epg::ITask>(
                    new SlamDfxTask(processorMu, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<SlamMonitorTask>([&stop, &sessionOk]() {
                return std::unique_ptr<epg::ITask>(
                    new SlamMonitorTask(stop, sessionOk));
            }),
    });
}

} // namespace

bool RunSlamSessionGraph(const UnifiedConfig &cfg, LiveRuntimeTuning &tuning, Px4MavlinkGateway &mav,
                         std::atomic<bool> &stop, LivePoseState &livePose, std::atomic<bool> &runningFlag)
{
    std::atomic<bool> sessionOk{true};

    {
        epg::Registry registry;
        auto runtimeState =
            std::make_shared<SlamRuntimeState>(cfg, tuning, mav, stop, livePose, runningFlag);
        auto processorMu = std::make_shared<std::mutex>();
        RegisterEpgTypes(
            registry, EpgDomain::SlamSession,
            MakeSlamGraphTaskFactoryResolver(runtimeState, processorMu, stop, runningFlag, sessionOk,
                                             tuning, cfg.app.camera.fps));

        epg::EventPipelineGraph graph(registry);
        auto graphConfig = CompileEpgConfig(EpgDomain::SlamSession, registry);
        ApplySlamRuntimePacing(graphConfig, cfg);
        graph.Configure(graphConfig);
        graph.Start();

        auto nextDfxSnapshot = std::chrono::steady_clock::now();
        while (runningFlag.load() && !stop.load()) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= nextDfxSnapshot) {
                WriteEpgDfxSnapshotFile(
                    kSlamEpgDfxSnapshotPath,
                    graph.DfxSnapshotJson("cluster_slam_session_graph", EpgDfxNowMs()));
                nextDfxSnapshot = now + std::chrono::milliseconds(500);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        WriteEpgDfxSnapshotFile(
            kSlamEpgDfxSnapshotPath,
            graph.DfxSnapshotJson("cluster_slam_session_graph", EpgDfxNowMs()));
        graph.Stop();
    }

    return sessionOk.load(std::memory_order_relaxed);
}

} // namespace smartdrone::core::application
