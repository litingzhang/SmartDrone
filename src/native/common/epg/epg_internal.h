#pragma once

#include "common/epg/epg.h"
#include "common/epg/epg_trigger_modes.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

namespace Epg {

struct TaskResourceGate {
    std::atomic<bool> busy{false};
};

struct TaskRunnerSpec {
    TaskConfig config;
    std::unique_ptr<ITask> task;
    std::unordered_map<PortId, IQueue *> inputs;
    std::unordered_map<PortId, IQueue *> outputs;
    std::vector<IQueue *> triggerQueues;
    std::shared_ptr<TaskResourceGate> resourceGate;
};

class TaskWakeSignal final {
  public:
    TaskWakeSignal();
    ~TaskWakeSignal();

    int Notify() const;
    void TriggerExternal();
    bool ConsumeExternalTrigger();
    bool ExternalTriggerPending() const;
    int EventFd() const;

  private:
    std::atomic<bool> m_externalTriggerPending{false};
    int m_eventFd{-1};
};

class EventPipelineGraph::TaskRunner {
  public:
    explicit TaskRunner(TaskRunnerSpec spec);
    ~TaskRunner();

    const std::string &Name() const;
    void Start();
    void RequestStop();
    bool JoinStopped();
    void Stop();
    void Notify();
    std::weak_ptr<TaskWakeSignal> ExternalTriggerSignal() const;
    TaskDiagnosticsSnapshot Diagnostics() const;

  private:
    static constexpr std::size_t LOOP_SAMPLE_CAPACITY = 64;

    void Run();
    void ApplyScheduling();
    bool WaitForTrigger();
    bool WaitForConfiguredPhase();
    bool WaitForDuration(std::chrono::milliseconds duration);
    bool WaitForQueueTrigger();
    bool WaitForPeriodicOrQueueTrigger();
    bool WaitForPeriodicOrExternalTrigger();
    void PollWakeEvent(int timeoutMs);
    void DrainWakeEvents();
    bool QueuesReady() const;
    bool BackpressureBlocked() const;
    bool WaitForResourceGate();
    void ReleaseResourceGate();
    void StoreLoopSample(std::uint64_t elapsedUs);
    void FillLoopPercentiles(TaskDiagnosticsSnapshot &snapshot) const;

    TaskConfig m_config;
    std::unique_ptr<ITask> m_task;
    TaskContext m_context;
    std::vector<IQueue *> m_triggerQueues;
    std::shared_ptr<TaskResourceGate> m_resourceGate;
    mutable ::Epg::TaskDiagnostics m_diag;
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_exited{true};
    std::thread m_thread;
    std::shared_ptr<TaskWakeSignal> m_wakeSignal;
    std::array<std::atomic<std::uint64_t>, LOOP_SAMPLE_CAPACITY> m_loopSamples{};
    std::size_t m_loopSampleCursor{};
    std::atomic<std::size_t> m_loopSampleCount{0};
};

} // namespace Epg
