#pragma once

#include "common/epg/epg.h"
#include "common/epg/epg_trigger_modes.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <thread>

namespace Epg {

class EventPipelineGraph::TaskRunner {
  public:
    TaskRunner(TaskConfig config,
               std::unique_ptr<ITask> task,
               std::unordered_map<PortId, IQueue *> inputs,
               std::unordered_map<PortId, IQueue *> outputs,
               std::vector<IQueue *> triggerQueues);
    ~TaskRunner();

    const std::string &Name() const;
    void Start();
    void RequestStop();
    bool JoinStopped();
    void Stop();
    void Notify();
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
    void PollWakeEvent(int timeoutMs);
    void DrainWakeEvents();
    bool QueuesReady() const;
    bool BackpressureBlocked() const;
    void StoreLoopSample(std::uint64_t elapsedUs);
    void FillLoopPercentiles(TaskDiagnosticsSnapshot &snapshot) const;

    TaskConfig m_config;
    std::unique_ptr<ITask> m_task;
    TaskContext m_context;
    std::vector<IQueue *> m_triggerQueues;
    mutable ::Epg::TaskDiagnostics m_diag;
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_exited{true};
    std::thread m_thread;
    int m_wakeEventFd{-1};
    std::array<std::atomic<std::uint64_t>, LOOP_SAMPLE_CAPACITY> m_loopSamples{};
    std::size_t m_loopSampleCursor{};
    std::atomic<std::size_t> m_loopSampleCount{0};
};

} // namespace Epg
