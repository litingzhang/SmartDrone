#pragma once

#include "common/epg/epg.h"

#include <algorithm>
#include <array>
#include <condition_variable>
#include <thread>

namespace Epg {

inline bool IsQueueTriggeredMode(TriggerMode mode)
{
    return mode == TriggerMode::AnyQueueReady ||
           mode == TriggerMode::AllQueueReady ||
           mode == TriggerMode::PeriodicOrAnyQueueReady;
}

inline TaskDiagnosticsSnapshot SnapshotTaskDiagnostics(const TaskDiagnostics &diag)
{
    TaskDiagnosticsSnapshot result;
    result.loopCount = diag.loopCount.load(std::memory_order_relaxed);
    result.errorCount = diag.errorCount.load(std::memory_order_relaxed);
    result.idleWakeups = diag.idleWakeups.load(std::memory_order_relaxed);
    result.lastLoopUs = diag.lastLoopUs.load(std::memory_order_relaxed);
    result.maxLoopUs = diag.maxLoopUs.load(std::memory_order_relaxed);
    result.totalLoopUs = diag.totalLoopUs.load(std::memory_order_relaxed);
    result.resourceWaitCount =
        diag.resourceWaitCount.load(std::memory_order_relaxed);
    result.lastResourceWaitUs =
        diag.lastResourceWaitUs.load(std::memory_order_relaxed);
    result.maxResourceWaitUs =
        diag.maxResourceWaitUs.load(std::memory_order_relaxed);
    result.totalResourceWaitUs =
        diag.totalResourceWaitUs.load(std::memory_order_relaxed);
    result.firstLoopMs = diag.firstLoopMs.load(std::memory_order_relaxed);
    result.lastLoopMs = diag.lastLoopMs.load(std::memory_order_relaxed);
    result.budgetOverrunCount =
        diag.budgetOverrunCount.load(std::memory_order_relaxed);
    result.deadlineMissCount =
        diag.deadlineMissCount.load(std::memory_order_relaxed);
    result.schedulingErrorCount =
        diag.schedulingErrorCount.load(std::memory_order_relaxed);
    result.lastSchedulingError =
        diag.lastSchedulingError.load(std::memory_order_relaxed);
    return result;
}

inline std::map<PortId, PortSpec> MakePortMap(const std::vector<PortSpec> &specs)
{
    std::map<PortId, PortSpec> result;
    for (const auto &spec : specs) {
        result.emplace(spec.id, spec);
    }
    return result;
}

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
    std::mutex m_mutex;
    std::condition_variable m_cv;
    mutable std::mutex m_sampleMutex;
    std::array<std::uint64_t, LOOP_SAMPLE_CAPACITY> m_loopSamples{};
    std::size_t m_loopSampleCursor{};
    std::size_t m_loopSampleCount{};
};

} // namespace Epg
