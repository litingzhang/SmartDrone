#pragma once

#include "common/epg/epg.h"

#include <algorithm>

namespace epg {

inline bool IsQueueTriggeredMode(TriggerMode mode) {
    return mode == TriggerMode::AnyQueueReady ||
           mode == TriggerMode::AllQueueReady ||
           mode == TriggerMode::PeriodicOrAnyQueueReady;
}

inline TaskDiagnosticsSnapshot SnapshotTaskDiagnostics(const TaskDiagnostics& diag) {
    TaskDiagnosticsSnapshot result;
    result.loopCount = diag.loopCount.load(std::memory_order_relaxed);
    result.errorCount = diag.errorCount.load(std::memory_order_relaxed);
    result.idleWakeups = diag.idleWakeups.load(std::memory_order_relaxed);
    result.lastLoopUs = diag.lastLoopUs.load(std::memory_order_relaxed);
    result.maxLoopUs = diag.maxLoopUs.load(std::memory_order_relaxed);
    return result;
}

inline std::map<PortId, PortSpec> MakePortMap(const std::vector<PortSpec>& specs) {
    std::map<PortId, PortSpec> result;
    for (const auto& spec : specs) {
        result.emplace(spec.id, spec);
    }
    return result;
}

class EventPipelineGraph::TaskRunner {
public:
    TaskRunner(TaskConfig config,
               std::unique_ptr<ITask> task,
               std::unordered_map<PortId, IQueue*> inputs,
               std::unordered_map<PortId, IQueue*> outputs,
               std::vector<IQueue*> triggerQueues);
    ~TaskRunner();

    const std::string& Name() const;
    void Start();
    void Stop();
    void Notify();
    TaskDiagnosticsSnapshot Diagnostics() const;

private:
    void Run();
    bool WaitForTrigger();
    bool QueuesReady() const;

    TaskConfig m_config;
    std::unique_ptr<ITask> m_task;
    TaskContext m_context;
    std::vector<IQueue*> m_triggerQueues;
    mutable ::epg::TaskDiagnostics m_diag;
    std::atomic<bool> m_running{false};
    std::thread m_thread;
    std::mutex m_mutex;
    std::condition_variable m_cv;
};

} // namespace epg
