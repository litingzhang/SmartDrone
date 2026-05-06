#include "common/epg/epg_internal.h"

namespace epg {

EventPipelineGraph::TaskRunner::TaskRunner(TaskConfig config,
                                     std::unique_ptr<ITask> task,
                                     std::unordered_map<PortId, IQueue*> inputs,
                                     std::unordered_map<PortId, IQueue*> outputs,
                                     std::vector<IQueue*> triggerQueues)
    : m_config(std::move(config)),
      m_task(std::move(task)),
      m_context(std::move(inputs), std::move(outputs)),
      m_triggerQueues(std::move(triggerQueues)) {
}

EventPipelineGraph::TaskRunner::~TaskRunner() {
    Stop();
}

const std::string& EventPipelineGraph::TaskRunner::Name() const {
    return m_config.name;
}

void EventPipelineGraph::TaskRunner::Start() {
    if (m_running.exchange(true, std::memory_order_acq_rel)) {
        return;
    }
    m_thread = std::thread([this]() { Run(); });
}

void EventPipelineGraph::TaskRunner::Stop() {
    if (!m_running.exchange(false, std::memory_order_acq_rel)) {
        return;
    }
    Notify();
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void EventPipelineGraph::TaskRunner::Notify() {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
    }
    m_cv.notify_one();
}

TaskDiagnosticsSnapshot EventPipelineGraph::TaskRunner::Diagnostics() const {
    return SnapshotTaskDiagnostics(m_diag);
}

void EventPipelineGraph::TaskRunner::Run() {
    while (m_running.load(std::memory_order_acquire)) {
        if (!WaitForTrigger()) {
            break;
        }

        if (IsQueueTriggeredMode(m_config.trigger.mode) && !QueuesReady()) {
            m_diag.idleWakeups.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        const auto begin = std::chrono::steady_clock::now();
        try {
            m_task->OnTick(m_context);
        } catch (...) {
            m_diag.errorCount.fetch_add(1, std::memory_order_relaxed);
        }
        const auto end = std::chrono::steady_clock::now();
        const auto elapsedUs =
            std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        m_diag.lastLoopUs.store(static_cast<std::uint64_t>(elapsedUs), std::memory_order_relaxed);
        auto max = m_diag.maxLoopUs.load(std::memory_order_relaxed);
        while (static_cast<std::uint64_t>(elapsedUs) > max &&
               !m_diag.maxLoopUs.compare_exchange_weak(max,
                                                       static_cast<std::uint64_t>(elapsedUs),
                                                       std::memory_order_relaxed,
                                                       std::memory_order_relaxed)) {
        }
        m_diag.loopCount.fetch_add(1, std::memory_order_relaxed);
    }
}

bool EventPipelineGraph::TaskRunner::WaitForTrigger() {
    switch (m_config.trigger.mode) {
        case TriggerMode::Periodic:
            if (m_config.trigger.interval.count() > 0) {
                std::unique_lock<std::mutex> lock(m_mutex);
                m_cv.wait_for(lock, m_config.trigger.interval,
                              [this]() { return !m_running.load(std::memory_order_acquire); });
            }
            return m_running.load(std::memory_order_acquire);

        case TriggerMode::AnyQueueReady:
        case TriggerMode::AllQueueReady: {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [this]() {
                return !m_running.load(std::memory_order_acquire) || QueuesReady();
            });
            return m_running.load(std::memory_order_acquire);
        }

        case TriggerMode::PeriodicOrAnyQueueReady: {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait_for(lock, m_config.trigger.interval, [this]() {
                return !m_running.load(std::memory_order_acquire) || QueuesReady();
            });
            return m_running.load(std::memory_order_acquire);
        }
    }
    return false;
}

bool EventPipelineGraph::TaskRunner::QueuesReady() const {
    if (m_triggerQueues.empty()) {
        return false;
    }
    if (m_config.trigger.mode == TriggerMode::AllQueueReady) {
        return std::all_of(m_triggerQueues.begin(), m_triggerQueues.end(),
                           [](const IQueue* queue) { return !queue->Empty(); });
    }
    return std::any_of(m_triggerQueues.begin(), m_triggerQueues.end(),
                       [](const IQueue* queue) { return !queue->Empty(); });
}

} // namespace epg
