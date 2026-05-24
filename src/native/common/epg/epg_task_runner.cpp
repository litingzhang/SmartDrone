#include "common/epg/epg_internal.h"

#include <vector>
#include <pthread.h>
#include <sched.h>

namespace Epg {
namespace {

std::uint64_t SteadyNowMs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

void StoreFirstLoopTime(TaskDiagnostics &diag, std::uint64_t nowMs)
{
    std::uint64_t empty = 0;
    (void)diag.firstLoopMs.compare_exchange_strong(
        empty, nowMs, std::memory_order_relaxed, std::memory_order_relaxed);
}

void UpdateMaxLoopUs(TaskDiagnostics &diag, std::uint64_t elapsedUs)
{
    auto max = diag.maxLoopUs.load(std::memory_order_relaxed);
    while (elapsedUs > max &&
           !diag.maxLoopUs.compare_exchange_weak(
               max, elapsedUs,
               std::memory_order_relaxed,
               std::memory_order_relaxed)) {
    }
}

void StoreLoopTiming(TaskDiagnostics &diag, std::uint64_t elapsedUs)
{
    const std::uint64_t nowMs = SteadyNowMs();
    StoreFirstLoopTime(diag, nowMs);
    diag.lastLoopMs.store(nowMs, std::memory_order_relaxed);
    diag.lastLoopUs.store(elapsedUs, std::memory_order_relaxed);
    diag.totalLoopUs.fetch_add(elapsedUs, std::memory_order_relaxed);
    UpdateMaxLoopUs(diag, elapsedUs);
}

void StoreBudgetDiagnostics(TaskDiagnostics &diag,
                            const TaskSchedulingConfig &scheduling,
                            std::uint64_t elapsedUs)
{
    if (scheduling.budgetUs > 0 && elapsedUs > scheduling.budgetUs) {
        diag.budgetOverrunCount.fetch_add(1, std::memory_order_relaxed);
    }
    if (scheduling.deadlineUs > 0 && elapsedUs > scheduling.deadlineUs) {
        diag.deadlineMissCount.fetch_add(1, std::memory_order_relaxed);
    }
}

std::uint64_t PercentileValue(std::vector<std::uint64_t> values,
                              std::uint64_t percentile)
{
    if (values.empty()) {
        return 0;
    }
    std::sort(values.begin(), values.end());
    const auto index =
        ((values.size() - 1) * percentile + 99) / 100;
    return values[index];
}

bool WaitUntilRunningOrStopped(std::condition_variable &cv,
                               std::mutex &mutex,
                               const std::chrono::steady_clock::time_point &time,
                               const std::atomic<bool> &running)
{
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait_until(lock, time, [&running]() {
        return !running.load(std::memory_order_acquire);
    });
    return running.load(std::memory_order_acquire);
}

bool WaitForConfiguredPhase(std::condition_variable &cv,
                            std::mutex &mutex,
                            const TaskSchedulingConfig &scheduling,
                            const std::atomic<bool> &running)
{
    if (!scheduling.phaseOffsetConfigured ||
        scheduling.phaseOffsetMs == 0) {
        return running.load(std::memory_order_acquire);
    }
    const auto wakeAt = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(scheduling.phaseOffsetMs);
    return WaitUntilRunningOrStopped(cv, mutex, wakeAt, running);
}

} // namespace

EventPipelineGraph::TaskRunner::TaskRunner(TaskConfig config,
                                           std::unique_ptr<ITask> task,
                                           std::unordered_map<PortId, IQueue *> inputs,
                                           std::unordered_map<PortId, IQueue *> outputs,
                                           std::vector<IQueue *> triggerQueues)
    : m_config(std::move(config)),
      m_task(std::move(task)),
      m_context(std::move(inputs), std::move(outputs)),
      m_triggerQueues(std::move(triggerQueues))
{
    m_context.AttachDiagnostics(&m_diag);
}

EventPipelineGraph::TaskRunner::~TaskRunner()
{
    Stop();
}

const std::string &EventPipelineGraph::TaskRunner::Name() const
{
    return m_config.name;
}

void EventPipelineGraph::TaskRunner::Start()
{
    if (m_running.exchange(true, std::memory_order_acq_rel)) {
        return;
    }
    m_exited.store(false, std::memory_order_release);
    m_thread = std::thread([this]() { Run(); });
}

void EventPipelineGraph::TaskRunner::RequestStop()
{
    if (!m_running.exchange(false, std::memory_order_acq_rel)) {
        return;
    }
    Notify();
}

bool EventPipelineGraph::TaskRunner::JoinStopped()
{
    if (m_running.load(std::memory_order_acquire)) {
        return false;
    }
    if (!m_exited.load(std::memory_order_acquire)) {
        return false;
    }
    if (m_thread.joinable()) {
        m_thread.join();
    }
    return true;
}

void EventPipelineGraph::TaskRunner::Stop()
{
    RequestStop();
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void EventPipelineGraph::TaskRunner::Notify()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
    }
    m_cv.notify_one();
}

TaskDiagnosticsSnapshot EventPipelineGraph::TaskRunner::Diagnostics() const
{
    auto snapshot = SnapshotTaskDiagnostics(m_diag);
    FillLoopPercentiles(snapshot);
    return snapshot;
}

void EventPipelineGraph::TaskRunner::Run()
{
    ApplyScheduling();
    while (m_running.load(std::memory_order_acquire)) {
        if (!WaitForTrigger()) {
            break;
        }

        if (m_config.trigger.mode != TriggerMode::PeriodicOrAnyQueueReady &&
            IsQueueTriggeredMode(m_config.trigger.mode) && !QueuesReady()) {
            m_diag.idleWakeups.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        if (BackpressureBlocked()) {
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
        const auto elapsed = static_cast<std::uint64_t>(elapsedUs);
        StoreLoopTiming(m_diag, elapsed);
        StoreBudgetDiagnostics(m_diag, m_config.scheduling, elapsed);
        StoreLoopSample(elapsed);
        m_diag.loopCount.fetch_add(1, std::memory_order_relaxed);
    }
    m_exited.store(true, std::memory_order_release);
}

void EventPipelineGraph::TaskRunner::StoreLoopSample(std::uint64_t elapsedUs)
{
    std::lock_guard<std::mutex> lock(m_sampleMutex);
    m_loopSamples[m_loopSampleCursor] = elapsedUs;
    m_loopSampleCursor = (m_loopSampleCursor + 1) % m_loopSamples.size();
    if (m_loopSampleCount < m_loopSamples.size()) {
        ++m_loopSampleCount;
    }
}

void EventPipelineGraph::TaskRunner::FillLoopPercentiles(
    TaskDiagnosticsSnapshot &snapshot) const
{
    std::vector<std::uint64_t> values;
    {
        std::lock_guard<std::mutex> lock(m_sampleMutex);
        values.assign(m_loopSamples.begin(),
                      m_loopSamples.begin() + m_loopSampleCount);
    }
    snapshot.p50LoopUs = PercentileValue(values, 50);
    snapshot.p90LoopUs = PercentileValue(values, 90);
    snapshot.p99LoopUs = PercentileValue(values, 99);
}

namespace {

void RecordSchedulingError(TaskDiagnostics &diag, int error)
{
    if (error == 0) {
        return;
    }
    diag.schedulingErrorCount.fetch_add(1, std::memory_order_relaxed);
    diag.lastSchedulingError.store(error, std::memory_order_relaxed);
}

void ApplyRealtimeScheduling(const TaskSchedulingConfig &scheduling,
                             TaskDiagnostics &diag)
{
    if (!scheduling.realtime) {
        return;
    }
    sched_param schedParam{};
    schedParam.sched_priority = scheduling.priority;
    RecordSchedulingError(
        diag, pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedParam));
}

void ApplyCpuAffinity(const TaskSchedulingConfig &scheduling,
                      TaskDiagnostics &diag)
{
    if (scheduling.cpuAffinity < 0) {
        return;
    }
    cpu_set_t cpuSet;
    CPU_ZERO(&cpuSet);
    CPU_SET(scheduling.cpuAffinity, &cpuSet);
    RecordSchedulingError(
        diag, pthread_setaffinity_np(pthread_self(), sizeof(cpuSet), &cpuSet));
}

} // namespace

void EventPipelineGraph::TaskRunner::ApplyScheduling()
{
    ApplyCpuAffinity(m_config.scheduling, m_diag);
    ApplyRealtimeScheduling(m_config.scheduling, m_diag);
}

bool EventPipelineGraph::TaskRunner::WaitForTrigger()
{
    switch (m_config.trigger.mode) {
    case TriggerMode::Periodic:
        if (m_config.scheduling.phaseOffsetConfigured &&
            m_diag.loopCount.load(std::memory_order_relaxed) == 0) {
            return WaitForConfiguredPhase(
                m_cv, m_mutex, m_config.scheduling, m_running);
        }
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

bool EventPipelineGraph::TaskRunner::QueuesReady() const
{
    if (m_triggerQueues.empty()) {
        return false;
    }
    if (m_config.trigger.mode == TriggerMode::AllQueueReady) {
        return std::all_of(m_triggerQueues.begin(), m_triggerQueues.end(),
                           [](const IQueue *queue) { return !queue->Empty(); });
    }
    return std::any_of(m_triggerQueues.begin(), m_triggerQueues.end(),
                       [](const IQueue *queue) { return !queue->Empty(); });
}

bool EventPipelineGraph::TaskRunner::BackpressureBlocked() const
{
    for (const auto port : m_config.scheduling.backpressureOutputs) {
        const auto queueIt = m_context.OutputQueueByPort(port);
        if (!queueIt || queueIt->Size() >= queueIt->Depth()) {
            return true;
        }
    }
    return false;
}

} // namespace Epg
