#include "common/epg/epg_internal.h"

#include <algorithm>
#include <cerrno>
#include <limits>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <vector>

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

void UpdateMaxResourceWaitUs(TaskDiagnostics &diag, std::uint64_t waitUs)
{
    auto max = diag.maxResourceWaitUs.load(std::memory_order_relaxed);
    while (waitUs > max &&
           !diag.maxResourceWaitUs.compare_exchange_weak(
               max, waitUs,
               std::memory_order_relaxed,
               std::memory_order_relaxed)) {
    }
}

void StoreResourceWait(TaskDiagnostics &diag, std::uint64_t waitUs)
{
    if (waitUs == 0) {
        return;
    }
    diag.resourceWaitCount.fetch_add(1, std::memory_order_relaxed);
    diag.lastResourceWaitUs.store(waitUs, std::memory_order_relaxed);
    diag.totalResourceWaitUs.fetch_add(waitUs, std::memory_order_relaxed);
    UpdateMaxResourceWaitUs(diag, waitUs);
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

TaskDiagnosticsSnapshot SnapshotTaskDiagnostics(const TaskDiagnostics &diag)
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

int CreateWakeEventFd()
{
    const int fd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (fd >= 0) {
        return fd;
    }
    return -1;
}

int TimeoutUntil(const std::chrono::steady_clock::time_point &deadline)
{
    const auto remaining = deadline - std::chrono::steady_clock::now();
    const auto remainingUs =
        std::chrono::duration_cast<std::chrono::microseconds>(remaining).count();
    if (remainingUs <= 0) {
        return 0;
    }
    const auto timeoutMs = (remainingUs + 999) / 1000;
    const auto maxTimeout = std::numeric_limits<int>::max();
    return timeoutMs > maxTimeout ? maxTimeout : static_cast<int>(timeoutMs);
}

std::chrono::milliseconds WakeFallbackSleep(int timeoutMs)
{
    if (timeoutMs < 0) {
        return std::chrono::milliseconds(1);
    }
    if (timeoutMs == 0) {
        return std::chrono::milliseconds(0);
    }
    return std::chrono::milliseconds(std::min(timeoutMs, 1));
}

} // namespace

EventPipelineGraph::TaskRunner::TaskRunner(TaskRunnerSpec spec)
    : m_config(std::move(spec.config)),
      m_task(std::move(spec.task)),
      m_context(std::move(spec.inputs), std::move(spec.outputs)),
      m_triggerQueues(std::move(spec.triggerQueues)),
      m_resourceGate(std::move(spec.resourceGate)),
      m_wakeSignal(std::make_shared<TaskWakeSignal>())
{
    m_context.AttachDiagnostics(&m_diag);
}

EventPipelineGraph::TaskRunner::~TaskRunner()
{
    Stop();
}

TaskWakeSignal::TaskWakeSignal() : m_eventFd(CreateWakeEventFd())
{
}

TaskWakeSignal::~TaskWakeSignal()
{
    if (m_eventFd >= 0) {
        (void)::close(m_eventFd);
    }
}

int TaskWakeSignal::Notify() const
{
    if (m_eventFd < 0) {
        return 0;
    }
    const eventfd_t value = 1;
    if (::eventfd_write(m_eventFd, value) == 0 || errno == EAGAIN) {
        return 0;
    }
    return errno;
}

void TaskWakeSignal::TriggerExternal()
{
    m_externalTriggerPending.store(true, std::memory_order_release);
    (void)Notify();
}

bool TaskWakeSignal::ConsumeExternalTrigger()
{
    return m_externalTriggerPending.exchange(false, std::memory_order_acq_rel);
}

bool TaskWakeSignal::ExternalTriggerPending() const
{
    return m_externalTriggerPending.load(std::memory_order_acquire);
}

int TaskWakeSignal::EventFd() const
{
    return m_eventFd;
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
    const int error = m_wakeSignal->Notify();
    if (error != 0) {
        m_diag.schedulingErrorCount.fetch_add(1, std::memory_order_relaxed);
        m_diag.lastSchedulingError.store(error, std::memory_order_relaxed);
    }
}

std::weak_ptr<TaskWakeSignal>
EventPipelineGraph::TaskRunner::ExternalTriggerSignal() const
{
    return m_wakeSignal;
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
        if (!WaitForResourceGate()) {
            break;
        }

        const auto begin = std::chrono::steady_clock::now();
        try {
            m_task->OnTick(m_context);
        } catch (...) {
            m_diag.errorCount.fetch_add(1, std::memory_order_relaxed);
        }
        const auto end = std::chrono::steady_clock::now();
        ReleaseResourceGate();
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
    m_loopSamples[m_loopSampleCursor].store(elapsedUs,
                                            std::memory_order_relaxed);
    m_loopSampleCursor = (m_loopSampleCursor + 1) % m_loopSamples.size();
    auto count = m_loopSampleCount.load(std::memory_order_relaxed);
    while (count < m_loopSamples.size() &&
           !m_loopSampleCount.compare_exchange_weak(
               count, count + 1, std::memory_order_release,
               std::memory_order_relaxed)) {
    }
}

void EventPipelineGraph::TaskRunner::FillLoopPercentiles(
    TaskDiagnosticsSnapshot &snapshot) const
{
    std::vector<std::uint64_t> values;
    const auto count = std::min(m_loopSampleCount.load(std::memory_order_acquire),
                                m_loopSamples.size());
    values.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        values.push_back(m_loopSamples[index].load(std::memory_order_relaxed));
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
            return WaitForConfiguredPhase();
        }
        if (m_config.trigger.interval.count() > 0) {
            return WaitForDuration(m_config.trigger.interval);
        }
        return m_running.load(std::memory_order_acquire);

    case TriggerMode::AnyQueueReady:
    case TriggerMode::AllQueueReady:
        return WaitForQueueTrigger();

    case TriggerMode::PeriodicOrAnyQueueReady:
        return WaitForPeriodicOrQueueTrigger();

    case TriggerMode::PeriodicOrExternal:
        return WaitForPeriodicOrExternalTrigger();
    }
    return false;
}

bool EventPipelineGraph::TaskRunner::WaitForConfiguredPhase()
{
    if (m_config.scheduling.phaseOffsetMs == 0) {
        return m_running.load(std::memory_order_acquire);
    }
    return WaitForDuration(
        std::chrono::milliseconds(m_config.scheduling.phaseOffsetMs));
}

bool EventPipelineGraph::TaskRunner::WaitForDuration(
    std::chrono::milliseconds duration)
{
    if (duration.count() <= 0) {
        return m_running.load(std::memory_order_acquire);
    }
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (m_running.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() < deadline) {
        PollWakeEvent(TimeoutUntil(deadline));
    }
    return m_running.load(std::memory_order_acquire);
}

bool EventPipelineGraph::TaskRunner::WaitForQueueTrigger()
{
    while (m_running.load(std::memory_order_acquire) && !QueuesReady()) {
        PollWakeEvent(-1);
    }
    return m_running.load(std::memory_order_acquire);
}

bool EventPipelineGraph::TaskRunner::WaitForPeriodicOrQueueTrigger()
{
    if (QueuesReady()) {
        return m_running.load(std::memory_order_acquire);
    }
    const auto deadline =
        std::chrono::steady_clock::now() + m_config.trigger.interval;
    while (m_running.load(std::memory_order_acquire) && !QueuesReady() &&
           std::chrono::steady_clock::now() < deadline) {
        PollWakeEvent(TimeoutUntil(deadline));
    }
    return m_running.load(std::memory_order_acquire);
}

bool EventPipelineGraph::TaskRunner::WaitForPeriodicOrExternalTrigger()
{
    if (m_wakeSignal->ConsumeExternalTrigger()) {
        return m_running.load(std::memory_order_acquire);
    }
    const auto deadline =
        std::chrono::steady_clock::now() + m_config.trigger.interval;
    while (m_running.load(std::memory_order_acquire) &&
           !m_wakeSignal->ExternalTriggerPending() &&
           std::chrono::steady_clock::now() < deadline) {
        PollWakeEvent(TimeoutUntil(deadline));
    }
    (void)m_wakeSignal->ConsumeExternalTrigger();
    return m_running.load(std::memory_order_acquire);
}

void EventPipelineGraph::TaskRunner::PollWakeEvent(int timeoutMs)
{
    if (m_wakeSignal->EventFd() < 0) {
        std::this_thread::sleep_for(WakeFallbackSleep(timeoutMs));
        return;
    }
    pollfd fd{};
    fd.fd = m_wakeSignal->EventFd();
    fd.events = POLLIN;
    while (m_running.load(std::memory_order_acquire)) {
        const int result = ::poll(&fd, 1, timeoutMs);
        if (result >= 0 || errno != EINTR) {
            break;
        }
    }
    DrainWakeEvents();
}

void EventPipelineGraph::TaskRunner::DrainWakeEvents()
{
    if (m_wakeSignal->EventFd() < 0) {
        return;
    }
    eventfd_t value = 0;
    while (::eventfd_read(m_wakeSignal->EventFd(), &value) == 0) {
    }
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

bool EventPipelineGraph::TaskRunner::WaitForResourceGate()
{
    if (!m_resourceGate) {
        return true;
    }
    const auto waitStart = std::chrono::steady_clock::now();
    bool waited = false;
    while (m_running.load(std::memory_order_acquire)) {
        bool idle = false;
        if (m_resourceGate->busy.compare_exchange_strong(
                idle, true, std::memory_order_acq_rel,
                std::memory_order_relaxed)) {
            if (waited) {
                const auto waitUs =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - waitStart).count();
                StoreResourceWait(m_diag, static_cast<std::uint64_t>(waitUs));
            }
            return true;
        }
        waited = true;
        PollWakeEvent(1);
    }
    return false;
}

void EventPipelineGraph::TaskRunner::ReleaseResourceGate()
{
    if (!m_resourceGate) {
        return;
    }
    m_resourceGate->busy.store(false, std::memory_order_release);
}

} // namespace Epg
