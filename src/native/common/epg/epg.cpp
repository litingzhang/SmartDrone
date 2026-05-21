#include "common/epg/epg_internal.h"

#include <functional>
#include <set>
#include <sstream>

namespace epg {
namespace {

constexpr int MIN_REALTIME_PRIORITY = 1;
constexpr int MAX_REALTIME_PRIORITY = 99;
constexpr std::uint64_t MICROS_PER_MILLI = 1000;
constexpr std::uint64_t MILLIS_PER_SECOND = 1000;

std::string JsonEscape(const std::string& value) {
    std::ostringstream out;
    for (const char ch : value) {
        switch (ch) {
        case '\\':
            out << "\\\\";
            break;
        case '"':
            out << "\\\"";
            break;
        case '\n':
            out << "\\n";
            break;
        case '\r':
            out << "\\r";
            break;
        case '\t':
            out << "\\t";
            break;
        default:
            out << ch;
            break;
        }
    }
    return out.str();
}

void ValidateTaskScheduling(const TaskConfig& taskConfig) {
    const auto& scheduling = taskConfig.scheduling;
    if (scheduling.resource.empty()) {
        throw std::runtime_error("task scheduling resource must not be empty: " +
                                 taskConfig.name);
    }
    if (scheduling.cpuAffinity < -1) {
        throw std::runtime_error("task cpu_affinity must be -1 or greater: " +
                                 taskConfig.name);
    }
    if (scheduling.deadlineUs > 0 &&
        scheduling.budgetUs > scheduling.deadlineUs) {
        throw std::runtime_error("task budget_us must not exceed deadline_us: " +
                                 taskConfig.name);
    }
    if (!scheduling.realtime) {
        return;
    }
    if (scheduling.priority < MIN_REALTIME_PRIORITY ||
        scheduling.priority > MAX_REALTIME_PRIORITY) {
        throw std::runtime_error("realtime task priority must be in [1,99]: " +
                                 taskConfig.name);
    }
}

const char* OverflowPolicyName(OverflowPolicy policy)
{
    switch (policy) {
    case OverflowPolicy::DropNewest:
        return "drop_newest";
    case OverflowPolicy::OverwriteOldest:
        return "overwrite_oldest";
    }
    return "unknown";
}

const char* TriggerModeName(TriggerMode mode)
{
    switch (mode) {
    case TriggerMode::Periodic:
        return "periodic";
    case TriggerMode::AnyQueueReady:
        return "any_queue_ready";
    case TriggerMode::AllQueueReady:
        return "all_queue_ready";
    case TriggerMode::PeriodicOrAnyQueueReady:
        return "periodic_or_any_queue_ready";
    }
    return "unknown";
}

void WriteStringArray(std::ostringstream& out,
                      const std::vector<std::string>& values)
{
    out << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            out << ", ";
        }
        out << "\"" << JsonEscape(values[i]) << "\"";
    }
    out << "]";
}

void WritePortMap(std::ostringstream& out,
                  const std::map<PortId, std::string>& ports)
{
    out << "{";
    bool first = true;
    for (const auto& port : ports) {
        if (!first) {
            out << ", ";
        }
        first = false;
        out << "\"" << port.first << "\": \"" << JsonEscape(port.second) << "\"";
    }
    out << "}";
}

void WritePortIdArray(std::ostringstream& out,
                      const std::vector<PortId>& values)
{
    out << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            out << ", ";
        }
        out << values[i];
    }
    out << "]";
}

void WriteQueueConfigJson(std::ostringstream& out,
                          const QueueConfig& queueConfig)
{
    out << "{";
    out << "\"name\": \"" << JsonEscape(queueConfig.name) << "\", ";
    out << "\"type\": \"" << JsonEscape(queueConfig.type) << "\", ";
    out << "\"depth\": " << queueConfig.depth << ", ";
    out << "\"overflow\": \"" << OverflowPolicyName(queueConfig.overflow) << "\"";
    out << "}";
}

void WriteTaskConfigJson(std::ostringstream& out,
                         const TaskConfig& taskConfig)
{
    out << "{";
    out << "\"name\": \"" << JsonEscape(taskConfig.name) << "\", ";
    out << "\"type\": \"" << JsonEscape(taskConfig.type) << "\", ";
    out << "\"trigger\": {";
    out << "\"mode\": \"" << TriggerModeName(taskConfig.trigger.mode) << "\", ";
    out << "\"interval_ms\": " << taskConfig.trigger.interval.count() << ", ";
    out << "\"queues\": ";
    WriteStringArray(out, taskConfig.trigger.queues);
    out << "}, ";
    out << "\"scheduling\": {";
    out << "\"resource\": \"" << JsonEscape(taskConfig.scheduling.resource) << "\", ";
    out << "\"cpu_affinity\": " << taskConfig.scheduling.cpuAffinity << ", ";
    out << "\"budget_us\": " << taskConfig.scheduling.budgetUs << ", ";
    out << "\"deadline_us\": " << taskConfig.scheduling.deadlineUs << ", ";
    out << "\"backpressure_outputs\": ";
    WritePortIdArray(out, taskConfig.scheduling.backpressureOutputs);
    out << ", ";
    out << "\"realtime\": " << (taskConfig.scheduling.realtime ? "true" : "false") << ", ";
    out << "\"priority\": " << taskConfig.scheduling.priority;
    out << "}, ";
    out << "\"inputs\": ";
    WritePortMap(out, taskConfig.inputs);
    out << ", \"outputs\": ";
    WritePortMap(out, taskConfig.outputs);
    out << "}";
}

void WriteJsonMetadata(
    std::ostringstream& out,
    const std::map<std::string, std::string>& stringMetadata,
    const std::map<std::string, std::uint64_t>& numericMetadata)
{
    for (const auto& item : stringMetadata) {
        out << "  \"" << JsonEscape(item.first) << "\": \""
            << JsonEscape(item.second) << "\",\n";
    }
    for (const auto& item : numericMetadata) {
        out << "  \"" << JsonEscape(item.first) << "\": "
            << item.second << ",\n";
    }
}

void WriteGraphConfigBody(std::ostringstream& out, const GraphConfig& config)
{
    out << "  \"queues\": [\n";
    for (std::size_t i = 0; i < config.queues.size(); ++i) {
        if (i != 0) {
            out << ",\n";
        }
        out << "    ";
        WriteQueueConfigJson(out, config.queues[i]);
    }
    out << "\n  ],\n";
    out << "  \"tasks\": [\n";
    for (std::size_t i = 0; i < config.tasks.size(); ++i) {
        if (i != 0) {
            out << ",\n";
        }
        out << "    ";
        WriteTaskConfigJson(out, config.tasks[i]);
    }
    out << "\n  ]\n";
}

std::uint64_t ObservationWindowMs(const TaskDiagnosticsSnapshot& diag)
{
    if (diag.firstLoopMs == 0 || diag.lastLoopMs <= diag.firstLoopMs) {
        return 0;
    }
    return diag.lastLoopMs - diag.firstLoopMs;
}

std::uint64_t AverageLoopUs(const TaskDiagnosticsSnapshot& diag)
{
    if (diag.loopCount == 0) {
        return 0;
    }
    return diag.totalLoopUs / diag.loopCount;
}

std::uint64_t AverageResourceWaitUs(const TaskDiagnosticsSnapshot& diag)
{
    if (diag.resourceWaitCount == 0) {
        return 0;
    }
    return diag.totalResourceWaitUs / diag.resourceWaitCount;
}

std::uint64_t UtilizationPpm(const TaskDiagnosticsSnapshot& diag)
{
    const std::uint64_t windowMs = ObservationWindowMs(diag);
    if (windowMs == 0) {
        return 0;
    }
    return (diag.totalLoopUs * MICROS_PER_MILLI) / windowMs;
}

std::uint64_t QueueObservationWindowMs(const QueueDiagnosticsSnapshot& diag)
{
    if (diag.firstActivityMs == 0 || diag.lastActivityMs <= diag.firstActivityMs) {
        return 0;
    }
    return diag.lastActivityMs - diag.firstActivityMs;
}

std::uint64_t CounterRatePerSecond(std::uint64_t count, std::uint64_t windowMs)
{
    if (windowMs == 0) {
        return 0;
    }
    return (count * MILLIS_PER_SECOND) / windowMs;
}

void WriteQueueDiagnosticsJson(std::ostringstream& out,
                               const IQueue& queue,
                               const QueueDiagnosticsSnapshot& diag)
{
    const std::uint64_t windowMs = QueueObservationWindowMs(diag);
    out << "\"type\": \"" << JsonEscape(queue.TypeName()) << "\", ";
    out << "\"size\": " << queue.Size() << ", ";
    out << "\"depth\": " << queue.Depth() << ", ";
    out << "\"pushed\": " << diag.pushed << ", ";
    out << "\"popped\": " << diag.popped << ", ";
    out << "\"droppedNewest\": " << diag.droppedNewest << ", ";
    out << "\"overwrittenOldest\": " << diag.overwrittenOldest << ", ";
    out << "\"wakeups\": " << diag.wakeups << ", ";
    out << "\"maxDepthObserved\": " << diag.maxDepthObserved << ", ";
    out << "\"firstActivityMs\": " << diag.firstActivityMs << ", ";
    out << "\"lastActivityMs\": " << diag.lastActivityMs << ", ";
    out << "\"windowMs\": " << windowMs << ", ";
    out << "\"pushedPerSecond\": " << CounterRatePerSecond(diag.pushed, windowMs) << ", ";
    out << "\"poppedPerSecond\": " << CounterRatePerSecond(diag.popped, windowMs) << ", ";
    out << "\"droppedPerSecond\": " <<
        CounterRatePerSecond(diag.droppedNewest + diag.overwrittenOldest, windowMs);
}

void WriteTaskDiagnosticsJson(std::ostringstream& out,
                              const TaskDiagnosticsSnapshot& diag)
{
    out << "\"lastLoopUs\": " << diag.lastLoopUs << ", ";
    out << "\"maxLoopUs\": " << diag.maxLoopUs << ", ";
    out << "\"p50LoopUs\": " << diag.p50LoopUs << ", ";
    out << "\"p90LoopUs\": " << diag.p90LoopUs << ", ";
    out << "\"p99LoopUs\": " << diag.p99LoopUs << ", ";
    out << "\"totalLoopUs\": " << diag.totalLoopUs << ", ";
    out << "\"averageLoopUs\": " << AverageLoopUs(diag) << ", ";
    out << "\"resourceWaitCount\": " << diag.resourceWaitCount << ", ";
    out << "\"lastResourceWaitUs\": " << diag.lastResourceWaitUs << ", ";
    out << "\"maxResourceWaitUs\": " << diag.maxResourceWaitUs << ", ";
    out << "\"totalResourceWaitUs\": " << diag.totalResourceWaitUs << ", ";
    out << "\"averageResourceWaitUs\": " << AverageResourceWaitUs(diag) << ", ";
    out << "\"loopCount\": " << diag.loopCount << ", ";
    out << "\"errorCount\": " << diag.errorCount << ", ";
    out << "\"idleWakeups\": " << diag.idleWakeups << ", ";
    out << "\"firstLoopMs\": " << diag.firstLoopMs << ", ";
    out << "\"lastLoopMs\": " << diag.lastLoopMs << ", ";
    out << "\"windowMs\": " << ObservationWindowMs(diag) << ", ";
    out << "\"utilizationPpm\": " << UtilizationPpm(diag) << ", ";
    out << "\"budgetOverrunCount\": " << diag.budgetOverrunCount << ", ";
    out << "\"deadlineMissCount\": " << diag.deadlineMissCount << ", ";
    out << "\"schedulingErrorCount\": " << diag.schedulingErrorCount << ", ";
    out << "\"lastSchedulingError\": " << diag.lastSchedulingError;
}

} // namespace

std::string GraphConfigToJson(
    const GraphConfig& config,
    const std::map<std::string, std::string>& stringMetadata,
    const std::map<std::string, std::uint64_t>& numericMetadata)
{
    std::ostringstream out;
    out << "{\n";
    WriteJsonMetadata(out, stringMetadata, numericMetadata);
    WriteGraphConfigBody(out, config);
    out << "}\n";
    return out.str();
}

EventPipelineGraph::EventPipelineGraph(const Registry& registry) : m_registry(registry) {
}

EventPipelineGraph::~EventPipelineGraph() {
    Stop();
}

struct EventPipelineGraph::ConfigureUsage {
    std::set<std::string> queueNames;
    std::set<std::string> taskNames;
    std::map<std::string, int> producers;
    std::map<std::string, int> consumers;
};

void EventPipelineGraph::ConfigureJson(const std::string& jsonText) {
    Configure(ParseGraphConfigJson(jsonText));
}

void EventPipelineGraph::Configure(const GraphConfig& config) {
    if (m_running) {
        throw std::runtime_error("cannot configure a running EventPipelineGraph");
    }

    ConfigureUsage usage;
    ResetConfiguredGraph();
    CreateConfiguredQueues(config, usage);
    ValidateConfiguredTasks(config, usage);
    PublishTaskProducedQueues(usage);
    CreateConfiguredTaskRunners(config);
    BindInputNotifiers(config);
    m_config = config;
    m_configured = true;
}

void EventPipelineGraph::ResetConfiguredGraph() {
    m_queues.clear();
    m_runners.clear();
    m_taskProducedQueues.clear();
    m_externalIngressQueues.clear();
    m_config = {};
}

void EventPipelineGraph::CreateConfiguredQueues(const GraphConfig& config,
                                                ConfigureUsage& usage) {
    for (const auto& queueConfig : config.queues) {
        if (queueConfig.name.empty()) {
            throw std::runtime_error("queue name must not be empty");
        }
        if (!usage.queueNames.insert(queueConfig.name).second) {
            throw std::runtime_error("duplicate queue name: " + queueConfig.name);
        }
        if (queueConfig.depth == 0) {
            throw std::runtime_error("queue depth must be greater than zero: " +
                                     queueConfig.name);
        }
        const auto* type = m_registry.FindQueueType(queueConfig.type);
        if (!type) {
            throw std::runtime_error("unregistered queue type: " + queueConfig.type);
        }
        m_queues[queueConfig.name] = type->factory(queueConfig);
    }
}

void EventPipelineGraph::ValidateConfiguredTasks(const GraphConfig& config,
                                                 ConfigureUsage& usage) const {
    for (const auto& taskConfig : config.tasks) {
        ValidateTaskConfig(taskConfig, usage);
    }
}

void EventPipelineGraph::ValidateTaskConfig(const TaskConfig& taskConfig,
                                            ConfigureUsage& usage) const {
    if (taskConfig.name.empty()) {
        throw std::runtime_error("task name must not be empty");
    }
    if (!usage.taskNames.insert(taskConfig.name).second) {
        throw std::runtime_error("duplicate task name: " + taskConfig.name);
    }
    const auto* taskType = m_registry.FindTaskType(taskConfig.type);
    if (!taskType) {
        throw std::runtime_error("unregistered task type: " + taskConfig.type);
    }
    ValidateTaskScheduling(taskConfig);
    ValidateTaskPorts(taskConfig, *taskType, usage);
    ValidateTaskTrigger(taskConfig);
}

void EventPipelineGraph::ValidateTaskPorts(
    const TaskConfig& taskConfig,
    const Registry::TaskTypeInfo& taskType,
    ConfigureUsage& usage) const {
    ValidateTaskInputs(taskConfig, MakePortMap(taskType.inputs), usage);
    ValidateTaskOutputs(taskConfig, MakePortMap(taskType.outputs), usage);
}

void EventPipelineGraph::ValidateTaskInputs(
    const TaskConfig& taskConfig,
    const std::map<PortId, PortSpec>& declaredInputs,
    ConfigureUsage& usage) const {
    for (const auto& input : taskConfig.inputs) {
        const auto specIt = declaredInputs.find(input.first);
        if (specIt == declaredInputs.end()) {
            throw std::runtime_error("task input port is not declared: " +
                                     taskConfig.name + "." + std::to_string(input.first));
        }
        const auto queueIt = m_queues.find(input.second);
        if (queueIt == m_queues.end()) {
            throw std::runtime_error("task input references missing queue: " +
                                     taskConfig.name + "." + std::to_string(input.first) +
                                     " -> " + input.second);
        }
        if (queueIt->second->TypeName() != specIt->second.type) {
            throw std::runtime_error("task input type mismatch: " +
                                     taskConfig.name + "." + std::to_string(input.first));
        }
        usage.consumers[input.second] += 1;
    }
}

void EventPipelineGraph::ValidateTaskOutputs(
    const TaskConfig& taskConfig,
    const std::map<PortId, PortSpec>& declaredOutputs,
    ConfigureUsage& usage) const {
    for (const auto& output : taskConfig.outputs) {
        const auto specIt = declaredOutputs.find(output.first);
        if (specIt == declaredOutputs.end()) {
            throw std::runtime_error("task output port is not declared: " +
                                     taskConfig.name + "." + std::to_string(output.first));
        }
        const auto queueIt = m_queues.find(output.second);
        if (queueIt == m_queues.end()) {
            throw std::runtime_error("task output references missing queue: " +
                                     taskConfig.name + "." + std::to_string(output.first) +
                                     " -> " + output.second);
        }
        if (queueIt->second->TypeName() != specIt->second.type) {
            throw std::runtime_error("task output type mismatch: " +
                                     taskConfig.name + "." + std::to_string(output.first));
        }
        usage.producers[output.second] += 1;
    }
}

void EventPipelineGraph::ValidateTaskTrigger(const TaskConfig& taskConfig) const {
    if (taskConfig.trigger.mode == TriggerMode::Periodic ||
        taskConfig.trigger.mode == TriggerMode::PeriodicOrAnyQueueReady) {
        if (taskConfig.trigger.interval.count() <= 0) {
            throw std::runtime_error("periodic task interval_ms must be greater than zero: " +
                                     taskConfig.name);
        }
    }
    if (IsQueueTriggeredMode(taskConfig.trigger.mode)) {
        ValidateTriggerQueues(taskConfig);
        return;
    }
    if (taskConfig.inputs.empty() && taskConfig.trigger.interval.count() <= 0) {
        throw std::runtime_error("task has no wake source: " + taskConfig.name);
    }
}

void EventPipelineGraph::ValidateTriggerQueues(
    const TaskConfig& taskConfig) const {
    if (taskConfig.trigger.queues.empty()) {
        throw std::runtime_error("queue-triggered task has no trigger queues: " +
                                 taskConfig.name);
    }
    for (const auto& triggerQueue : taskConfig.trigger.queues) {
        if (m_queues.find(triggerQueue) == m_queues.end()) {
            throw std::runtime_error("task trigger references missing queue: " +
                                     taskConfig.name + " -> " + triggerQueue);
        }
        const auto usedAsInput =
            std::find_if(taskConfig.inputs.begin(), taskConfig.inputs.end(),
                         [&triggerQueue](const std::pair<const PortId, std::string>& input) {
                             return input.second == triggerQueue;
                         }) != taskConfig.inputs.end();
        if (!usedAsInput) {
            throw std::runtime_error("trigger queue must also be a task input: " +
                                     taskConfig.name + " -> " + triggerQueue);
        }
    }
}

void EventPipelineGraph::PublishTaskProducedQueues(const ConfigureUsage& usage) {
    for (const auto& producer : usage.producers) {
        if (producer.second > 1) {
            throw std::runtime_error("SPSC queue has multiple producers: " + producer.first);
        }
        m_taskProducedQueues.insert(producer.first);
    }
    for (const auto& consumer : usage.consumers) {
        if (consumer.second > 1) {
            throw std::runtime_error("SPSC queue has multiple consumers: " + consumer.first);
        }
    }
}

void EventPipelineGraph::CreateConfiguredTaskRunners(const GraphConfig& config) {
    for (const auto& taskConfig : config.tasks) {
        const auto* taskType = m_registry.FindTaskType(taskConfig.type);
        m_runners.emplace_back(new TaskRunner(
            taskConfig,
            taskType->factory(),
            MakeInputQueueBindings(taskConfig),
            MakeOutputQueueBindings(taskConfig),
            MakeTriggerQueueBindings(taskConfig)));
    }
}

std::unordered_map<PortId, IQueue*>
EventPipelineGraph::MakeInputQueueBindings(const TaskConfig& taskConfig) const {
    std::unordered_map<PortId, IQueue*> inputs;
    for (const auto& input : taskConfig.inputs) {
        inputs[input.first] = m_queues.at(input.second).get();
    }
    return inputs;
}

std::unordered_map<PortId, IQueue*>
EventPipelineGraph::MakeOutputQueueBindings(const TaskConfig& taskConfig) const {
    std::unordered_map<PortId, IQueue*> outputs;
    for (const auto& output : taskConfig.outputs) {
        outputs[output.first] = m_queues.at(output.second).get();
    }
    return outputs;
}

std::vector<IQueue*>
EventPipelineGraph::MakeTriggerQueueBindings(const TaskConfig& taskConfig) const {
    std::vector<IQueue*> triggerQueues;
    for (const auto& triggerQueue : taskConfig.trigger.queues) {
        triggerQueues.push_back(m_queues.at(triggerQueue).get());
    }
    return triggerQueues;
}

void EventPipelineGraph::BindInputNotifiers(const GraphConfig& config) {
    for (auto& runner : m_runners) {
        for (const auto& taskConfig : config.tasks) {
            if (taskConfig.name == runner->Name()) {
                BindTaskInputNotifiers(*runner, taskConfig);
                break;
            }
        }
    }
}

void EventPipelineGraph::BindTaskInputNotifiers(TaskRunner& runner,
                                                const TaskConfig& taskConfig) {
    for (const auto& input : taskConfig.inputs) {
        auto* inputQueue = m_queues.at(input.second).get();
        inputQueue->SetNotifier([runnerPtr = &runner]() {
            runnerPtr->Notify();
        });
    }
}

void EventPipelineGraph::Start() {
    if (!m_configured) {
        throw std::runtime_error("EventPipelineGraph must be configured before start");
    }
    if (m_running) {
        return;
    }
    m_running = true;
    for (auto& runner : m_runners) {
        runner->Start();
    }
}

void EventPipelineGraph::Stop() {
    if (!m_running) {
        return;
    }
    for (auto& runner : m_runners) {
        runner->Stop();
    }
    m_running = false;
}

void EventPipelineGraph::RequestStop() {
    if (!m_running) {
        return;
    }
    for (auto& runner : m_runners) {
        runner->RequestStop();
    }
}

bool EventPipelineGraph::JoinStopped() {
    if (!m_running) {
        return true;
    }
    for (auto& runner : m_runners) {
        if (!runner->JoinStopped()) {
            return false;
        }
    }
    m_running = false;
    return true;
}

bool EventPipelineGraph::Running() const {
    return m_running;
}

IQueue* EventPipelineGraph::Queue(const std::string& name) {
    auto it = m_queues.find(name);
    return it == m_queues.end() ? nullptr : it->second.get();
}

const IQueue* EventPipelineGraph::Queue(const std::string& name) const {
    auto it = m_queues.find(name);
    return it == m_queues.end() ? nullptr : it->second.get();
}

std::map<std::string, QueueDiagnosticsSnapshot> EventPipelineGraph::QueueDiagnostics() const {
    std::map<std::string, QueueDiagnosticsSnapshot> result;
    for (const auto& queue : m_queues) {
        result[queue.first] = queue.second->Diagnostics();
    }
    return result;
}

std::map<std::string, TaskDiagnosticsSnapshot> EventPipelineGraph::TaskDiagnostics() const {
    std::map<std::string, TaskDiagnosticsSnapshot> result;
    for (const auto& runner : m_runners) {
        result[runner->Name()] = runner->Diagnostics();
    }
    return result;
}

std::string EventPipelineGraph::DfxSnapshotJson(const std::string& graphName,
                                                std::uint64_t timestampMs) const {
    std::ostringstream out;
    out << "{\n";
    out << "  \"graph\": \"" << JsonEscape(graphName) << "\",\n";
    out << "  \"timestampMs\": " << timestampMs << ",\n";
    out << "  \"queues\": {\n";
    bool first = true;
    for (const auto& queue : m_queues) {
        const auto diag = queue.second->Diagnostics();
        if (!first) {
            out << ",\n";
        }
        first = false;
        out << "    \"" << JsonEscape(queue.first) << "\": {";
        WriteQueueDiagnosticsJson(out, *queue.second, diag);
        out << "}";
    }
    out << "\n  },\n";
    out << "  \"tasks\": {\n";
    first = true;
    for (const auto& runner : m_runners) {
        const auto diag = runner->Diagnostics();
        if (!first) {
            out << ",\n";
        }
        first = false;
        out << "    \"" << JsonEscape(runner->Name()) << "\": {";
        WriteTaskDiagnosticsJson(out, diag);
        out << "}";
    }
    out << "\n  }\n";
    out << "}\n";
    return out.str();
}

std::string EventPipelineGraph::ProfileJson(const std::string& graphName,
                                            std::uint64_t timestampMs,
                                            const std::string& topologyVersion,
                                            const std::string& taskCatalogJson) const
{
    std::ostringstream out;
    out << "{\n";
    out << "  \"schema\": \"" << GRAPH_PROFILE_SCHEMA << "\",\n";
    out << "  \"graph\": \"" << JsonEscape(graphName) << "\",\n";
    if (!topologyVersion.empty()) {
        out << "  \"topologyVersion\": \"" << JsonEscape(topologyVersion) << "\",\n";
    }
    out << "  \"timestampMs\": " << timestampMs << ",\n";
    if (!taskCatalogJson.empty()) {
        out << "  \"taskCatalog\": " << taskCatalogJson << ",\n";
    }
    out << "  \"topology\": {\n";
    out << "    \"queues\": [\n";
    for (std::size_t i = 0; i < m_config.queues.size(); ++i) {
        if (i != 0) {
            out << ",\n";
        }
        out << "      ";
        WriteQueueConfigJson(out, m_config.queues[i]);
    }
    out << "\n    ],\n";
    out << "    \"tasks\": [\n";
    for (std::size_t i = 0; i < m_config.tasks.size(); ++i) {
        if (i != 0) {
            out << ",\n";
        }
        out << "      ";
        WriteTaskConfigJson(out, m_config.tasks[i]);
    }
    out << "\n    ]\n";
    out << "  },\n";
    out << "  \"diagnostics\": ";
    out << DfxSnapshotJson(graphName, timestampMs);
    out << "}\n";
    return out.str();
}

} // namespace epg
