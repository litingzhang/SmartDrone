#include "common/epg/epg.h"

#include <algorithm>

namespace Epg {
namespace {

void UpdateMaxAtomic(std::atomic<std::uint64_t> &value,
                     std::uint64_t observed)
{
    auto current = value.load(std::memory_order_relaxed);
    while (observed > current &&
           !value.compare_exchange_weak(current, observed,
                                        std::memory_order_relaxed,
                                        std::memory_order_relaxed)) {
    }
}

} // namespace

TaskContext::TaskContext(std::unordered_map<PortId, IQueue *> inputs,
                         std::unordered_map<PortId, IQueue *> outputs)
    : m_inputs(std::move(inputs)), m_outputs(std::move(outputs))
{
}

void ValidateSpscQueueDepth(std::size_t queueDepth)
{
    if (queueDepth != 0) {
        return;
    }
    throw std::invalid_argument("SPSC queue depth must be greater than zero");
}

void FillQueueDiagnosticsSnapshot(const QueueDiagnostics &diag,
                                  QueueDiagnosticsSnapshot &snapshot)
{
    snapshot.pushed = diag.pushed.load(std::memory_order_relaxed);
    snapshot.popped = diag.popped.load(std::memory_order_relaxed);
    snapshot.droppedNewest = diag.droppedNewest.load(std::memory_order_relaxed);
    snapshot.overwrittenOldest =
        diag.overwrittenOldest.load(std::memory_order_relaxed);
    snapshot.wakeups = diag.wakeups.load(std::memory_order_relaxed);
    snapshot.maxDepthObserved =
        diag.maxDepthObserved.load(std::memory_order_relaxed);
    snapshot.firstActivityMs =
        diag.firstActivityMs.load(std::memory_order_relaxed);
    snapshot.lastActivityMs =
        diag.lastActivityMs.load(std::memory_order_relaxed);
}

bool TaskContext::InputReady(PortId port) const
{
    auto it = m_inputs.find(port);
    if (it == m_inputs.end()) {
        throw std::runtime_error("missing input port: " + std::to_string(port));
    }
    return !it->second->Empty();
}

bool TaskContext::InputExists(PortId port) const
{
    return m_inputs.find(port) != m_inputs.end();
}

std::size_t TaskContext::InputSize(PortId port) const
{
    auto it = m_inputs.find(port);
    if (it == m_inputs.end()) {
        throw std::runtime_error("missing input port: " + std::to_string(port));
    }
    return it->second->Size();
}

bool TaskContext::OutputExists(PortId port) const
{
    return m_outputs.find(port) != m_outputs.end();
}

std::size_t TaskContext::OutputSize(PortId port) const
{
    auto it = m_outputs.find(port);
    if (it == m_outputs.end()) {
        throw std::runtime_error("missing output port: " + std::to_string(port));
    }
    return it->second->Size();
}

const IQueue *TaskContext::OutputQueueByPort(PortId port) const
{
    const auto it = m_outputs.find(port);
    return it == m_outputs.end() ? nullptr : it->second;
}

IQueue *TaskContext::InputQueueByPort(PortId port) const
{
    auto it = m_inputs.find(port);
    if (it == m_inputs.end()) {
        throw std::runtime_error("missing input port: " + std::to_string(port));
    }
    return it->second;
}

IQueue *TaskContext::OutputQueueByPortMutable(PortId port) const
{
    auto it = m_outputs.find(port);
    if (it == m_outputs.end()) {
        throw std::runtime_error("missing output port: " +
                                 std::to_string(port));
    }
    return it->second;
}

void TaskContext::ValidateQueueType(IQueue *queue, PortId port,
                                    std::type_index expectedType,
                                    const char *direction)
{
    if (queue->TypeIndex() == expectedType) {
        return;
    }
    throw std::runtime_error(std::string(direction) +
                             " port type mismatch: " +
                             std::to_string(port));
}

void TaskContext::AttachDiagnostics(TaskDiagnostics *diagnostics)
{
    m_diagnostics = diagnostics;
}

void TaskContext::ReportResourceWait(std::uint64_t waitUs)
{
    if (!m_diagnostics || waitUs == 0) {
        return;
    }
    m_diagnostics->resourceWaitCount.fetch_add(1, std::memory_order_relaxed);
    m_diagnostics->lastResourceWaitUs.store(waitUs, std::memory_order_relaxed);
    m_diagnostics->totalResourceWaitUs.fetch_add(waitUs,
                                                 std::memory_order_relaxed);
    UpdateMaxAtomic(m_diagnostics->maxResourceWaitUs, waitUs);
}

void Registry::RegisterTaskFactory(const std::string &name,
                                   std::vector<PortSpec> inputs,
                                   std::vector<PortSpec> outputs,
                                   TaskFactory factory)
{
    if (!factory) {
        throw std::runtime_error(
            "registered task factory must be callable: " + name);
    }

    TaskTypeInfo info;
    info.name = name;
    info.inputs = std::move(inputs);
    info.outputs = std::move(outputs);
    info.factory = std::move(factory);
    m_taskTypes[name] = std::move(info);
}

Registry::QueueTypeInfo Registry::MakeQueueTypeInfo(
    const std::string &name, std::type_index type, QueueFactory factory)
{
    QueueTypeInfo info;
    info.name = name;
    info.type = type;
    info.factory = std::move(factory);
    return info;
}

Registry::TaskTypeInfo Registry::MakeTaskTypeInfo(
    const std::string &name, std::vector<PortSpec> inputs,
    std::vector<PortSpec> outputs, TaskFactory factory)
{
    TaskTypeInfo info;
    info.name = name;
    info.inputs = std::move(inputs);
    info.outputs = std::move(outputs);
    info.factory = std::move(factory);
    return info;
}

void Registry::RegisterQueueType(QueueTypeInfo info)
{
    m_queueTypes[info.name] = std::move(info);
}

void Registry::RegisterTaskTypeInfo(TaskTypeInfo info)
{
    m_taskTypes[info.name] = std::move(info);
}

namespace {

void MergePortSpecs(std::vector<PortSpec> &existing,
                    const std::vector<PortSpec> &incoming,
                    const std::string &taskName,
                    const char *direction)
{
    for (const auto &port : incoming) {
        auto it = std::find_if(
            existing.begin(), existing.end(),
            [&port](const PortSpec &current) {
                return current.id == port.id;
            });
        if (it == existing.end()) {
            existing.push_back(port);
            continue;
        }
        if (it->type != port.type) {
            throw std::runtime_error(
                "EventPipelineGraph task " + std::string(direction) +
                " port type mismatch: " + taskName + "." +
                std::to_string(port.id));
        }
    }
}

} // namespace

void Registry::MergeTaskPorts(const std::string &name,
                              const std::vector<PortSpec> &inputs,
                              const std::vector<PortSpec> &outputs)
{
    auto it = m_taskTypes.find(name);
    if (it == m_taskTypes.end()) {
        throw std::runtime_error(
            "cannot merge ports into unregistered task type: " + name);
    }
    MergePortSpecs(it->second.inputs, inputs, name, "input");
    MergePortSpecs(it->second.outputs, outputs, name, "output");
}

const Registry::QueueTypeInfo *Registry::FindQueueType(const std::string &name) const
{
    auto it = m_queueTypes.find(name);
    return it == m_queueTypes.end() ? nullptr : &it->second;
}

const Registry::TaskTypeInfo *Registry::FindTaskType(const std::string &name) const
{
    auto it = m_taskTypes.find(name);
    return it == m_taskTypes.end() ? nullptr : &it->second;
}

TypeCatalog &TypeCatalog::Global()
{
    static TypeCatalog catalog;
    return catalog;
}

TypeCatalog::TypeCatalog()
{
    std::atomic_store_explicit(&m_snapshot,
                               std::make_shared<const CatalogSnapshot>(),
                               std::memory_order_release);
}

TypeCatalog::TaskReflectionInfo TypeCatalog::MakeTaskReflectionInfo(
    std::string name, std::vector<PortSpec> inputs,
    std::vector<PortSpec> outputs)
{
    TaskReflectionInfo info;
    info.name = std::move(name);
    info.inputs = std::move(inputs);
    info.outputs = std::move(outputs);
    return info;
}

bool TypeCatalog::RegisterMessageFactory(
    const std::string &name, std::function<void(Registry &)> factory)
{
    std::shared_ptr<const CatalogSnapshot> current = LoadSnapshot();
    while (current) {
        auto next = std::make_shared<CatalogSnapshot>(*current);
        next->messages[name] = factory;
        if (ReplaceSnapshot(current, std::move(next))) {
            return true;
        }
    }
    return false;
}

bool TypeCatalog::RegisterTaskReflection(std::type_index taskType,
                                         TaskReflectionInfo info)
{
    const auto registeredName = info.name;
    std::shared_ptr<const CatalogSnapshot> current = LoadSnapshot();
    while (current) {
        auto next = std::make_shared<CatalogSnapshot>(*current);
        next->tasks[info.name] = info;
        next->taskNamesByType[taskType] = registeredName;
        if (ReplaceSnapshot(current, std::move(next))) {
            return true;
        }
    }
    return false;
}

std::string TypeCatalog::ReflectedTaskName(std::type_index taskType) const
{
    std::shared_ptr<const CatalogSnapshot> snapshot = LoadSnapshot();
    const auto it = snapshot->taskNamesByType.find(taskType);
    if (it == snapshot->taskNamesByType.end()) {
        throw std::runtime_error("missing reflected EventPipelineGraph task type");
    }
    return it->second;
}

TypeCatalog::TaskFactoryResolver TypeCatalog::MakeTaskFactoryResolver(
    std::vector<TaskFactoryEntry> entries)
{
    std::unordered_map<std::string, Registry::TaskFactory> factories;
    factories.reserve(entries.size());
    for (auto &entry : entries) {
        factories.emplace(std::move(entry.first), std::move(entry.second));
    }

    return [factories = std::move(factories)](const std::string &type) {
        const auto it = factories.find(type);
        if (it == factories.end()) {
            return Registry::TaskFactory{};
        }
        return it->second;
    };
}

void TypeCatalog::RegisterReflectedMessageTypes(Registry &registry) const
{
    std::shared_ptr<const CatalogSnapshot> snapshot = LoadSnapshot();
    for (const auto &message : snapshot->messages) {
        message.second(registry);
    }
}

void TypeCatalog::RegisterReflectedTaskTypes(
    Registry &registry,
    const std::vector<std::string> &taskTypes,
    const TaskFactoryResolver &resolver) const
{
    if (!resolver) {
        throw std::runtime_error("EventPipelineGraph task factory resolver must be callable");
    }

    std::shared_ptr<const CatalogSnapshot> snapshot = LoadSnapshot();
    std::vector<TaskReflectionInfo> reflectedTasks;
    reflectedTasks.reserve(taskTypes.size());
    for (const auto &taskType : taskTypes) {
        const auto it = snapshot->tasks.find(taskType);
        if (it == snapshot->tasks.end()) {
            throw std::runtime_error("missing reflected EventPipelineGraph task type: " + taskType);
        }
        reflectedTasks.push_back(it->second);
    }

    for (const auto &task : reflectedTasks) {
        auto factory = resolver(task.name);
        if (!factory) {
            throw std::runtime_error("missing EventPipelineGraph task factory: " + task.name);
        }
        registry.RegisterTaskFactory(task.name, task.inputs, task.outputs, std::move(factory));
    }
}

std::shared_ptr<const TypeCatalog::CatalogSnapshot>
TypeCatalog::LoadSnapshot() const
{
    return std::atomic_load_explicit(&m_snapshot, std::memory_order_acquire);
}

bool TypeCatalog::ReplaceSnapshot(
    std::shared_ptr<const CatalogSnapshot> &expected,
    std::shared_ptr<const CatalogSnapshot> next)
{
    return std::atomic_compare_exchange_weak_explicit(&m_snapshot, &expected,
                                                      std::move(next),
                                                      std::memory_order_acq_rel,
                                                      std::memory_order_acquire);
}

} // namespace Epg
