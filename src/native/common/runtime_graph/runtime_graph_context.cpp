#include "common/runtime_graph/runtime_graph.h"

namespace smartdrone {
namespace runtime_graph {

TaskContext::TaskContext(std::unordered_map<std::string, IQueue*> inputs,
                         std::unordered_map<std::string, IQueue*> outputs)
    : m_inputs(std::move(inputs)), m_outputs(std::move(outputs)) {
}

bool TaskContext::InputReady(const std::string& slot) const {
    auto it = m_inputs.find(slot);
    if (it == m_inputs.end()) {
        throw std::runtime_error("missing input slot: " + slot);
    }
    return !it->second->Empty();
}

bool TaskContext::OutputExists(const std::string& slot) const {
    return m_outputs.find(slot) != m_outputs.end();
}

void Registry::RegisterTaskFactory(const std::string& name,
                                   std::vector<PortSpec> inputs,
                                   std::vector<PortSpec> outputs,
                                   TaskFactory factory) {
    if (!factory) {
        throw std::runtime_error("registered task factory must be callable: " + name);
    }

    TaskTypeInfo info;
    info.name = name;
    info.inputs = std::move(inputs);
    info.outputs = std::move(outputs);
    info.factory = std::move(factory);
    m_taskTypes[name] = std::move(info);
}

const Registry::QueueTypeInfo* Registry::FindQueueType(const std::string& name) const {
    auto it = m_queueTypes.find(name);
    return it == m_queueTypes.end() ? nullptr : &it->second;
}

const Registry::TaskTypeInfo* Registry::FindTaskType(const std::string& name) const {
    auto it = m_taskTypes.find(name);
    return it == m_taskTypes.end() ? nullptr : &it->second;
}

RuntimeGraphTypeCatalog& RuntimeGraphTypeCatalog::Global() {
    static RuntimeGraphTypeCatalog catalog;
    return catalog;
}

std::string RuntimeGraphTypeCatalog::ReflectedTaskName(std::type_index taskType) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    const auto it = m_taskNamesByType.find(taskType);
    if (it == m_taskNamesByType.end()) {
        throw std::runtime_error("missing reflected runtime graph task type");
    }
    return it->second;
}

RuntimeGraphTypeCatalog::TaskFactoryResolver RuntimeGraphTypeCatalog::MakeTaskFactoryResolver(
    std::vector<TaskFactoryEntry> entries) {
    std::unordered_map<std::string, Registry::TaskFactory> factories;
    factories.reserve(entries.size());
    for (auto& entry : entries) {
        factories.emplace(std::move(entry.first), std::move(entry.second));
    }

    return [factories = std::move(factories)](const std::string& type) {
        const auto it = factories.find(type);
        if (it == factories.end()) {
            return Registry::TaskFactory{};
        }
        return it->second;
    };
}

void RuntimeGraphTypeCatalog::RegisterReflectedMessageTypes(Registry& registry) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (const auto& message : m_messages) {
        message.second(registry);
    }
}

void RuntimeGraphTypeCatalog::RegisterReflectedTaskTypes(
    Registry& registry,
    const std::vector<std::string>& taskTypes,
    const TaskFactoryResolver& resolver) const {
    if (!resolver) {
        throw std::runtime_error("runtime graph task factory resolver must be callable");
    }

    std::vector<TaskReflectionInfo> reflectedTasks;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        reflectedTasks.reserve(taskTypes.size());
        for (const auto& taskType : taskTypes) {
            const auto it = m_tasks.find(taskType);
            if (it == m_tasks.end()) {
                throw std::runtime_error("missing reflected runtime graph task type: " + taskType);
            }
            reflectedTasks.push_back(it->second);
        }
    }

    for (const auto& task : reflectedTasks) {
        auto factory = resolver(task.name);
        if (!factory) {
            throw std::runtime_error("missing runtime graph task factory: " + task.name);
        }
        registry.RegisterTaskFactory(task.name, task.inputs, task.outputs, std::move(factory));
    }
}

} // namespace runtime_graph
} // namespace smartdrone
