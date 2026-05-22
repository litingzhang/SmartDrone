#pragma once

#include "common/epg/epg_queue.h"

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Epg {

class Registry {
  public:
    using QueueFactory =
        std::function<std::unique_ptr<IQueue>(const QueueConfig &)>;
    using TaskFactory = std::function<std::unique_ptr<ITask>()>;

    struct QueueTypeInfo {
        std::string name;
        std::type_index type{typeid(void)};
        QueueFactory factory;
    };

    struct TaskTypeInfo {
        std::string name;
        std::vector<PortSpec> inputs;
        std::vector<PortSpec> outputs;
        TaskFactory factory;
    };

    template <class T>
    void RegisterMessageType(const std::string &name)
    {
        QueueTypeInfo info;
        info.name = name;
        info.type = std::type_index(typeid(T));
        info.factory = [name](const QueueConfig &config) {
            return std::unique_ptr<IQueue>(
                new SpscSharedPtrQueue<T>(config.name, name, config.depth,
                                          config.overflow));
        };
        m_queueTypes[name] = std::move(info);
    }

    template <class TTask>
    void RegisterTaskType(const std::string &name,
                          std::vector<PortSpec> inputs,
                          std::vector<PortSpec> outputs)
    {
        static_assert(std::is_base_of<ITask, TTask>::value,
                      "registered task must derive from ITask");
        TaskTypeInfo info;
        info.name = name;
        info.inputs = std::move(inputs);
        info.outputs = std::move(outputs);
        info.factory = []() { return std::unique_ptr<ITask>(new TTask()); };
        m_taskTypes[name] = std::move(info);
    }

    void RegisterTaskFactory(const std::string &name,
                             std::vector<PortSpec> inputs,
                             std::vector<PortSpec> outputs,
                             TaskFactory factory);
    void MergeTaskPorts(const std::string &name,
                        const std::vector<PortSpec> &inputs,
                        const std::vector<PortSpec> &outputs);

    const QueueTypeInfo *FindQueueType(const std::string &name) const;
    const TaskTypeInfo *FindTaskType(const std::string &name) const;

  private:
    std::unordered_map<std::string, QueueTypeInfo> m_queueTypes;
    std::unordered_map<std::string, TaskTypeInfo> m_taskTypes;
};

class TypeCatalog {
  public:
    using TaskFactoryResolver =
        std::function<Registry::TaskFactory(const std::string &)>;
    using TaskFactoryEntry = std::pair<std::string, Registry::TaskFactory>;

    static TypeCatalog &Global();

    template <class T>
    bool RegisterMessage(const std::string &name)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_messages[name] = [name](Registry &registry) {
            registry.RegisterMessageType<T>(name);
        };
        return true;
    }

    template <class TTask>
    bool RegisterTask(std::string name,
                      std::vector<PortSpec> inputs,
                      std::vector<PortSpec> outputs)
    {
        static_assert(std::is_base_of<ITask, TTask>::value,
                      "reflected task must derive from ITask");
        std::lock_guard<std::mutex> lock(m_mutex);
        TaskReflectionInfo info;
        info.name = std::move(name);
        info.inputs = std::move(inputs);
        info.outputs = std::move(outputs);
        const auto registeredName = info.name;
        m_tasks[info.name] = std::move(info);
        m_taskNamesByType[std::type_index(typeid(TTask))] = registeredName;
        return true;
    }

    template <class TTask>
    bool RegisterTaskType(std::string name)
    {
        static_assert(std::is_base_of<ITask, TTask>::value,
                      "reflected task must derive from ITask");
        return RegisterTask<TTask>(std::move(name), {}, {});
    }

    template <class TTask>
    std::string ReflectedTaskName() const
    {
        static_assert(std::is_base_of<ITask, TTask>::value,
                      "reflected task must derive from ITask");
        return ReflectedTaskName(std::type_index(typeid(TTask)));
    }

    std::string ReflectedTaskName(std::type_index taskType) const;

    template <class TTask, class Factory>
    TaskFactoryEntry MakeTaskFactoryEntry(Factory factory) const
    {
        static_assert(std::is_base_of<ITask, TTask>::value,
                      "reflected task must derive from ITask");
        return {ReflectedTaskName<TTask>(),
                Registry::TaskFactory(std::move(factory))};
    }

    static TaskFactoryResolver MakeTaskFactoryResolver(
        std::vector<TaskFactoryEntry> entries);

    void RegisterReflectedMessageTypes(Registry &registry) const;
    void RegisterReflectedTaskTypes(
        Registry &registry,
        const std::vector<std::string> &taskTypes,
        const TaskFactoryResolver &resolver) const;

  private:
    struct TaskReflectionInfo {
        std::string name;
        std::vector<PortSpec> inputs;
        std::vector<PortSpec> outputs;
    };

    mutable std::mutex m_mutex;
    std::unordered_map<std::string, std::function<void(Registry &)>>
        m_messages;
    std::unordered_map<std::string, TaskReflectionInfo> m_tasks;
    std::unordered_map<std::type_index, std::string> m_taskNamesByType;
};

} // namespace Epg
