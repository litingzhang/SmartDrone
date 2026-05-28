#pragma once

#include "common/epg/epg_queue.h"

#include <functional>
#include <memory>
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
        RegisterQueueType(MakeQueueTypeInfo(
            name, std::type_index(typeid(T)), [name](const QueueConfig &config) {
                return std::unique_ptr<IQueue>(
                    new SpscSharedPtrQueue<T>(config.name, name, config.depth,
                                              config.overflow));
            }));
    }

    template <class TTask>
    void RegisterTaskType(const std::string &name,
                          std::vector<PortSpec> inputs,
                          std::vector<PortSpec> outputs)
    {
        static_assert(IsTaskType<TTask>(),
                      "registered task must derive from ITask");
        RegisterTaskTypeInfo(MakeTaskTypeInfo(
            name, std::move(inputs), std::move(outputs),
            []() { return std::unique_ptr<ITask>(new TTask()); }));
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
    static QueueTypeInfo MakeQueueTypeInfo(const std::string &name,
                                           std::type_index type,
                                           QueueFactory factory);
    static TaskTypeInfo MakeTaskTypeInfo(const std::string &name,
                                         std::vector<PortSpec> inputs,
                                         std::vector<PortSpec> outputs,
                                         TaskFactory factory);
    void RegisterQueueType(QueueTypeInfo info);
    void RegisterTaskTypeInfo(TaskTypeInfo info);
    template <class TTask>
    static constexpr bool IsTaskType()
    {
        return std::is_base_of<ITask, TTask>::value;
    }

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
        return RegisterMessageFactory(name, [name](Registry &registry) {
            registry.RegisterMessageType<T>(name);
        });
    }

    template <class TTask>
    bool RegisterTask(std::string name,
                      std::vector<PortSpec> inputs,
                      std::vector<PortSpec> outputs)
    {
        static_assert(IsTaskType<TTask>(),
                      "reflected task must derive from ITask");
        return RegisterTaskReflection(std::type_index(typeid(TTask)),
                                      MakeTaskReflectionInfo(
                                          std::move(name), std::move(inputs),
                                          std::move(outputs)));
    }

    template <class TTask>
    bool RegisterTaskType(std::string name)
    {
        static_assert(IsTaskType<TTask>(),
                      "reflected task must derive from ITask");
        return RegisterTask<TTask>(std::move(name), {}, {});
    }

    template <class TTask>
    std::string ReflectedTaskName() const
    {
        static_assert(IsTaskType<TTask>(),
                      "reflected task must derive from ITask");
        return ReflectedTaskName(std::type_index(typeid(TTask)));
    }

    std::string ReflectedTaskName(std::type_index taskType) const;

    template <class TTask, class Factory>
    TaskFactoryEntry MakeTaskFactoryEntry(Factory factory) const
    {
        static_assert(IsTaskType<TTask>(),
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

    struct CatalogSnapshot {
        std::unordered_map<std::string, std::function<void(Registry &)>>
            messages;
        std::unordered_map<std::string, TaskReflectionInfo> tasks;
        std::unordered_map<std::type_index, std::string> taskNamesByType;
    };

    TypeCatalog();
    static TaskReflectionInfo MakeTaskReflectionInfo(
        std::string name, std::vector<PortSpec> inputs,
        std::vector<PortSpec> outputs);
    bool RegisterMessageFactory(
        const std::string &name, std::function<void(Registry &)> factory);
    bool RegisterTaskReflection(std::type_index taskType,
                                TaskReflectionInfo info);
    std::shared_ptr<const CatalogSnapshot> LoadSnapshot() const;
    bool ReplaceSnapshot(std::shared_ptr<const CatalogSnapshot> &expected,
                         std::shared_ptr<const CatalogSnapshot> next);
    template <class TTask>
    static constexpr bool IsTaskType()
    {
        return std::is_base_of<ITask, TTask>::value;
    }

    std::shared_ptr<const CatalogSnapshot> m_snapshot;
};

} // namespace Epg
