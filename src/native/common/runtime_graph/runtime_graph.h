#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <set>
#include <string>
#include <thread>
#include <type_traits>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

namespace smartdrone {
namespace runtime_graph {

enum class OverflowPolicy {
    DropNewest,
    OverwriteOldest
};

enum class TriggerMode {
    Periodic,
    AnyQueueReady,
    AllQueueReady,
    PeriodicOrAnyQueueReady
};

struct QueueDiagnostics {
    std::atomic<std::uint64_t> pushed{0};
    std::atomic<std::uint64_t> popped{0};
    std::atomic<std::uint64_t> droppedNewest{0};
    std::atomic<std::uint64_t> overwrittenOldest{0};
    std::atomic<std::uint64_t> wakeups{0};
    std::atomic<std::uint64_t> maxDepthObserved{0};
};

struct QueueDiagnosticsSnapshot {
    std::uint64_t pushed{};
    std::uint64_t popped{};
    std::uint64_t droppedNewest{};
    std::uint64_t overwrittenOldest{};
    std::uint64_t wakeups{};
    std::uint64_t maxDepthObserved{};
};

struct TaskDiagnostics {
    std::atomic<std::uint64_t> loopCount{0};
    std::atomic<std::uint64_t> errorCount{0};
    std::atomic<std::uint64_t> idleWakeups{0};
    std::atomic<std::uint64_t> lastLoopUs{0};
    std::atomic<std::uint64_t> maxLoopUs{0};
};

struct TaskDiagnosticsSnapshot {
    std::uint64_t loopCount{};
    std::uint64_t errorCount{};
    std::uint64_t idleWakeups{};
    std::uint64_t lastLoopUs{};
    std::uint64_t maxLoopUs{};
};

struct QueueConfig {
    std::string name;
    std::string type;
    std::size_t depth{};
    OverflowPolicy overflow{OverflowPolicy::DropNewest};
};

struct TriggerConfig {
    TriggerMode mode{TriggerMode::Periodic};
    std::chrono::milliseconds interval{0};
    std::vector<std::string> queues;
};

struct TaskConfig {
    std::string name;
    std::string type;
    TriggerConfig trigger;
    std::map<std::string, std::string> inputs;
    std::map<std::string, std::string> outputs;
};

struct RuntimeGraphConfig {
    std::vector<QueueConfig> queues;
    std::vector<TaskConfig> tasks;
};

class Registry;

RuntimeGraphConfig ParseRuntimeGraphConfigJson(const std::string& jsonText);
RuntimeGraphConfig ParseRuntimeGraphConfigJsonFile(const std::string& path);
RuntimeGraphConfig ParseRuntimeGraphConfigMermaid(const std::string& mermaidText);
RuntimeGraphConfig ParseRuntimeGraphConfigMermaidFile(const std::string& path);
RuntimeGraphConfig ParseRuntimeGraphConfigMermaid(const std::string& mermaidText,
                                                  const Registry& registry);
RuntimeGraphConfig ParseRuntimeGraphConfigMermaidFile(const std::string& path,
                                                      const Registry& registry);
RuntimeGraphConfig ParseRuntimeGraphConfigMermaidSubgraph(const std::string& mermaidText,
                                                          const std::string& subgraphName,
                                                          const Registry& registry);
RuntimeGraphConfig ParseRuntimeGraphConfigMermaidSubgraphFile(const std::string& path,
                                                              const std::string& subgraphName,
                                                              const Registry& registry);

struct PortSpec {
    std::string name;
    std::string type;
};

class IQueue {
public:
    virtual ~IQueue() = default;
    virtual const std::string& Name() const = 0;
    virtual const std::string& TypeName() const = 0;
    virtual std::type_index TypeIndex() const = 0;
    virtual std::size_t Depth() const = 0;
    virtual std::size_t Size() const = 0;
    virtual bool Empty() const = 0;
    virtual bool PushErased(std::shared_ptr<void> item) = 0;
    virtual std::shared_ptr<void> TryPopErased() = 0;
    virtual std::shared_ptr<void> TryPopLatestErased() = 0;
    virtual QueueDiagnosticsSnapshot Diagnostics() const = 0;
    virtual void SetNotifier(std::function<void()> notifier) = 0;
};

template <class T>
class SpscSharedPtrQueue final : public IQueue {
public:
    SpscSharedPtrQueue(std::string queueName,
                       std::string queueTypeName,
                       std::size_t queueDepth,
                       OverflowPolicy overflow)
        : m_name(std::move(queueName)),
          m_typeName(std::move(queueTypeName)),
          m_depth(queueDepth),
          m_capacity(queueDepth + 1),
          m_overflow(overflow),
          m_slots(m_capacity) {
        if (queueDepth == 0) {
            throw std::invalid_argument("SPSC queue depth must be greater than zero");
        }
    }

    const std::string& Name() const override { return m_name; }
    const std::string& TypeName() const override { return m_typeName; }
    std::type_index TypeIndex() const override { return std::type_index(typeid(T)); }
    std::size_t Depth() const override { return m_depth; }

    std::size_t Size() const override {
        const auto head = m_head.load(std::memory_order_acquire);
        const auto tail = m_tail.load(std::memory_order_acquire);
        return head >= tail ? head - tail : m_capacity - tail + head;
    }

    bool Empty() const override {
        return m_head.load(std::memory_order_acquire) == m_tail.load(std::memory_order_acquire);
    }

    bool Push(std::shared_ptr<T> item) {
        auto head = m_head.load(std::memory_order_relaxed);
        auto tail = m_tail.load(std::memory_order_acquire);
        auto next = Increment(head);

        if (next == tail) {
            if (m_overflow == OverflowPolicy::DropNewest) {
                m_diag.droppedNewest.fetch_add(1, std::memory_order_relaxed);
                return false;
            }

            while (next == tail) {
                auto tailNext = Increment(tail);
                if (m_tail.compare_exchange_weak(tail, tailNext,
                                                 std::memory_order_acq_rel,
                                                 std::memory_order_acquire)) {
                    m_diag.overwrittenOldest.fetch_add(1, std::memory_order_relaxed);
                    break;
                }
            }
        }

        m_slots[head] = std::move(item);
        m_head.store(next, std::memory_order_release);
        m_diag.pushed.fetch_add(1, std::memory_order_relaxed);
        UpdateMaxDepth(Size());
        Notify();
        return true;
    }

    std::shared_ptr<T> TryPop() {
        for (;;) {
            auto tail = m_tail.load(std::memory_order_acquire);
            const auto head = m_head.load(std::memory_order_acquire);
            if (tail == head) {
                return {};
            }

            const auto next = Increment(tail);
            if (m_tail.compare_exchange_weak(tail, next,
                                             std::memory_order_acq_rel,
                                             std::memory_order_acquire)) {
                auto item = std::move(m_slots[tail]);
                m_slots[tail].reset();
                m_diag.popped.fetch_add(1, std::memory_order_relaxed);
                return item;
            }
        }
    }

    std::shared_ptr<T> TryPopLatest() {
        std::shared_ptr<T> latest;
        while (auto item = TryPop()) {
            latest = std::move(item);
        }
        return latest;
    }

    bool PushErased(std::shared_ptr<void> item) override {
        return Push(std::static_pointer_cast<T>(std::move(item)));
    }

    std::shared_ptr<void> TryPopErased() override {
        return std::static_pointer_cast<void>(TryPop());
    }

    std::shared_ptr<void> TryPopLatestErased() override {
        return std::static_pointer_cast<void>(TryPopLatest());
    }

    QueueDiagnosticsSnapshot Diagnostics() const override {
        QueueDiagnosticsSnapshot snapshot;
        snapshot.pushed = m_diag.pushed.load(std::memory_order_relaxed);
        snapshot.popped = m_diag.popped.load(std::memory_order_relaxed);
        snapshot.droppedNewest = m_diag.droppedNewest.load(std::memory_order_relaxed);
        snapshot.overwrittenOldest = m_diag.overwrittenOldest.load(std::memory_order_relaxed);
        snapshot.wakeups = m_diag.wakeups.load(std::memory_order_relaxed);
        snapshot.maxDepthObserved = m_diag.maxDepthObserved.load(std::memory_order_relaxed);
        return snapshot;
    }

    void SetNotifier(std::function<void()> notifier) override {
        std::lock_guard<std::mutex> lock(m_notifierMutex);
        m_notifier = std::move(notifier);
    }

private:
    std::size_t Increment(std::size_t value) const {
        return (value + 1) % m_capacity;
    }

    void UpdateMaxDepth(std::size_t observed) {
        auto current = m_diag.maxDepthObserved.load(std::memory_order_relaxed);
        while (observed > current &&
               !m_diag.maxDepthObserved.compare_exchange_weak(current, observed,
                                                              std::memory_order_relaxed,
                                                              std::memory_order_relaxed)) {
        }
    }

    void Notify() {
        std::function<void()> notifier;
        {
            std::lock_guard<std::mutex> lock(m_notifierMutex);
            notifier = m_notifier;
        }
        if (notifier) {
            m_diag.wakeups.fetch_add(1, std::memory_order_relaxed);
            notifier();
        }
    }

    std::string m_name;
    std::string m_typeName;
    std::size_t m_depth{};
    std::size_t m_capacity{};
    OverflowPolicy m_overflow{OverflowPolicy::DropNewest};
    std::vector<std::shared_ptr<T>> m_slots;
    mutable QueueDiagnostics m_diag;
    std::atomic<std::size_t> m_head{0};
    std::atomic<std::size_t> m_tail{0};
    mutable std::mutex m_notifierMutex;
    std::function<void()> m_notifier;
};

class TaskContext {
public:
    TaskContext(std::unordered_map<std::string, IQueue*> inputs,
                std::unordered_map<std::string, IQueue*> outputs);

    template <class T, class... Args>
    std::shared_ptr<T> Make(Args&&... args) {
        return std::make_shared<T>(std::forward<Args>(args)...);
    }

    template <class T>
    std::shared_ptr<T> TryPop(const std::string& slot) {
        auto* queue = InputQueue<T>(slot);
        return std::static_pointer_cast<T>(queue->TryPopErased());
    }

    template <class T>
    std::shared_ptr<T> TryPopLatest(const std::string& slot) {
        auto* queue = InputQueue<T>(slot);
        return std::static_pointer_cast<T>(queue->TryPopLatestErased());
    }

    template <class T>
    bool Push(const std::string& slot, std::shared_ptr<T> item) {
        auto* queue = OutputQueue<T>(slot);
        return queue->PushErased(std::static_pointer_cast<void>(std::move(item)));
    }

    bool InputReady(const std::string& slot) const;
    bool OutputExists(const std::string& slot) const;

private:
    template <class T>
    IQueue* InputQueue(const std::string& slot) {
        auto it = m_inputs.find(slot);
        if (it == m_inputs.end()) {
            throw std::runtime_error("missing input slot: " + slot);
        }
        if (it->second->TypeIndex() != std::type_index(typeid(T))) {
            throw std::runtime_error("input slot type mismatch: " + slot);
        }
        return it->second;
    }

    template <class T>
    IQueue* OutputQueue(const std::string& slot) {
        auto it = m_outputs.find(slot);
        if (it == m_outputs.end()) {
            throw std::runtime_error("missing output slot: " + slot);
        }
        if (it->second->TypeIndex() != std::type_index(typeid(T))) {
            throw std::runtime_error("output slot type mismatch: " + slot);
        }
        return it->second;
    }

    std::unordered_map<std::string, IQueue*> m_inputs;
    std::unordered_map<std::string, IQueue*> m_outputs;
};

class ITask {
public:
    virtual ~ITask() = default;
    virtual void OnTick(TaskContext& context) = 0;
};

class Registry {
public:
    using QueueFactory = std::function<std::unique_ptr<IQueue>(const QueueConfig&)>;
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
    void RegisterMessageType(const std::string& name) {
        QueueTypeInfo info;
        info.name = name;
        info.type = std::type_index(typeid(T));
        info.factory = [name](const QueueConfig& config) {
            return std::unique_ptr<IQueue>(
                new SpscSharedPtrQueue<T>(config.name, name, config.depth, config.overflow));
        };
        m_queueTypes[name] = std::move(info);
    }

    template <class TTask>
    void RegisterTaskType(const std::string& name,
                          std::vector<PortSpec> inputs,
                          std::vector<PortSpec> outputs) {
        static_assert(std::is_base_of<ITask, TTask>::value,
                      "registered task must derive from ITask");
        TaskTypeInfo info;
        info.name = name;
        info.inputs = std::move(inputs);
        info.outputs = std::move(outputs);
        info.factory = []() { return std::unique_ptr<ITask>(new TTask()); };
        m_taskTypes[name] = std::move(info);
    }

    void RegisterTaskFactory(const std::string& name,
                             std::vector<PortSpec> inputs,
                             std::vector<PortSpec> outputs,
                             TaskFactory factory);

    const QueueTypeInfo* FindQueueType(const std::string& name) const;
    const TaskTypeInfo* FindTaskType(const std::string& name) const;

private:
    std::unordered_map<std::string, QueueTypeInfo> m_queueTypes;
    std::unordered_map<std::string, TaskTypeInfo> m_taskTypes;
};

class RuntimeGraphTypeCatalog {
public:
    using TaskFactoryResolver = std::function<Registry::TaskFactory(const std::string&)>;
    using TaskFactoryEntry = std::pair<std::string, Registry::TaskFactory>;

    static RuntimeGraphTypeCatalog& Global();

    template <class T>
    bool RegisterMessage(const std::string& name) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_messages[name] = [name](Registry& registry) {
            registry.RegisterMessageType<T>(name);
        };
        return true;
    }

    template <class TTask>
    bool RegisterTask(std::string name,
                      std::vector<PortSpec> inputs,
                      std::vector<PortSpec> outputs) {
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
    std::string ReflectedTaskName() const {
        static_assert(std::is_base_of<ITask, TTask>::value,
                      "reflected task must derive from ITask");
        return ReflectedTaskName(std::type_index(typeid(TTask)));
    }

    std::string ReflectedTaskName(std::type_index taskType) const;

    template <class TTask, class Factory>
    TaskFactoryEntry MakeTaskFactoryEntry(Factory factory) const {
        static_assert(std::is_base_of<ITask, TTask>::value,
                      "reflected task must derive from ITask");
        return {ReflectedTaskName<TTask>(), Registry::TaskFactory(std::move(factory))};
    }

    static TaskFactoryResolver MakeTaskFactoryResolver(std::vector<TaskFactoryEntry> entries);

    void RegisterReflectedMessageTypes(Registry& registry) const;
    void RegisterReflectedTaskTypes(Registry& registry,
                                    const std::vector<std::string>& taskTypes,
                                    const TaskFactoryResolver& resolver) const;

private:
    struct TaskReflectionInfo {
        std::string name;
        std::vector<PortSpec> inputs;
        std::vector<PortSpec> outputs;
    };

    mutable std::mutex m_mutex;
    std::unordered_map<std::string, std::function<void(Registry&)>> m_messages;
    std::unordered_map<std::string, TaskReflectionInfo> m_tasks;
    std::unordered_map<std::type_index, std::string> m_taskNamesByType;
};

#define SMARTDRONE_RUNTIME_GRAPH_PORT(portName, portType) \
    smartdrone::runtime_graph::PortSpec{portName, portType}

#define SMARTDRONE_RUNTIME_GRAPH_REGISTER_MESSAGE(messageType, messageName) \
    namespace { \
    const bool kRuntimeGraphMessageRegistration_##messageType = \
        smartdrone::runtime_graph::RuntimeGraphTypeCatalog::Global().RegisterMessage<messageType>(messageName); \
    }

#define SMARTDRONE_RUNTIME_GRAPH_REGISTER_TASK(taskType, taskName, ...) \
    namespace { \
    const bool kRuntimeGraphTaskRegistration_##taskType = \
        smartdrone::runtime_graph::RuntimeGraphTypeCatalog::Global().RegisterTask<taskType>( \
            taskName, __VA_ARGS__); \
    }

class RuntimeGraph {
public:
    template <class T>
    class ExternalIngress {
    public:
        ExternalIngress() = default;

        bool Valid() const { return m_queue != nullptr; }
        const std::string& QueueName() const
        {
            static const std::string kEmpty;
            return m_queue ? m_queue->Name() : kEmpty;
        }

        bool TryPush(std::shared_ptr<T> item) const
        {
            if (!m_queue) {
                throw std::runtime_error("runtime graph ingress is not bound");
            }
            return m_queue->PushErased(std::static_pointer_cast<void>(std::move(item)));
        }

        template <class... Args>
        bool Emplace(Args&&... args) const
        {
            return TryPush(std::make_shared<T>(std::forward<Args>(args)...));
        }

    private:
        friend class RuntimeGraph;
        explicit ExternalIngress(IQueue* queue) : m_queue(queue) {}

        IQueue* m_queue{nullptr};
    };

    explicit RuntimeGraph(const Registry& registry);
    ~RuntimeGraph();

    RuntimeGraph(const RuntimeGraph&) = delete;
    RuntimeGraph& operator=(const RuntimeGraph&) = delete;

    void Configure(const RuntimeGraphConfig& config);
    void ConfigureJson(const std::string& jsonText);
    void Start();
    void Stop();
    bool Running() const;

    template <class T>
    ExternalIngress<T> CreateExternalIngress(const std::string& queueName)
    {
        auto it = m_queues.find(queueName);
        if (it == m_queues.end()) {
            throw std::runtime_error("runtime graph ingress queue not found: " + queueName);
        }
        IQueue* queue = it->second.get();
        if (queue->TypeIndex() != std::type_index(typeid(T))) {
            throw std::runtime_error("runtime graph ingress queue type mismatch: " + queueName);
        }
        if (m_taskProducedQueues.find(queueName) != m_taskProducedQueues.end()) {
            throw std::runtime_error("runtime graph ingress queue already has task producer: " + queueName);
        }
        if (!m_externalIngressQueues.insert(queueName).second) {
            throw std::runtime_error("runtime graph ingress already exists for queue: " + queueName);
        }
        return ExternalIngress<T>(queue);
    }

    IQueue* Queue(const std::string& name);
    const IQueue* Queue(const std::string& name) const;
    std::map<std::string, QueueDiagnosticsSnapshot> QueueDiagnostics() const;
    std::map<std::string, TaskDiagnosticsSnapshot> TaskDiagnostics() const;

private:
    class TaskRunner;

    const Registry& m_registry;
    bool m_configured{false};
    bool m_running{false};
    std::unordered_map<std::string, std::unique_ptr<IQueue>> m_queues;
    std::set<std::string> m_taskProducedQueues;
    std::set<std::string> m_externalIngressQueues;
    std::vector<std::unique_ptr<TaskRunner>> m_runners;
};

} // namespace runtime_graph
} // namespace smartdrone
