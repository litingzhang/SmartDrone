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

namespace epg {

using PortId = std::uint32_t;

inline constexpr const char *GRAPH_PROFILE_SCHEMA =
    "smartdrone.epg.profile.v1";
inline constexpr const char *OPTIMIZED_GRAPH_SCHEMA =
    "smartdrone.epg.optimized_config.v1";
inline constexpr const char *SOLVER_REPORT_SCHEMA =
    "smartdrone.epg.solver_report.v1";
inline constexpr const char *NATIVE_HEURISTIC_SOLVER_VERSION =
    "native-heuristic-v2";

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
    std::atomic<std::uint64_t> firstActivityMs{0};
    std::atomic<std::uint64_t> lastActivityMs{0};
};

struct QueueDiagnosticsSnapshot {
    std::uint64_t pushed{};
    std::uint64_t popped{};
    std::uint64_t droppedNewest{};
    std::uint64_t overwrittenOldest{};
    std::uint64_t wakeups{};
    std::uint64_t maxDepthObserved{};
    std::uint64_t firstActivityMs{};
    std::uint64_t lastActivityMs{};
};

struct TaskDiagnostics {
    std::atomic<std::uint64_t> loopCount{0};
    std::atomic<std::uint64_t> errorCount{0};
    std::atomic<std::uint64_t> idleWakeups{0};
    std::atomic<std::uint64_t> lastLoopUs{0};
    std::atomic<std::uint64_t> maxLoopUs{0};
    std::atomic<std::uint64_t> totalLoopUs{0};
    std::atomic<std::uint64_t> firstLoopMs{0};
    std::atomic<std::uint64_t> lastLoopMs{0};
    std::atomic<std::uint64_t> budgetOverrunCount{0};
    std::atomic<std::uint64_t> deadlineMissCount{0};
    std::atomic<std::uint64_t> schedulingErrorCount{0};
    std::atomic<int> lastSchedulingError{0};
};

struct TaskDiagnosticsSnapshot {
    std::uint64_t loopCount{};
    std::uint64_t errorCount{};
    std::uint64_t idleWakeups{};
    std::uint64_t lastLoopUs{};
    std::uint64_t maxLoopUs{};
    std::uint64_t p50LoopUs{};
    std::uint64_t p90LoopUs{};
    std::uint64_t p99LoopUs{};
    std::uint64_t totalLoopUs{};
    std::uint64_t firstLoopMs{};
    std::uint64_t lastLoopMs{};
    std::uint64_t budgetOverrunCount{};
    std::uint64_t deadlineMissCount{};
    std::uint64_t schedulingErrorCount{};
    int lastSchedulingError{};
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

struct TaskSchedulingConfig {
    std::string resource{"cpu"};
    int cpuAffinity{-1};
    std::uint64_t budgetUs{0};
    std::uint64_t deadlineUs{0};
    std::vector<PortId> backpressureOutputs;
    bool realtime{false};
    int priority{0};
};

struct TaskConfig {
    std::string name;
    std::string type;
    TriggerConfig trigger;
    TaskSchedulingConfig scheduling;
    std::map<PortId, std::string> inputs;
    std::map<PortId, std::string> outputs;
};

struct GraphConfig {
    std::vector<QueueConfig> queues;
    std::vector<TaskConfig> tasks;
};

struct GraphProfileMetadata {
    std::string schema;
    std::string graph;
    std::string topologyVersion;
    std::uint64_t timestampMs{};
};

struct GraphProfileTaskCatalogEntry {
    std::string taskType;
    std::string role;
    std::string resource;
    std::uint64_t budgetUs{};
    std::uint64_t deadlineUs{};
    bool replaceable{false};
};

struct OptimizedGraphMetadata {
    std::string schema;
    std::string targetGraph;
    std::string topologyVersion;
    std::string solverVersion;
    std::string sourceProfile;
    std::uint64_t sourceTimestampMs{};
    std::uint64_t generatedAtMs{};
};

struct OptimizedGraph {
    OptimizedGraphMetadata metadata;
    GraphConfig config;
};

struct SolverReportMetadata {
    std::string schema;
    std::string targetGraph;
    std::string topologyVersion;
    std::string sourceProfile;
    std::string solverVersion;
    std::uint64_t sourceTimestampMs{};
    std::uint64_t generatedAtMs{};
};

struct QueueProfileMetrics {
    std::uint64_t maxDepthObserved{};
    std::uint64_t droppedNewest{};
    std::uint64_t overwrittenOldest{};
    std::uint64_t pushedPerSecond{};
    std::uint64_t poppedPerSecond{};
    std::uint64_t droppedPerSecond{};
};

struct TaskProfileMetrics {
    std::uint64_t maxLoopUs{};
    std::uint64_t averageLoopUs{};
    std::uint64_t p90LoopUs{};
    std::uint64_t p99LoopUs{};
    std::uint64_t utilizationPpm{};
    std::uint64_t budgetOverrunCount{};
    std::uint64_t deadlineMissCount{};
    std::uint64_t schedulingErrorCount{};
};

struct GraphProfileDiagnostics {
    std::map<std::string, QueueProfileMetrics> queues;
    std::map<std::string, TaskProfileMetrics> tasks;
};

struct GraphProfile {
    GraphProfileMetadata metadata;
    std::vector<GraphProfileTaskCatalogEntry> taskCatalog;
    GraphConfig topology;
    GraphProfileDiagnostics diagnostics;
};

class Registry;

GraphProfile ParseGraphProfileJson(const std::string& jsonText);
GraphProfileMetadata ParseGraphProfileMetadataJson(
    const std::string& jsonText);
GraphProfileDiagnostics ParseGraphProfileDiagnosticsJson(
    const std::string& jsonText);
OptimizedGraphMetadata ParseOptimizedGraphMetadataJson(
    const std::string& jsonText);
OptimizedGraph ParseOptimizedGraphJson(const std::string& jsonText);
SolverReportMetadata ParseSolverReportMetadataJson(
    const std::string& jsonText);
GraphConfig ParseGraphConfigJson(const std::string& jsonText);
GraphConfig ParseGraphConfigJsonField(const std::string& jsonText,
                                      const std::string& field);
GraphConfig ParseGraphConfigJsonFile(const std::string& path);
GraphConfig ParseGraphConfigMermaid(const std::string& mermaidText);
GraphConfig ParseGraphConfigMermaidFile(const std::string& path);
GraphConfig ParseGraphConfigMermaid(const std::string& mermaidText,
                                                  Registry& registry);
GraphConfig ParseGraphConfigMermaidFile(const std::string& path,
                                                      Registry& registry);
GraphConfig ParseGraphConfigMermaidSubgraph(const std::string& mermaidText,
                                                          const std::string& subgraphName,
                                                          Registry& registry);
GraphConfig ParseGraphConfigMermaidSubgraphFile(const std::string& path,
                                                              const std::string& subgraphName,
                                                              Registry& registry);
GraphConfig ParseGraphConfigDot(const std::string& dotText,
                                              const std::string& subgraphName,
                                              Registry& registry);
GraphConfig ParseGraphConfigDotFile(const std::string& path,
                                                  const std::string& subgraphName,
                                                  Registry& registry);
std::string GraphConfigToJson(
    const GraphConfig& config,
    const std::map<std::string, std::string>& stringMetadata = {},
    const std::map<std::string, std::uint64_t>& numericMetadata = {});

struct PortSpec {
    PortId id{};
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
                StoreQueueActivity();
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
        StoreQueueActivity();
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
                StoreQueueActivity();
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
        snapshot.firstActivityMs = m_diag.firstActivityMs.load(std::memory_order_relaxed);
        snapshot.lastActivityMs = m_diag.lastActivityMs.load(std::memory_order_relaxed);
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

    static std::uint64_t SteadyNowMs() {
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
    }

    void StoreFirstQueueActivity(std::uint64_t nowMs) {
        std::uint64_t empty = 0;
        (void)m_diag.firstActivityMs.compare_exchange_strong(
            empty, nowMs, std::memory_order_relaxed, std::memory_order_relaxed);
    }

    void StoreQueueActivity() {
        const std::uint64_t nowMs = SteadyNowMs();
        StoreFirstQueueActivity(nowMs);
        m_diag.lastActivityMs.store(nowMs, std::memory_order_relaxed);
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
    TaskContext(std::unordered_map<PortId, IQueue*> inputs,
                std::unordered_map<PortId, IQueue*> outputs);

    template <class T, class... Args>
    std::shared_ptr<T> Make(Args&&... args) {
        return std::make_shared<T>(std::forward<Args>(args)...);
    }

    template <class T>
    std::shared_ptr<T> TryPop(PortId port) {
        auto* queue = InputQueue<T>(port);
        return std::static_pointer_cast<T>(queue->TryPopErased());
    }

    template <class T>
    std::shared_ptr<T> TryPopLatest(PortId port) {
        auto* queue = InputQueue<T>(port);
        return std::static_pointer_cast<T>(queue->TryPopLatestErased());
    }

    template <class T>
    bool Push(PortId port, std::shared_ptr<T> item) {
        auto* queue = OutputQueue<T>(port);
        return queue->PushErased(std::static_pointer_cast<void>(std::move(item)));
    }

    bool InputReady(PortId port) const;
    std::size_t InputSize(PortId port) const;
    bool OutputExists(PortId port) const;
    std::size_t OutputSize(PortId port) const;
    const IQueue* OutputQueueByPort(PortId port) const;

private:
    template <class T>
    IQueue* InputQueue(PortId port) {
        auto it = m_inputs.find(port);
        if (it == m_inputs.end()) {
            throw std::runtime_error("missing input port: " + std::to_string(port));
        }
        if (it->second->TypeIndex() != std::type_index(typeid(T))) {
            throw std::runtime_error("input port type mismatch: " + std::to_string(port));
        }
        return it->second;
    }

    template <class T>
    IQueue* OutputQueue(PortId port) {
        auto it = m_outputs.find(port);
        if (it == m_outputs.end()) {
            throw std::runtime_error("missing output port: " + std::to_string(port));
        }
        if (it->second->TypeIndex() != std::type_index(typeid(T))) {
            throw std::runtime_error("output port type mismatch: " + std::to_string(port));
        }
        return it->second;
    }

    std::unordered_map<PortId, IQueue*> m_inputs;
    std::unordered_map<PortId, IQueue*> m_outputs;
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
    void MergeTaskPorts(const std::string& name,
                        const std::vector<PortSpec>& inputs,
                        const std::vector<PortSpec>& outputs);

    const QueueTypeInfo* FindQueueType(const std::string& name) const;
    const TaskTypeInfo* FindTaskType(const std::string& name) const;

private:
    std::unordered_map<std::string, QueueTypeInfo> m_queueTypes;
    std::unordered_map<std::string, TaskTypeInfo> m_taskTypes;
};

class TypeCatalog {
public:
    using TaskFactoryResolver = std::function<Registry::TaskFactory(const std::string&)>;
    using TaskFactoryEntry = std::pair<std::string, Registry::TaskFactory>;

    static TypeCatalog& Global();

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
    bool RegisterTaskType(std::string name) {
        static_assert(std::is_base_of<ITask, TTask>::value,
                      "reflected task must derive from ITask");
        return RegisterTask<TTask>(std::move(name), {}, {});
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

#define EPG_PORT(portId, portType) \
    epg::PortSpec{portId, portType}

#define EPG_REGISTER_MESSAGE(messageType, messageName) \
    namespace { \
    const bool kEventPipelineGraphMessageRegistration_##messageType = \
        epg::TypeCatalog::Global().RegisterMessage<messageType>(messageName); \
    }

#define EPG_REGISTER_TASK(taskType, taskName, ...) \
    namespace { \
    const bool kEventPipelineGraphTaskRegistration_##taskType = \
        epg::TypeCatalog::Global().RegisterTask<taskType>( \
            taskName, __VA_ARGS__); \
    }

#define EPG_REGISTER_TASK_TYPE(taskType, taskName) \
    namespace { \
    const bool kEventPipelineGraphTaskRegistration_##taskType = \
        epg::TypeCatalog::Global().RegisterTaskType<taskType>(taskName); \
    }

class EventPipelineGraph {
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
                throw std::runtime_error("EventPipelineGraph ingress is not bound");
            }
            return m_queue->PushErased(std::static_pointer_cast<void>(std::move(item)));
        }

        template <class... Args>
        bool Emplace(Args&&... args) const
        {
            return TryPush(std::make_shared<T>(std::forward<Args>(args)...));
        }

    private:
        friend class EventPipelineGraph;
        explicit ExternalIngress(IQueue* queue) : m_queue(queue) {}

        IQueue* m_queue{nullptr};
    };

    explicit EventPipelineGraph(const Registry& registry);
    ~EventPipelineGraph();

    EventPipelineGraph(const EventPipelineGraph&) = delete;
    EventPipelineGraph& operator=(const EventPipelineGraph&) = delete;

    void Configure(const GraphConfig& config);
    void ConfigureJson(const std::string& jsonText);
    void Start();
    void RequestStop();
    bool JoinStopped();
    void Stop();
    bool Running() const;

    template <class T>
    ExternalIngress<T> CreateExternalIngress(const std::string& queueName)
    {
        auto it = m_queues.find(queueName);
        if (it == m_queues.end()) {
            throw std::runtime_error("EventPipelineGraph ingress queue not found: " + queueName);
        }
        IQueue* queue = it->second.get();
        if (queue->TypeIndex() != std::type_index(typeid(T))) {
            throw std::runtime_error("EventPipelineGraph ingress queue type mismatch: " + queueName);
        }
        if (m_taskProducedQueues.find(queueName) != m_taskProducedQueues.end()) {
            throw std::runtime_error("EventPipelineGraph ingress queue already has task producer: " + queueName);
        }
        if (!m_externalIngressQueues.insert(queueName).second) {
            throw std::runtime_error("EventPipelineGraph ingress already exists for queue: " + queueName);
        }
        return ExternalIngress<T>(queue);
    }

    IQueue* Queue(const std::string& name);
    const IQueue* Queue(const std::string& name) const;
    std::map<std::string, QueueDiagnosticsSnapshot> QueueDiagnostics() const;
    std::map<std::string, TaskDiagnosticsSnapshot> TaskDiagnostics() const;
    std::string DfxSnapshotJson(const std::string& graphName,
                                std::uint64_t timestampMs) const;
    std::string ProfileJson(const std::string& graphName,
                            std::uint64_t timestampMs,
                            const std::string& topologyVersion = {},
                            const std::string& taskCatalogJson = {}) const;

private:
    class TaskRunner;
    struct ConfigureUsage;

    void ResetConfiguredGraph();
    void CreateConfiguredQueues(const GraphConfig& config,
                                ConfigureUsage& usage);
    void ValidateConfiguredTasks(const GraphConfig& config,
                                  ConfigureUsage& usage) const;
    void ValidateTaskConfig(const TaskConfig& taskConfig,
                            ConfigureUsage& usage) const;
    void ValidateTaskPorts(const TaskConfig& taskConfig,
                           const Registry::TaskTypeInfo& taskType,
                           ConfigureUsage& usage) const;
    void ValidateTaskInputs(const TaskConfig& taskConfig,
                            const std::map<PortId, PortSpec>& declaredInputs,
                            ConfigureUsage& usage) const;
    void ValidateTaskOutputs(const TaskConfig& taskConfig,
                             const std::map<PortId, PortSpec>& declaredOutputs,
                             ConfigureUsage& usage) const;
    void ValidateTaskTrigger(const TaskConfig& taskConfig) const;
    void ValidateTriggerQueues(const TaskConfig& taskConfig) const;
    void PublishTaskProducedQueues(const ConfigureUsage& usage);
    void CreateConfiguredTaskRunners(const GraphConfig& config);
    std::unordered_map<PortId, IQueue*>
    MakeInputQueueBindings(const TaskConfig& taskConfig) const;
    std::unordered_map<PortId, IQueue*>
    MakeOutputQueueBindings(const TaskConfig& taskConfig) const;
    std::vector<IQueue*>
    MakeTriggerQueueBindings(const TaskConfig& taskConfig) const;
    void BindInputNotifiers(const GraphConfig& config);
    void BindTaskInputNotifiers(TaskRunner& runner,
                                const TaskConfig& taskConfig);

    const Registry& m_registry;
    bool m_configured{false};
    bool m_running{false};
    GraphConfig m_config;
    std::unordered_map<std::string, std::unique_ptr<IQueue>> m_queues;
    std::set<std::string> m_taskProducedQueues;
    std::set<std::string> m_externalIngressQueues;
    std::vector<std::unique_ptr<TaskRunner>> m_runners;
};

} // namespace epg
