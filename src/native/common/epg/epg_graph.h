#pragma once

#include "common/epg/epg_registry_types.h"

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Epg {

class EventPipelineGraph {
  public:
    template <class T>
    class ExternalIngress {
      public:
        ExternalIngress() = default;

        bool Valid() const
        {
            return m_queue != nullptr;
        }
        const std::string &QueueName() const
        {
            static const std::string EMPTY_QUEUE_NAME;
            return m_queue ? m_queue->Name() : EMPTY_QUEUE_NAME;
        }

        bool TryPush(std::shared_ptr<T> item) const
        {
            if (!m_queue) {
                throw std::runtime_error(
                    "EventPipelineGraph ingress is not bound");
            }
            return m_queue->PushErased(
                std::static_pointer_cast<void>(std::move(item)));
        }

        template <class... Args>
        bool Emplace(Args &&...args) const
        {
            return TryPush(std::make_shared<T>(std::forward<Args>(args)...));
        }

      private:
        friend class EventPipelineGraph;
        explicit ExternalIngress(IQueue *queue)
            : m_queue(queue)
        {
        }

        IQueue *m_queue{nullptr};
    };

    explicit EventPipelineGraph(Registry registry);
    ~EventPipelineGraph();

    EventPipelineGraph(const EventPipelineGraph &) = delete;
    EventPipelineGraph &operator=(const EventPipelineGraph &) = delete;

    void Configure(const GraphConfig &config);
    void ConfigureJson(const std::string &jsonText);
    void Start();
    void RequestStop();
    bool JoinStopped();
    void Stop();
    bool Running() const;

    template <class T>
    ExternalIngress<T> CreateExternalIngress(const std::string &queueName)
    {
        IQueue *queue = ValidatedExternalIngressQueue<T>(queueName);
        MarkExternalIngressQueue(queueName);
        return ExternalIngress<T>(queue);
    }

    IQueue *Queue(const std::string &name);
    const IQueue *Queue(const std::string &name) const;
    std::map<std::string, QueueDiagnosticsSnapshot> QueueDiagnostics() const;
    std::map<std::string, TaskDiagnosticsSnapshot> TaskDiagnostics() const;
    std::string DfxSnapshotJson(const std::string &graphName,
                                std::uint64_t timestampMs) const;
    std::string ProfileJson(const std::string &graphName,
                            std::uint64_t timestampMs,
                            const std::string &topologyVersion = {},
                            const std::string &taskCatalogJson = {}) const;

  private:
    class TaskRunner;
    struct ConfigureUsage;

    template <class T>
    IQueue *ValidatedExternalIngressQueue(const std::string &queueName)
    {
        IQueue *queue = FindExternalIngressQueue(queueName);
        ValidateExternalIngressQueueType(
            queue, queueName, std::type_index(typeid(T)));
        ValidateExternalIngressQueueProducer(queueName);
        return queue;
    }

    IQueue *FindExternalIngressQueue(const std::string &queueName);
    void ValidateExternalIngressQueueType(
        const IQueue *queue, const std::string &queueName,
        std::type_index expectedType) const;
    void ValidateExternalIngressQueueProducer(
        const std::string &queueName) const;
    void MarkExternalIngressQueue(const std::string &queueName);
    void ResetConfiguredGraph();
    void CreateConfiguredQueues(const GraphConfig &config,
                                ConfigureUsage &usage);
    void ValidateConfiguredTasks(const GraphConfig &config,
                                 ConfigureUsage &usage) const;
    void ValidateTaskConfig(const TaskConfig &taskConfig,
                            ConfigureUsage &usage) const;
    void ValidateTaskPorts(const TaskConfig &taskConfig,
                           const Registry::TaskTypeInfo &taskType,
                           ConfigureUsage &usage) const;
    void ValidateTaskInputs(
        const TaskConfig &taskConfig,
        const std::map<PortId, PortSpec> &declaredInputs,
        ConfigureUsage &usage) const;
    void ValidateTaskOutputs(
        const TaskConfig &taskConfig,
        const std::map<PortId, PortSpec> &declaredOutputs,
        ConfigureUsage &usage) const;
    void ValidateTaskTrigger(const TaskConfig &taskConfig) const;
    void ValidateTriggerQueues(const TaskConfig &taskConfig) const;
    void PublishTaskProducedQueues(const ConfigureUsage &usage);
    void CreateConfiguredTaskRunners(const GraphConfig &config);
    std::unordered_map<PortId, IQueue *>
    MakeInputQueueBindings(const TaskConfig &taskConfig) const;
    std::unordered_map<PortId, IQueue *>
    MakeOutputQueueBindings(const TaskConfig &taskConfig) const;
    std::vector<IQueue *>
    MakeTriggerQueueBindings(const TaskConfig &taskConfig) const;
    void BindInputNotifiers(const GraphConfig &config);
    void BindTaskInputNotifiers(TaskRunner &runner,
                                const TaskConfig &taskConfig);

    Registry m_registry;
    bool m_configured{false};
    bool m_running{false};
    GraphConfig m_config;
    std::unordered_map<std::string, std::unique_ptr<IQueue>> m_queues;
    std::set<std::string> m_taskProducedQueues;
    std::set<std::string> m_externalIngressQueues;
    std::vector<std::unique_ptr<TaskRunner>> m_runners;
};

} // namespace Epg
