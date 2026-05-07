#include "common/epg/epg_internal.h"

#include <sstream>
#include <set>

namespace epg {
namespace {

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

} // namespace

EventPipelineGraph::EventPipelineGraph(const Registry& registry) : m_registry(registry) {
}

EventPipelineGraph::~EventPipelineGraph() {
    Stop();
}

void EventPipelineGraph::ConfigureJson(const std::string& jsonText) {
    Configure(ParseGraphConfigJson(jsonText));
}

void EventPipelineGraph::Configure(const GraphConfig& config) {
    if (m_running) {
        throw std::runtime_error("cannot configure a running EventPipelineGraph");
    }

    m_queues.clear();
    m_runners.clear();
    m_taskProducedQueues.clear();
    m_externalIngressQueues.clear();

    std::set<std::string> queueNames;
    std::set<std::string> taskNames;
    std::map<std::string, int> producers;
    std::map<std::string, int> consumers;

    for (const auto& queueConfig : config.queues) {
        if (queueConfig.name.empty()) {
            throw std::runtime_error("queue name must not be empty");
        }
        if (!queueNames.insert(queueConfig.name).second) {
            throw std::runtime_error("duplicate queue name: " + queueConfig.name);
        }
        if (queueConfig.depth == 0) {
            throw std::runtime_error("queue depth must be greater than zero: " + queueConfig.name);
        }
        const auto* type = m_registry.FindQueueType(queueConfig.type);
        if (!type) {
            throw std::runtime_error("unregistered queue type: " + queueConfig.type);
        }
        m_queues[queueConfig.name] = type->factory(queueConfig);
    }

    for (const auto& taskConfig : config.tasks) {
        if (taskConfig.name.empty()) {
            throw std::runtime_error("task name must not be empty");
        }
        if (!taskNames.insert(taskConfig.name).second) {
            throw std::runtime_error("duplicate task name: " + taskConfig.name);
        }
        const auto* taskType = m_registry.FindTaskType(taskConfig.type);
        if (!taskType) {
            throw std::runtime_error("unregistered task type: " + taskConfig.type);
        }

        const auto declaredInputs = MakePortMap(taskType->inputs);
        const auto declaredOutputs = MakePortMap(taskType->outputs);

        for (const auto& input : taskConfig.inputs) {
            const auto specIt = declaredInputs.find(input.first);
            if (specIt == declaredInputs.end()) {
                throw std::runtime_error("task input port is not declared: " +
                                         taskConfig.name + "." + std::to_string(input.first));
            }
            const auto qIt = m_queues.find(input.second);
            if (qIt == m_queues.end()) {
                throw std::runtime_error("task input references missing queue: " +
                                         taskConfig.name + "." + std::to_string(input.first) + " -> " + input.second);
            }
            if (qIt->second->TypeName() != specIt->second.type) {
                throw std::runtime_error("task input type mismatch: " +
                                         taskConfig.name + "." + std::to_string(input.first));
            }
            consumers[input.second] += 1;
        }

        for (const auto& output : taskConfig.outputs) {
            const auto specIt = declaredOutputs.find(output.first);
            if (specIt == declaredOutputs.end()) {
                throw std::runtime_error("task output port is not declared: " +
                                         taskConfig.name + "." + std::to_string(output.first));
            }
            const auto qIt = m_queues.find(output.second);
            if (qIt == m_queues.end()) {
                throw std::runtime_error("task output references missing queue: " +
                                         taskConfig.name + "." + std::to_string(output.first) + " -> " + output.second);
            }
            if (qIt->second->TypeName() != specIt->second.type) {
                throw std::runtime_error("task output type mismatch: " +
                                         taskConfig.name + "." + std::to_string(output.first));
            }
            producers[output.second] += 1;
        }

        if (taskConfig.trigger.mode == TriggerMode::Periodic ||
            taskConfig.trigger.mode == TriggerMode::PeriodicOrAnyQueueReady) {
            if (taskConfig.trigger.interval.count() <= 0) {
                throw std::runtime_error("periodic task interval_ms must be greater than zero: " +
                                         taskConfig.name);
            }
        }

        if (IsQueueTriggeredMode(taskConfig.trigger.mode)) {
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
        } else if (taskConfig.inputs.empty() && taskConfig.trigger.interval.count() <= 0) {
            throw std::runtime_error("task has no wake source: " + taskConfig.name);
        }
    }

    for (const auto& producer : producers) {
        if (producer.second > 1) {
            throw std::runtime_error("SPSC queue has multiple producers: " + producer.first);
        }
        m_taskProducedQueues.insert(producer.first);
    }
    for (const auto& consumer : consumers) {
        if (consumer.second > 1) {
            throw std::runtime_error("SPSC queue has multiple consumers: " + consumer.first);
        }
    }

    for (const auto& taskConfig : config.tasks) {
        const auto* taskType = m_registry.FindTaskType(taskConfig.type);
        std::unordered_map<PortId, IQueue*> inputs;
        std::unordered_map<PortId, IQueue*> outputs;
        std::vector<IQueue*> triggerQueues;

        for (const auto& input : taskConfig.inputs) {
            inputs[input.first] = m_queues.at(input.second).get();
        }
        for (const auto& output : taskConfig.outputs) {
            outputs[output.first] = m_queues.at(output.second).get();
        }
        for (const auto& triggerQueue : taskConfig.trigger.queues) {
            triggerQueues.push_back(m_queues.at(triggerQueue).get());
        }

        m_runners.emplace_back(new TaskRunner(taskConfig,
                                              taskType->factory(),
                                              std::move(inputs),
                                              std::move(outputs),
                                              std::move(triggerQueues)));
    }

    for (auto& runner : m_runners) {
        for (const auto& taskConfig : config.tasks) {
            if (taskConfig.name != runner->Name()) {
                continue;
            }
            for (const auto& input : taskConfig.inputs) {
                auto* inputQueue = m_queues.at(input.second).get();
                inputQueue->SetNotifier([runnerPtr = runner.get()]() {
                    runnerPtr->Notify();
                });
            }
        }
    }

    m_configured = true;
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
        out << "\"type\": \"" << JsonEscape(queue.second->TypeName()) << "\", ";
        out << "\"size\": " << queue.second->Size() << ", ";
        out << "\"depth\": " << queue.second->Depth() << ", ";
        out << "\"pushed\": " << diag.pushed << ", ";
        out << "\"popped\": " << diag.popped << ", ";
        out << "\"droppedNewest\": " << diag.droppedNewest << ", ";
        out << "\"overwrittenOldest\": " << diag.overwrittenOldest << ", ";
        out << "\"wakeups\": " << diag.wakeups << ", ";
        out << "\"maxDepthObserved\": " << diag.maxDepthObserved;
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
        out << "\"lastLoopUs\": " << diag.lastLoopUs << ", ";
        out << "\"maxLoopUs\": " << diag.maxLoopUs << ", ";
        out << "\"loopCount\": " << diag.loopCount << ", ";
        out << "\"errorCount\": " << diag.errorCount << ", ";
        out << "\"idleWakeups\": " << diag.idleWakeups;
        out << "}";
    }
    out << "\n  }\n";
    out << "}\n";
    return out.str();
}

} // namespace epg
