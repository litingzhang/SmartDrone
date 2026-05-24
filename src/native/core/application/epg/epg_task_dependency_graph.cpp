#include "core/application/epg/epg_solver_primitives.h"
#include "core/application/epg/epg_solver_task_graph_primitives.h"

#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {
namespace {

struct TaskTopologyState {
    const TaskDependencyNode *node{nullptr};
    std::size_t incoming{0};
    std::uint64_t level{0};
    std::uint64_t phaseOffsetMs{0};
};

using QueueProducerMap = std::map<std::string, std::size_t>;
using TaskIndexMap = std::map<std::string, std::size_t>;

std::uint64_t TaskDurationMs(
    const std::string &taskName,
    const std::map<std::string, std::uint64_t> &durationsMs)
{
    const auto it = durationsMs.find(taskName);
    if (it == durationsMs.end()) {
        return 0;
    }
    return it->second;
}

TaskIndexMap BuildTaskIndexMap(const Epg::GraphConfig &graphConfig)
{
    TaskIndexMap indexes;
    for (std::size_t index = 0; index < graphConfig.tasks.size(); ++index) {
        const auto &task = graphConfig.tasks[index];
        if (!indexes.emplace(task.name, index).second) {
            throw std::runtime_error(
                "duplicate task in dependency graph: " + task.name);
        }
    }
    return indexes;
}

QueueProducerMap BuildQueueProducerMap(const Epg::GraphConfig &graphConfig,
                                       const TaskIndexMap &taskIndexes)
{
    QueueProducerMap producers;
    for (const auto &task : graphConfig.tasks) {
        const auto taskIndex = taskIndexes.at(task.name);
        for (const auto &output : task.outputs) {
            const auto inserted = producers.emplace(output.second, taskIndex);
            if (!inserted.second) {
                throw std::runtime_error(
                    "multiple producers in dependency graph: " +
                    output.second);
            }
        }
    }
    return producers;
}

std::vector<TaskDependencyNode> BuildTaskDependencyNodes(
    const Epg::GraphConfig &graphConfig,
    const std::map<std::string, std::uint64_t> &taskDurationsMs)
{
    std::vector<TaskDependencyNode> nodes;
    nodes.reserve(graphConfig.tasks.size());
    for (std::size_t index = 0; index < graphConfig.tasks.size(); ++index) {
        const auto &task = graphConfig.tasks[index];
        nodes.push_back({task.name,
                         index,
                         TaskDurationMs(task.name, taskDurationsMs),
                         {},
                         {}});
    }
    return nodes;
}

void AddTaskDependencyEdge(std::vector<TaskDependencyNode> &nodes,
                           std::size_t producerIndex,
                           std::size_t consumerIndex)
{
    auto &successors = nodes[producerIndex].successors;
    if (std::find(successors.begin(), successors.end(), consumerIndex) !=
        successors.end()) {
        return;
    }
    successors.push_back(consumerIndex);
    nodes[consumerIndex].predecessors.push_back(producerIndex);
}

void BuildTaskDependencyEdges(
    const Epg::GraphConfig &graphConfig,
    const QueueProducerMap &queueProducers,
    std::vector<TaskDependencyNode> &nodes)
{
    for (std::size_t consumerIndex = 0;
         consumerIndex < graphConfig.tasks.size(); ++consumerIndex) {
        const auto &task = graphConfig.tasks[consumerIndex];
        for (const auto &input : task.inputs) {
            const auto producer = queueProducers.find(input.second);
            if (producer != queueProducers.end()) {
                AddTaskDependencyEdge(nodes, producer->second, consumerIndex);
            }
        }
    }
}

std::vector<TaskTopologyState> BuildTaskTopologyStates(
    const TaskDependencyGraph &graph)
{
    std::vector<TaskTopologyState> states;
    states.reserve(graph.nodes.size());
    for (const auto &node : graph.nodes) {
        states.push_back({&node, node.predecessors.size(), 0, 0});
    }
    return states;
}

std::vector<std::size_t> InitialReadyTaskIndexes(
    const std::vector<TaskTopologyState> &states)
{
    std::vector<std::size_t> ready;
    for (std::size_t index = 0; index < states.size(); ++index) {
        if (states[index].incoming == 0) {
            ready.push_back(index);
        }
    }
    return ready;
}

TaskTopologySchedule TopologicalScheduleFromNodes(
    std::vector<TaskTopologyState> states)
{
    TaskTopologySchedule schedule;
    auto ready = InitialReadyTaskIndexes(states);
    for (std::size_t cursor = 0; cursor < ready.size(); ++cursor) {
        const auto nodeIndex = ready[cursor];
        const auto &state = states[nodeIndex];
        const auto &node = *state.node;
        schedule.entries.push_back({
            node.name,
            node.sourceIndex,
            state.level,
            state.phaseOffsetMs,
            node.durationMs,
        });
        for (const auto successorIndex : node.successors) {
            auto &successor = states[successorIndex];
            successor.level = std::max(successor.level, state.level + 1);
            successor.phaseOffsetMs =
                std::max(successor.phaseOffsetMs,
                         state.phaseOffsetMs + node.durationMs);
            --successor.incoming;
            if (successor.incoming == 0) {
                ready.push_back(successorIndex);
            }
        }
    }
    if (schedule.entries.size() != states.size()) {
        throw std::runtime_error("cycle in topology schedule");
    }
    return schedule;
}

} // namespace

TaskDependencyGraph BuildTaskDependencyGraph(
    const Epg::GraphConfig &graphConfig,
    const std::map<std::string, std::uint64_t> &taskDurationsMs)
{
    const auto taskIndexes = BuildTaskIndexMap(graphConfig);
    const auto queueProducers =
        BuildQueueProducerMap(graphConfig, taskIndexes);

    TaskDependencyGraph graph;
    graph.nodes = BuildTaskDependencyNodes(graphConfig, taskDurationsMs);
    BuildTaskDependencyEdges(graphConfig, queueProducers, graph.nodes);
    return graph;
}

TaskTopologySchedule BuildTaskTopologySchedule(
    const Epg::GraphConfig &graphConfig,
    const std::map<std::string, std::uint64_t> &taskDurationsMs)
{
    const auto graph =
        BuildTaskDependencyGraph(graphConfig, taskDurationsMs);
    return TopologicalScheduleFromNodes(BuildTaskTopologyStates(graph));
}

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
