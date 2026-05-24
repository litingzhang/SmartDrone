#include "core/application/epg/epg_cpu_binding_solver_internal.h"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {
namespace {

enum class CriticalPathVisit { Unvisited,
                               Visiting,
                               Done };

struct HeuristicCoreChoice {
    std::size_t core{0};
    std::uint64_t startMs{0};
    std::uint64_t finishMs{0};
};

std::uint64_t CriticalPathLengthAt(
    std::size_t index,
    const std::vector<TaskDependencyNode> &nodes,
    std::vector<std::uint64_t> &lengths,
    std::vector<CriticalPathVisit> &visits)
{
    if (visits[index] == CriticalPathVisit::Done) {
        return lengths[index];
    }
    if (visits[index] == CriticalPathVisit::Visiting) {
        throw std::runtime_error("cycle in CPU binding");
    }
    visits[index] = CriticalPathVisit::Visiting;
    std::uint64_t successorPath = 0;
    for (const auto successor : nodes[index].successors) {
        successorPath = std::max(
            successorPath,
            CriticalPathLengthAt(successor, nodes, lengths, visits));
    }
    lengths[index] = nodes[index].durationMs + successorPath;
    visits[index] = CriticalPathVisit::Done;
    return lengths[index];
}

std::vector<std::uint64_t> CriticalPathLengths(
    const std::vector<TaskDependencyNode> &nodes)
{
    std::vector<std::uint64_t> lengths(nodes.size(), 0);
    std::vector<CriticalPathVisit> visits(
        nodes.size(), CriticalPathVisit::Unvisited);
    for (std::size_t index = 0; index < nodes.size(); ++index) {
        CriticalPathLengthAt(index, nodes, lengths, visits);
    }
    return lengths;
}

void SortReadyByCriticalPath(std::vector<std::size_t> &ready,
                             const std::vector<TaskDependencyNode> &nodes,
                             const std::vector<std::uint64_t> &criticalPaths)
{
    std::sort(ready.begin(), ready.end(), [&](auto left, auto right) {
        if (criticalPaths[left] != criticalPaths[right]) {
            return criticalPaths[left] > criticalPaths[right];
        }
        return nodes[left].name < nodes[right].name;
    });
}

HeuristicCoreChoice BestCoreForNode(
    const TaskDependencyNode &node,
    std::uint64_t dependencyReadyMs,
    const std::vector<std::uint64_t> &coreAvailableMs)
{
    HeuristicCoreChoice choice;
    choice.finishMs = std::numeric_limits<std::uint64_t>::max();
    for (std::size_t core = 0; core < coreAvailableMs.size(); ++core) {
        const auto startMs =
            std::max(dependencyReadyMs, coreAvailableMs[core]);
        const auto finishMs = startMs + node.durationMs;
        if (finishMs < choice.finishMs) {
            choice = {core, startMs, finishMs};
        }
    }
    return choice;
}

void AppendHeuristicEntry(CpuBindingSchedule &schedule,
                          const TaskDependencyNode &node,
                          const HeuristicCoreChoice &choice)
{
    schedule.entries.push_back({node.name, node.sourceIndex,
                                static_cast<int>(choice.core),
                                choice.startMs, choice.finishMs});
    schedule.makespanMs = std::max(schedule.makespanMs, choice.finishMs);
}

} // namespace

CpuBindingSchedule BuildHeuristicCpuBindingSchedule(
    const std::vector<TaskDependencyNode> &nodes,
    std::size_t cpuCount)
{
    CpuBindingSchedule schedule;
    std::vector<bool> scheduled(nodes.size(), false);
    std::vector<std::uint64_t> coreAvailableMs(cpuCount, 0);
    const auto criticalPaths = CriticalPathLengths(nodes);
    while (schedule.entries.size() < nodes.size()) {
        auto ready = ReadyNodeIndexes(nodes, scheduled);
        if (ready.empty()) {
            throw std::runtime_error("cycle in CPU binding");
        }
        SortReadyByCriticalPath(ready, nodes, criticalPaths);
        const auto nodeIndex = ready.front();
        const auto &node = nodes[nodeIndex];
        const auto dependencyReadyMs =
            DependencyReadyMs(node, nodes, schedule.entries);
        const auto choice =
            BestCoreForNode(node, dependencyReadyMs, coreAvailableMs);
        coreAvailableMs[choice.core] = choice.finishMs;
        scheduled[nodeIndex] = true;
        AppendHeuristicEntry(schedule, node, choice);
    }
    SortCpuBindingScheduleEntries(schedule);
    return schedule;
}

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
