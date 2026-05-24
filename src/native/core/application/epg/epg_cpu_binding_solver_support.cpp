#include "core/application/epg/epg_cpu_binding_solver_internal.h"

#include <algorithm>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {
namespace {

bool PredecessorsScheduled(const TaskDependencyNode &node,
                           const std::vector<bool> &scheduled)
{
    for (const auto predecessor : node.predecessors) {
        if (!scheduled[predecessor]) {
            return false;
        }
    }
    return true;
}

std::uint64_t EntryFinishMs(
    const std::vector<CpuBindingScheduleEntry> &entries,
    const std::string &name)
{
    for (const auto &entry : entries) {
        if (entry.name == name) {
            return entry.finishMs;
        }
    }
    return 0;
}

} // namespace

std::vector<std::size_t> ReadyNodeIndexes(
    const std::vector<TaskDependencyNode> &nodes,
    const std::vector<bool> &scheduled)
{
    std::vector<std::size_t> ready;
    for (std::size_t index = 0; index < nodes.size(); ++index) {
        if (!scheduled[index] &&
            PredecessorsScheduled(nodes[index], scheduled)) {
            ready.push_back(index);
        }
    }
    std::sort(ready.begin(), ready.end(), [&](auto left, auto right) {
        return nodes[left].name < nodes[right].name;
    });
    return ready;
}

std::uint64_t DependencyReadyMs(
    const TaskDependencyNode &node,
    const std::vector<TaskDependencyNode> &nodes,
    const std::vector<CpuBindingScheduleEntry> &entries)
{
    std::uint64_t readyMs = 0;
    for (const auto predecessor : node.predecessors) {
        readyMs = std::max(readyMs,
                           EntryFinishMs(entries, nodes[predecessor].name));
    }
    return readyMs;
}

std::uint64_t CurrentMakespan(
    const std::vector<CpuBindingScheduleEntry> &entries)
{
    std::uint64_t makespanMs = 0;
    for (const auto &entry : entries) {
        makespanMs = std::max(makespanMs, entry.finishMs);
    }
    return makespanMs;
}

void SortCpuBindingScheduleEntries(CpuBindingSchedule &schedule)
{
    std::sort(schedule.entries.begin(), schedule.entries.end(),
              [](const auto &left, const auto &right) {
                  return left.sourceIndex < right.sourceIndex;
              });
}

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
