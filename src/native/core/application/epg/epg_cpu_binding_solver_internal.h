#pragma once

#include "core/application/epg/epg_solver_task_graph_primitives.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {

std::vector<std::size_t> ReadyNodeIndexes(
    const std::vector<TaskDependencyNode> &nodes,
    const std::vector<bool> &scheduled);

std::uint64_t DependencyReadyMs(
    const TaskDependencyNode &node,
    const std::vector<TaskDependencyNode> &nodes,
    const std::vector<CpuBindingScheduleEntry> &entries);

std::uint64_t CurrentMakespan(
    const std::vector<CpuBindingScheduleEntry> &entries);

void SortCpuBindingScheduleEntries(CpuBindingSchedule &schedule);

CpuBindingSchedule BuildHeuristicCpuBindingSchedule(
    const std::vector<TaskDependencyNode> &nodes,
    std::size_t cpuCount);

CpuBindingSchedule BuildExactCpuBindingSchedule(
    const std::vector<TaskDependencyNode> &nodes,
    std::size_t cpuCount,
    std::uint64_t maxExactStates,
    const CpuBindingSchedule &heuristic);

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
