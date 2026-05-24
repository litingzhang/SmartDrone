#include "core/application/epg/epg_cpu_binding_solver_internal.h"

namespace SmartDrone::Core::Application::EpgSolverPrimitives {

CpuBindingSchedule BuildCpuBindingSchedule(
    const Epg::GraphConfig &graphConfig,
    const std::map<std::string, std::uint64_t> &taskDurationsMs,
    std::size_t cpuCount,
    std::uint64_t maxExactStates)
{
    if (cpuCount == 0) {
        return {};
    }
    const auto graph = BuildTaskDependencyGraph(graphConfig, taskDurationsMs);
    const auto heuristic =
        BuildHeuristicCpuBindingSchedule(graph.nodes, cpuCount);
    return BuildExactCpuBindingSchedule(
        graph.nodes, cpuCount, maxExactStates, heuristic);
}

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
