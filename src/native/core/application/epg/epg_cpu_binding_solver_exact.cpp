#include "core/application/epg/epg_cpu_binding_solver_internal.h"

#include <algorithm>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {
namespace {

struct CpuBindingSearchState {
    std::vector<bool> scheduled;
    std::vector<std::uint64_t> coreAvailableMs;
    std::vector<CpuBindingScheduleEntry> entries;
    CpuBindingSchedule best;
    std::uint64_t visitedStates{0};
};

struct CpuBindingSearchContext {
    const std::vector<TaskDependencyNode> *nodes{nullptr};
    std::uint64_t maxExactStates{0};
};

struct CpuBindingCandidate {
    std::size_t nodeIndex{0};
    std::size_t core{0};
    std::uint64_t finishMs{0};
    std::uint64_t startMs{0};
};

bool SearchStateComplete(const CpuBindingSearchContext &context,
                         const CpuBindingSearchState &state)
{
    return state.entries.size() == context.nodes->size();
}

void StoreBestCpuSchedule(const CpuBindingSearchState &state,
                          CpuBindingSchedule &best)
{
    best.entries = state.entries;
    SortCpuBindingScheduleEntries(best);
    best.makespanMs = CurrentMakespan(state.entries);
    best.exact = true;
}

void UpdateBestSchedule(const CpuBindingSearchState &state,
                        CpuBindingSchedule &best)
{
    const auto makespanMs = CurrentMakespan(state.entries);
    if (makespanMs < best.makespanMs) {
        StoreBestCpuSchedule(state, best);
    }
}

void SearchCpuBindings(const CpuBindingSearchContext &context,
                       CpuBindingSearchState &state);

void VisitCpuCandidate(const CpuBindingSearchContext &context,
                       const CpuBindingCandidate &candidate,
                       CpuBindingSearchState &state)
{
    const auto &node = context.nodes->at(candidate.nodeIndex);
    if (candidate.finishMs >= state.best.makespanMs) {
        return;
    }
    const auto previousAvailable = state.coreAvailableMs[candidate.core];
    state.coreAvailableMs[candidate.core] = candidate.finishMs;
    state.scheduled[candidate.nodeIndex] = true;
    state.entries.push_back({node.name, node.sourceIndex,
                             static_cast<int>(candidate.core),
                             candidate.startMs, candidate.finishMs});
    SearchCpuBindings(context, state);
    state.entries.pop_back();
    state.scheduled[candidate.nodeIndex] = false;
    state.coreAvailableMs[candidate.core] = previousAvailable;
}

void SearchReadyNode(const CpuBindingSearchContext &context,
                     std::size_t nodeIndex,
                     CpuBindingSearchState &state)
{
    const auto &node = context.nodes->at(nodeIndex);
    const auto dependencyReadyMs =
        DependencyReadyMs(node, *context.nodes, state.entries);
    for (std::size_t core = 0; core < state.coreAvailableMs.size(); ++core) {
        const auto startMs =
            std::max(dependencyReadyMs, state.coreAvailableMs[core]);
        VisitCpuCandidate(
            context, {nodeIndex, core, startMs + node.durationMs, startMs},
            state);
    }
}

void SearchCpuBindings(const CpuBindingSearchContext &context,
                       CpuBindingSearchState &state)
{
    if (++state.visitedStates > context.maxExactStates) {
        return;
    }
    if (SearchStateComplete(context, state)) {
        UpdateBestSchedule(state, state.best);
        return;
    }
    if (CurrentMakespan(state.entries) >= state.best.makespanMs) {
        return;
    }
    for (const auto nodeIndex :
         ReadyNodeIndexes(*context.nodes, state.scheduled)) {
        SearchReadyNode(context, nodeIndex, state);
    }
}

CpuBindingSearchState MakeCpuBindingSearchState(
    std::size_t nodeCount,
    std::size_t cpuCount,
    const CpuBindingSchedule &heuristic)
{
    CpuBindingSearchState state;
    state.scheduled.assign(nodeCount, false);
    state.coreAvailableMs.assign(cpuCount, 0);
    state.best = heuristic;
    return state;
}

} // namespace

CpuBindingSchedule BuildExactCpuBindingSchedule(
    const std::vector<TaskDependencyNode> &nodes,
    std::size_t cpuCount,
    std::uint64_t maxExactStates,
    const CpuBindingSchedule &heuristic)
{
    auto state = MakeCpuBindingSearchState(nodes.size(), cpuCount, heuristic);
    SearchCpuBindings({&nodes, maxExactStates}, state);
    if (state.visitedStates > maxExactStates) {
        return heuristic;
    }
    return state.best;
}

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
