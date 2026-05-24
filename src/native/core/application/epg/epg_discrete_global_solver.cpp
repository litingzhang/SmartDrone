#include "core/application/epg/epg_solver_primitives.h"

#include <utility>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {
namespace {

struct DiscreteSearchState {
    std::vector<std::size_t> indexes;
    std::vector<std::size_t> bestIndexes;
    std::uint64_t bestPenalty{0};
    std::uint64_t bestTieWeight{0};
    bool hasBest{false};
};

bool CandidateSpaceWithinLimit(
    const std::vector<DiscreteCandidateSet> &candidateSets,
    std::uint64_t maxExactCandidates)
{
    std::uint64_t total = 1;
    for (const auto &set : candidateSets) {
        if (set.penalties.empty()) {
            return false;
        }
        if (total > maxExactCandidates / set.penalties.size()) {
            return false;
        }
        total *= set.penalties.size();
    }
    return total <= maxExactCandidates;
}

std::uint64_t TieWeight(
    const std::vector<DiscreteCandidateSet> &candidateSets,
    const std::vector<std::size_t> &indexes)
{
    std::uint64_t weight = 0;
    for (std::size_t index = 0; index < candidateSets.size(); ++index) {
        const auto &weights = candidateSets[index].tieWeights;
        if (indexes[index] < weights.size()) {
            weight += weights[indexes[index]];
        }
    }
    return weight;
}

void UpdateBestCandidate(
    const std::vector<DiscreteCandidateSet> &candidateSets,
    std::uint64_t penalty,
    DiscreteSearchState &state)
{
    const auto tieWeight = TieWeight(candidateSets, state.indexes);
    if (!state.hasBest || penalty < state.bestPenalty ||
        (penalty == state.bestPenalty && tieWeight > state.bestTieWeight)) {
        state.bestPenalty = penalty;
        state.bestTieWeight = tieWeight;
        state.bestIndexes = state.indexes;
        state.hasBest = true;
    }
}

void SearchDiscreteCandidates(
    const std::vector<DiscreteCandidateSet> &candidateSets,
    std::size_t setIndex,
    std::uint64_t penalty,
    DiscreteSearchState &state)
{
    if (state.hasBest && penalty > state.bestPenalty) {
        return;
    }
    if (setIndex == candidateSets.size()) {
        UpdateBestCandidate(candidateSets, penalty, state);
        return;
    }
    const auto &set = candidateSets[setIndex];
    for (std::size_t index = 0; index < set.penalties.size(); ++index) {
        state.indexes[setIndex] = index;
        SearchDiscreteCandidates(candidateSets, setIndex + 1,
                                 penalty + set.penalties[index], state);
    }
}

} // namespace

DiscreteGlobalSolution SolveDiscreteGlobalTopology(
    const std::vector<DiscreteCandidateSet> &candidateSets,
    std::uint64_t maxExactCandidates)
{
    DiscreteGlobalSolution solution;
    solution.candidateIndexes.assign(candidateSets.size(), 0);
    if (!CandidateSpaceWithinLimit(candidateSets, maxExactCandidates)) {
        return solution;
    }

    DiscreteSearchState state;
    state.indexes.assign(candidateSets.size(), 0);
    SearchDiscreteCandidates(candidateSets, 0, 0, state);
    if (!state.hasBest) {
        return solution;
    }
    solution.candidateIndexes = std::move(state.bestIndexes);
    solution.totalPenalty = state.bestPenalty;
    solution.exact = true;
    return solution;
}

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
