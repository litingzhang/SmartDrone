#include "core/application/epg/epg_solver_primitives.h"

#include <algorithm>
#include <sstream>

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

std::uint64_t CeilDiv(std::uint64_t numerator, std::uint64_t denominator)
{
    if (denominator == 0) {
        return numerator;
    }
    return (numerator + denominator - 1) / denominator;
}

std::string JsonEscape(const std::string &value)
{
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

std::uint64_t EffectiveLoopUs(const Epg::TaskProfileMetrics &stats)
{
    return std::max({stats.p99LoopUs, stats.p90LoopUs, stats.maxLoopUs,
                     stats.averageLoopUs});
}

bool HasResourceWaitPressure(const Epg::TaskProfileMetrics &stats,
                             std::uint64_t pressureThresholdUs)
{
    return stats.maxResourceWaitUs > pressureThresholdUs ||
           stats.averageResourceWaitUs > pressureThresholdUs ||
           stats.totalResourceWaitUs > pressureThresholdUs;
}

std::uint64_t QueuePressureAtDepth(
    std::uint64_t depth,
    const Epg::QueueProfileMetrics &stats)
{
    const auto depthPressure =
        stats.maxDepthObserved > depth ? stats.maxDepthObserved - depth : 0;
    return depthPressure + stats.droppedNewest + stats.overwrittenOldest;
}

std::uint64_t QueuePressure(const Epg::QueueConfig &queue,
                            const Epg::QueueProfileMetrics &stats)
{
    return QueuePressureAtDepth(static_cast<std::uint64_t>(queue.depth),
                                stats);
}

std::uint64_t QueueCandidatePenalty(
    std::uint64_t depth,
    const Epg::QueueProfileMetrics &stats)
{
    return QueuePressureAtDepth(depth, stats) * 1000 + depth;
}

std::uint64_t TaskPeriodicOverloadUs(std::uint64_t intervalAfterMs,
                                     std::uint64_t effectiveLoopUs)
{
    const std::uint64_t intervalUs = intervalAfterMs * 1000;
    if (intervalUs == 0 || effectiveLoopUs <= intervalUs) {
        return 0;
    }
    return effectiveLoopUs - intervalUs;
}

std::uint64_t TaskUtilizationOverPpm(std::uint64_t intervalBeforeMs,
                                     std::uint64_t intervalAfterMs,
                                     std::uint64_t utilizationPpm,
                                     std::uint64_t targetUtilizationPpm)
{
    if (intervalAfterMs == 0 || utilizationPpm <= targetUtilizationPpm) {
        return 0;
    }
    const auto scaledUtilization =
        CeilDiv(utilizationPpm * intervalBeforeMs, intervalAfterMs);
    if (scaledUtilization <= targetUtilizationPpm) {
        return 0;
    }
    return scaledUtilization - targetUtilizationPpm;
}

std::uint64_t TaskFeasibleIntervalLimit(
    std::uint64_t intervalBeforeMs,
    const Epg::TaskProfileMetrics &stats,
    std::uint64_t effectiveLoopUs,
    std::uint64_t targetUtilizationPpm,
    std::uint64_t maxPeriodicIntervalMs)
{
    std::uint64_t limit = intervalBeforeMs;
    if (intervalBeforeMs > 0 && effectiveLoopUs > intervalBeforeMs * 1000) {
        limit = std::max(limit, CeilDiv(effectiveLoopUs, 1000));
    }
    if (intervalBeforeMs > 0 && stats.utilizationPpm > targetUtilizationPpm) {
        limit = std::max(
            limit,
            CeilDiv(intervalBeforeMs * stats.utilizationPpm,
                    targetUtilizationPpm));
    }
    return std::min(std::max(limit, intervalBeforeMs),
                    maxPeriodicIntervalMs);
}

std::vector<Epg::PortId> SortedUniquePorts(std::vector<Epg::PortId> ports)
{
    std::sort(ports.begin(), ports.end());
    ports.erase(std::unique(ports.begin(), ports.end()), ports.end());
    return ports;
}

bool ContainsPort(const std::vector<Epg::PortId> &ports, Epg::PortId port)
{
    return std::find(ports.begin(), ports.end(), port) != ports.end();
}

const Epg::QueueConfig *FindQueueConfig(const Epg::GraphConfig &graphConfig,
                                        const std::string &name)
{
    for (const auto &queue : graphConfig.queues) {
        if (queue.name == name) {
            return &queue;
        }
    }
    return nullptr;
}

const Epg::TaskConfig *FindTaskConfig(const Epg::GraphConfig &graphConfig,
                                      const std::string &name)
{
    for (const auto &task : graphConfig.tasks) {
        if (task.name == name) {
            return &task;
        }
    }
    return nullptr;
}

const Epg::QueueProfileMetrics *FindQueueStats(
    const Epg::GraphProfileDiagnostics *diagnostics,
    const std::string &name)
{
    if (!diagnostics) {
        return nullptr;
    }
    const auto it = diagnostics->queues.find(name);
    if (it == diagnostics->queues.end()) {
        return nullptr;
    }
    return &it->second;
}

const Epg::QueueProfileMetrics *FindQueueStats(
    const Epg::GraphProfileDiagnostics *diagnostics,
    const Epg::QueueConfig &queue)
{
    return FindQueueStats(diagnostics, queue.name);
}

bool QueueHasPressure(const Epg::QueueConfig &queue,
                      const Epg::GraphProfileDiagnostics *diagnostics)
{
    const auto *stats = FindQueueStats(diagnostics, queue);
    return stats && QueuePressure(queue, *stats) > 0;
}

std::vector<Epg::PortId> CandidateBackpressurePorts(
    const Epg::GraphConfig &config,
    const Epg::GraphProfileDiagnostics *diagnostics,
    const Epg::TaskConfig &task,
    const std::vector<Epg::PortId> &before,
    bool replaceable)
{
    if (!replaceable) {
        return SortedUniquePorts(before);
    }
    auto ports = before;
    for (const auto &output : task.outputs) {
        const auto *queue = FindQueueConfig(config, output.second);
        if (!queue || !QueueHasPressure(*queue, diagnostics)) {
            continue;
        }
        ports.push_back(output.first);
    }
    return SortedUniquePorts(std::move(ports));
}

std::uint64_t BackpressureChangeCost(
    const std::vector<Epg::PortId> &before,
    const std::vector<Epg::PortId> &after,
    std::uint64_t totalResourceWaitUs)
{
    std::uint64_t penalty = after.size();
    for (const auto port : after) {
        if (!ContainsPort(before, port)) {
            penalty += 10;
        }
    }
    if (totalResourceWaitUs == 0 && after.size() > before.size()) {
        penalty += 100;
    }
    return penalty;
}

std::uint64_t BackpressureTopologyPenalty(
    const Epg::GraphConfig *sourceGraphConfig,
    const Epg::GraphProfileDiagnostics *diagnostics,
    const Epg::TaskConfig &task,
    const std::vector<Epg::PortId> &before,
    const std::vector<Epg::PortId> &after,
    std::uint64_t totalResourceWaitUs,
    bool replaceable)
{
    std::uint64_t penalty =
        BackpressureChangeCost(before, after, totalResourceWaitUs);
    if (!replaceable || !sourceGraphConfig) {
        return penalty;
    }
    for (const auto &output : task.outputs) {
        if (ContainsPort(after, output.first)) {
            continue;
        }
        const auto *queue = FindQueueConfig(*sourceGraphConfig,
                                            output.second);
        if (!queue) {
            continue;
        }
        const auto *queueStats = FindQueueStats(diagnostics, *queue);
        if (!queueStats) {
            continue;
        }
        penalty += QueuePressure(*queue, *queueStats) * 1000;
    }
    return penalty;
}

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
