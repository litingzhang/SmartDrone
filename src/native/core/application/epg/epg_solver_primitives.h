#pragma once

#include "common/epg/epg_types.h"
#include "core/application/epg/epg_solver_task_evaluation_primitives.h"
#include "core/application/epg/epg_solver_task_graph_primitives.h"
#include "core/application/epg/epg_solver_topology_primitives.h"

#include <cstdint>
#include <string>
#include <vector>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {

std::uint64_t CeilDiv(std::uint64_t numerator, std::uint64_t denominator);

std::string JsonEscape(const std::string &value);

std::string JoinReasonTags(const std::vector<std::string> &reasons);

struct DiscreteCandidateSet {
    std::vector<std::uint64_t> penalties;
    std::vector<std::uint64_t> tieWeights;
};

struct DiscreteGlobalSolution {
    std::vector<std::size_t> candidateIndexes;
    std::uint64_t totalPenalty{0};
    bool exact{false};
};

struct SolverLimits {
    std::size_t maxQueueDepth{16};
    std::uint64_t maxPeriodicIntervalMs{1000};
    std::uint64_t targetUtilizationPpm{800000};
    std::uint64_t resourceWaitPressureUs{1000};
    std::uint64_t maxExactTopologyCandidates{200000};
    std::uint64_t maxExactCpuBindingStates{200000};
    std::size_t maxCpuBindingCores{8};
};

DiscreteGlobalSolution SolveDiscreteGlobalTopology(
    const std::vector<DiscreteCandidateSet> &candidateSets,
    std::uint64_t maxExactCandidates);

SolverLimits DefaultSolverLimits();

Epg::SolverReportScore BuildSolverReportScore(
    const std::vector<Epg::SolverReportDecision> &decisions);

Epg::SolverReportDecision *FindSolverReportTaskDecision(
    std::vector<Epg::SolverReportDecision> &decisions,
    const std::string &taskName);

std::uint64_t EffectiveLoopUs(const Epg::TaskProfileMetrics &stats);

bool HasResourceWaitPressure(const Epg::TaskProfileMetrics &stats,
                             std::uint64_t pressureThresholdUs);

bool HasResourceSplitPressure(const Epg::TaskProfileMetrics &stats,
                              std::uint64_t resourceWaitThresholdUs,
                              std::uint64_t targetUtilizationPpm);

std::uint64_t QueuePressureAtDepth(
    std::uint64_t depth,
    const Epg::QueueProfileMetrics &stats);

std::uint64_t QueuePressure(const Epg::QueueConfig &queue,
                            const Epg::QueueProfileMetrics &stats);

std::uint64_t QueueCandidatePenalty(
    std::uint64_t depth,
    const Epg::QueueProfileMetrics &stats);

std::uint64_t TaskPeriodicOverloadUs(std::uint64_t intervalAfterMs,
                                     std::uint64_t effectiveLoopUs);

std::uint64_t TaskUtilizationOverPpm(std::uint64_t intervalBeforeMs,
                                     std::uint64_t intervalAfterMs,
                                     std::uint64_t utilizationPpm,
                                     std::uint64_t targetUtilizationPpm);

std::uint64_t TaskFeasibleIntervalLimit(
    std::uint64_t intervalBeforeMs,
    const Epg::TaskProfileMetrics &stats,
    std::uint64_t effectiveLoopUs,
    std::uint64_t targetUtilizationPpm,
    std::uint64_t maxPeriodicIntervalMs);

std::uint64_t PredictedResourceWaitUs(
    std::uint64_t totalResourceWaitUs,
    const std::string &resourceBefore,
    const std::string &resourceAfter);

std::uint64_t ResourceTopologyPenalty(
    const std::string &resourceBefore,
    const std::string &resourceAfter);

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
