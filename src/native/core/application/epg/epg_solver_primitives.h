#pragma once

#include "common/epg/epg_types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {

std::uint64_t CeilDiv(std::uint64_t numerator, std::uint64_t denominator);

std::string JsonEscape(const std::string &value);

struct DiscreteCandidateSet {
    std::vector<std::uint64_t> penalties;
    std::vector<std::uint64_t> tieWeights;
};

struct DiscreteGlobalSolution {
    std::vector<std::size_t> candidateIndexes;
    std::uint64_t totalPenalty{0};
    bool exact{false};
};

DiscreteGlobalSolution SolveDiscreteGlobalTopology(
    const std::vector<DiscreteCandidateSet> &candidateSets,
    std::uint64_t maxExactCandidates);

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

std::uint64_t TaskCandidatePenalty(
    std::uint64_t intervalBeforeMs,
    std::uint64_t intervalAfterMs,
    const Epg::TaskProfileMetrics &stats,
    std::uint64_t effectiveLoopUs,
    std::uint64_t predictedResourceWaitUs,
    std::uint64_t topologyPenalty,
    std::uint64_t targetUtilizationPpm);

std::vector<Epg::PortId> SortedUniquePorts(
    std::vector<Epg::PortId> ports);

bool ContainsPort(const std::vector<Epg::PortId> &ports, Epg::PortId port);

const Epg::QueueConfig *FindQueueConfig(const Epg::GraphConfig &graphConfig,
                                        const std::string &name);

const Epg::TaskConfig *FindTaskConfig(const Epg::GraphConfig &graphConfig,
                                      const std::string &name);

std::vector<Epg::PortId> CandidateBackpressurePorts(
    const Epg::GraphConfig &config,
    const Epg::GraphProfileDiagnostics *diagnostics,
    const Epg::TaskConfig &task,
    const std::vector<Epg::PortId> &before,
    bool replaceable);

std::uint64_t BackpressureTopologyPenalty(
    const Epg::GraphConfig *sourceGraphConfig,
    const Epg::GraphProfileDiagnostics *diagnostics,
    const Epg::TaskConfig &task,
    const std::vector<Epg::PortId> &before,
    const std::vector<Epg::PortId> &after,
    std::uint64_t totalResourceWaitUs,
    bool replaceable);

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
