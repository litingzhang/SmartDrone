#pragma once

#include "common/epg/epg_types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {

struct TaskCandidatePenaltyInput {
    std::uint64_t intervalBeforeMs{0};
    std::uint64_t intervalAfterMs{0};
    const Epg::TaskProfileMetrics *stats{nullptr};
    std::uint64_t effectiveLoopUs{0};
    std::uint64_t predictedResourceWaitUs{0};
    std::uint64_t topologyPenalty{0};
    std::uint64_t targetUtilizationPpm{0};
};

struct TaskCandidateEvaluationInput {
    const Epg::GraphConfig *sourceGraphConfig{nullptr};
    const Epg::GraphProfileDiagnostics *diagnostics{nullptr};
    const Epg::TaskConfig *task{nullptr};
    const Epg::TaskProfileMetrics *stats{nullptr};
    const std::vector<Epg::PortId> *backpressureBefore{nullptr};
    const std::vector<Epg::PortId> *backpressureAfter{nullptr};
    std::uint64_t intervalBeforeMs{0};
    std::uint64_t intervalAfterMs{0};
    std::uint64_t effectiveLoopUs{0};
    std::uint64_t targetUtilizationPpm{0};
    const std::string *resourceBefore{nullptr};
    const std::string *resourceAfter{nullptr};
    bool replaceable{false};
};

struct TaskCandidateEvaluation {
    std::uint64_t predictedResourceWaitUs{0};
    std::uint64_t topologyPenalty{0};
    std::uint64_t penalty{0};
};

struct TaskCandidateOptionSpaceInput {
    const Epg::GraphConfig *sourceGraphConfig{nullptr};
    const Epg::GraphProfileDiagnostics *diagnostics{nullptr};
    const Epg::TaskConfig *task{nullptr};
    const Epg::TaskProfileMetrics *stats{nullptr};
    const std::vector<std::string> *resourceAlternates{nullptr};
    std::uint64_t intervalBeforeMs{0};
    std::uint64_t effectiveLoopUs{0};
    std::uint64_t targetUtilizationPpm{0};
    std::uint64_t maxPeriodicIntervalMs{0};
    std::uint64_t resourceWaitPressureUs{0};
    const std::string *resourceBefore{nullptr};
    const std::vector<Epg::PortId> *backpressureBefore{nullptr};
    bool replaceable{false};
    bool preserveAccuracy{false};
};

struct TaskCandidateOptionSpace {
    std::uint64_t maxIntervalMs{0};
    std::vector<std::string> resourceCandidates;
    std::vector<Epg::PortId> backpressureOptimized;
};

struct TaskDecisionReasonInput {
    const Epg::TaskProfileMetrics *stats{nullptr};
    std::uint64_t intervalBeforeMs{0};
    std::uint64_t intervalAfterMs{0};
    std::uint64_t effectiveLoopUs{0};
    std::uint64_t targetUtilizationPpm{0};
    std::uint64_t budgetUs{0};
    std::uint64_t deadlineUs{0};
    std::uint64_t resourceWaitPressureUs{0};
    const std::string *resourceBefore{nullptr};
    const std::string *resourceAfter{nullptr};
    const std::vector<Epg::PortId> *backpressureBefore{nullptr};
    const std::vector<Epg::PortId> *backpressureAfter{nullptr};
    bool replaceable{false};
    int cpuAffinityBefore{-1};
    int cpuAffinityAfter{-1};
    int cpuBindingAffinity{-1};
};

Epg::TaskProfileMetrics TaskMetricsFromDecision(
    const Epg::SolverReportDecision &decision);

std::uint64_t TaskCandidatePenalty(
    const TaskCandidatePenaltyInput &input);

TaskCandidateEvaluation EvaluateTaskCandidate(
    const TaskCandidateEvaluationInput &input);

TaskCandidateOptionSpace BuildTaskCandidateOptionSpace(
    const TaskCandidateOptionSpaceInput &input);

std::string BuildTaskDecisionReason(
    const TaskDecisionReasonInput &input);

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
