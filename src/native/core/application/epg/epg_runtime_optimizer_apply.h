#pragma once

#include "core/application/epg/epg_runtime_optimizer_plan_types.h"

namespace SmartDrone::Core::Application::EpgOptimizerPlanInternal {

void ApplyQueueSolution(Epg::QueueConfig &queue,
                        const QueueSolverNode &node,
                        const QueueCandidate &candidate,
                        std::vector<SolverDecision> &decisions);

void ApplyTaskSolution(Epg::TaskConfig &task,
                       const TaskSolverNode &node,
                       const TaskCandidate &candidate,
                       std::vector<SolverDecision> &decisions);

bool HasExecutionPlanChange(const std::vector<SolverDecision> &decisions);

} // namespace SmartDrone::Core::Application::EpgOptimizerPlanInternal
