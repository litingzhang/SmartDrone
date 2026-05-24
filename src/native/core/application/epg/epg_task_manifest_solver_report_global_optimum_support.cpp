#include "core/application/epg/epg_task_manifest_solver_report_global_optimum_support.h"

#include <stdexcept>
#include <string>

namespace SmartDrone::Core::Application {

const Epg::SolverReportDecision &RequireEpgSolverReportDecision(
    const Epg::SolverReport &report,
    const std::string &kind,
    const std::string &name)
{
    for (const auto &decision : report.decisions) {
        if (decision.kind == kind && decision.name == name) {
            return decision;
        }
    }
    throw std::runtime_error("solver report decision missing: " + kind + ":" +
                             name);
}

} // namespace SmartDrone::Core::Application
