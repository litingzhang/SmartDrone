#include "core/application/epg/epg_task_manifest_solver_report_metadata.h"

#include "core/application/epg/epg_task_manifest.h"

#include <stdexcept>

namespace SmartDrone::Core::Application {

void ValidateEpgSolverReportManifest(
    const EpgTaskManifest &manifest,
    const Epg::OptimizedGraphMetadata &optimizedMetadata,
    const Epg::SolverReportMetadata &reportMetadata)
{
    if (reportMetadata.schema != Epg::SOLVER_REPORT_SCHEMA) {
        throw std::runtime_error("solver report schema mismatch");
    }
    if (reportMetadata.targetGraph != manifest.subgraphName ||
        reportMetadata.targetGraph != optimizedMetadata.targetGraph) {
        throw std::runtime_error("solver report target mismatch");
    }
    if (reportMetadata.topologyVersion != manifest.topologyVersion ||
        reportMetadata.topologyVersion != optimizedMetadata.topologyVersion) {
        throw std::runtime_error("solver report topology version mismatch");
    }
    if (reportMetadata.sourceProfile != optimizedMetadata.sourceProfile ||
        reportMetadata.sourceProfile != manifest.subgraphName) {
        throw std::runtime_error("solver report source profile mismatch");
    }
    if (reportMetadata.solverVersion != optimizedMetadata.solverVersion) {
        throw std::runtime_error("solver report version mismatch");
    }
    if (reportMetadata.sourceTimestampMs !=
            optimizedMetadata.sourceTimestampMs ||
        reportMetadata.generatedAtMs != optimizedMetadata.generatedAtMs) {
        throw std::runtime_error("solver report provenance mismatch");
    }
}

void ValidateEpgSolverReportProfile(
    const EpgTaskManifest &manifest,
    const Epg::GraphProfileMetadata &profileMetadata,
    const Epg::OptimizedGraphMetadata &optimizedMetadata,
    const Epg::SolverReportMetadata &reportMetadata)
{
    if (profileMetadata.schema != Epg::GRAPH_PROFILE_SCHEMA) {
        throw std::runtime_error("solver report profile schema mismatch");
    }
    if (profileMetadata.graph != manifest.subgraphName ||
        profileMetadata.graph != optimizedMetadata.sourceProfile ||
        profileMetadata.graph != reportMetadata.sourceProfile) {
        throw std::runtime_error("solver report profile graph mismatch");
    }
    if (profileMetadata.topologyVersion != manifest.topologyVersion ||
        profileMetadata.topologyVersion != optimizedMetadata.topologyVersion ||
        profileMetadata.topologyVersion != reportMetadata.topologyVersion) {
        throw std::runtime_error("solver report profile topology mismatch");
    }
    if (profileMetadata.timestampMs == 0 ||
        profileMetadata.timestampMs != optimizedMetadata.sourceTimestampMs ||
        profileMetadata.timestampMs != reportMetadata.sourceTimestampMs) {
        throw std::runtime_error("solver report profile timestamp mismatch");
    }
}

} // namespace SmartDrone::Core::Application
