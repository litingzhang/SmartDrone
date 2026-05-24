#include "core/application/epg/epg_task_manifest_validation.h"

#include <stdexcept>

namespace SmartDrone::Core::Application {

void ValidateEpgOptimizedGraphManifest(
    const EpgTaskManifest &manifest,
    const Epg::OptimizedGraph &optimizedGraph)
{
    const auto &metadata = optimizedGraph.metadata;
    if (metadata.schema != Epg::OPTIMIZED_GRAPH_SCHEMA) {
        throw std::runtime_error("optimized graph schema mismatch");
    }
    if (metadata.targetGraph != manifest.subgraphName) {
        throw std::runtime_error("optimized graph target mismatch");
    }
    if (metadata.topologyVersion != manifest.topologyVersion) {
        throw std::runtime_error("optimized graph topology version mismatch");
    }
    if (metadata.sourceProfile != manifest.subgraphName ||
        metadata.sourceTimestampMs == 0) {
        throw std::runtime_error("optimized graph source profile mismatch");
    }
    if (metadata.generatedAtMs == 0 ||
        metadata.generatedAtMs < metadata.sourceTimestampMs) {
        throw std::runtime_error("optimized graph generation timestamp invalid");
    }
    if (metadata.solverVersion.empty()) {
        throw std::runtime_error("optimized graph solver version missing");
    }
    ValidateEpgTaskGraphManifest(manifest, optimizedGraph.config);
}

} // namespace SmartDrone::Core::Application
