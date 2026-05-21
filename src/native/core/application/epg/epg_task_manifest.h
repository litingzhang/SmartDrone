#pragma once

#include <string>
#include <vector>

#include "core/application/epg/epg_registry.h"

namespace smartdrone::core::application {

struct EpgTaskAliasManifestEntry {
    std::string alias;
    std::string targetType;
};

struct EpgTaskCatalogEntry {
    std::string taskType;
    std::string role;
    std::string resource;
    std::uint64_t budgetUs{0};
    std::uint64_t deadlineUs{0};
    bool replaceable{false};
};

struct EpgTaskArtifactSpec {
    std::string snapshotStem;
    std::string optimizedStem;
};

struct EpgTaskArtifactPaths {
    std::string dfxSnapshotPath;
    std::string profilePath;
    std::string optimizedConfigPath;
    std::string solverReportPath;
};

struct EpgTaskTopologySpec {
    std::string path;
    std::string revision;
};

struct EpgTaskRuntimeTuningEntry {
    std::string taskName;
    bool interval{false};
    bool realtime{false};
    bool priority{false};
};

struct EpgTaskManifest {
    EpgDomain domain{EpgDomain::SystemRuntime};
    std::string subgraphName;
    std::string topologyPath;
    std::string topologyVersion;
    EpgTaskArtifactPaths artifactPaths;
    std::vector<EpgTaskAliasManifestEntry> aliases;
    std::vector<EpgTaskRuntimeTuningEntry> runtimeTuning;
    std::vector<EpgTaskCatalogEntry> catalog;
};

EpgTaskArtifactPaths BuildEpgTaskArtifactPaths(
    const EpgTaskArtifactSpec &spec);
std::string BuildEpgTaskTopologyVersion(
    const EpgTaskTopologySpec &spec);
const EpgTaskManifest &EpgManifestForDomain(EpgDomain domain);
std::vector<std::string> EpgTaskCatalogTypes(
    const EpgTaskManifest &manifest);
std::string EpgTaskCatalogJson(const EpgTaskManifest &manifest);
void ApplyEpgTaskCatalogDefaults(
    const EpgTaskManifest &manifest,
    epg::GraphConfig &graphConfig);
void ValidateEpgTaskRuntimeTuning(
    const EpgTaskManifest &manifest,
    const epg::GraphConfig &graphConfig,
    const std::vector<EpgTaskRuntimeTuningEntry> &requestedTuning);
void ValidateEpgTaskManifest(
    const EpgTaskManifest &manifest);
void ValidateEpgTaskFactoryManifest(
    const EpgTaskManifest &manifest,
    const EpgTaskFactoryResolver &resolver);
void ValidateEpgTaskGraphManifest(
    const EpgTaskManifest &manifest,
    const epg::GraphConfig &graphConfig);
void ValidateEpgOptimizedGraphManifest(
    const EpgTaskManifest &manifest,
    const epg::OptimizedGraph &optimizedGraph);
void ValidateEpgSolverReportManifest(
    const EpgTaskManifest &manifest,
    const epg::OptimizedGraphMetadata &optimizedMetadata,
    const epg::SolverReportMetadata &reportMetadata);
void ValidateEpgSolverReport(
    const EpgTaskManifest &manifest,
    const epg::OptimizedGraph &optimizedGraph,
    const epg::SolverReport &report);

} // namespace smartdrone::core::application
