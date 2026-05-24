#include "core/application/epg/epg_runtime_optimizer.h"

#include "core/application/epg/epg_runtime_optimizer_io.h"
#include "core/application/epg/epg_runtime_optimizer_plan.h"
#include "core/application/epg/epg_runtime_optimizer_report.h"
#include "core/application/epg/epg_task_manifest.h"

#include <map>
#include <stdexcept>
#include <string>

namespace SmartDrone::Core::Application {
namespace {

const Epg::GraphProfileTaskCatalogEntry *FindProfileCatalogEntry(
    const Epg::GraphProfile &profile,
    const std::string &taskType)
{
    for (const auto &entry : profile.taskCatalog) {
        if (entry.taskType == taskType) {
            return &entry;
        }
    }
    return nullptr;
}

void ValidateProfileCatalogEntry(
    const EpgTaskCatalogEntry &manifestEntry,
    const Epg::GraphProfileTaskCatalogEntry &entry)
{
    if (entry.role != manifestEntry.role ||
        entry.resource != manifestEntry.resource ||
        entry.budgetUs != manifestEntry.budgetUs ||
        entry.deadlineUs != manifestEntry.deadlineUs ||
        entry.replaceable != manifestEntry.replaceable ||
        entry.resourceAlternates != manifestEntry.resourceAlternates ||
        entry.preserveAccuracy != manifestEntry.preserveAccuracy) {
        throw std::runtime_error("profile task catalog mismatch: " +
                                 manifestEntry.taskType);
    }
}

void ValidateProfileCatalog(const EpgTaskManifest &manifest,
                            const Epg::GraphProfile &profile)
{
    if (profile.taskCatalog.size() != manifest.catalog.size()) {
        throw std::runtime_error("profile task catalog size mismatch");
    }
    for (const auto &entry : manifest.catalog) {
        const auto *profileEntry =
            FindProfileCatalogEntry(profile, entry.taskType);
        if (!profileEntry) {
            throw std::runtime_error("profile task catalog missing: " +
                                     entry.taskType);
        }
        ValidateProfileCatalogEntry(entry, *profileEntry);
    }
}

void ValidateProfileDiagnosticsCoverage(
    const Epg::GraphConfig &topology,
    const Epg::GraphProfileDiagnostics &diagnostics)
{
    for (const auto &queue : topology.queues) {
        if (diagnostics.queues.find(queue.name) == diagnostics.queues.end()) {
            throw std::runtime_error("profile diagnostics missing queue: " +
                                     queue.name);
        }
    }
    for (const auto &task : topology.tasks) {
        if (diagnostics.tasks.find(task.name) == diagnostics.tasks.end()) {
            throw std::runtime_error("profile diagnostics missing task: " +
                                     task.name);
        }
    }
}

std::map<std::string, std::uint64_t>
MakeOptimizerNumbers(const Epg::GraphProfileMetadata &metadata,
                     std::uint64_t nowMs)
{
    return {
        {"generatedAtMs", nowMs},
        {"sourceTimestampMs", metadata.timestampMs},
    };
}

std::map<std::string, std::string>
MakeOptimizerStrings(const EpgTaskManifest &manifest)
{
    return {
        {"schema", Epg::OPTIMIZED_GRAPH_SCHEMA},
        {"sourceProfile", manifest.subgraphName},
        {"targetGraph", manifest.subgraphName},
        {"topologyVersion", manifest.topologyVersion},
        {"solverVersion", Epg::NATIVE_EXACT_SOLVER_VERSION},
    };
}

Epg::OptimizedGraph ValidateGeneratedArtifacts(
    const EpgTaskManifest &manifest,
    const Epg::GraphProfile &sourceProfile,
    const std::string &optimizedJson,
    const std::string &reportJson)
{
    auto optimized = Epg::ParseOptimizedGraphJson(optimizedJson);
    ValidateEpgOptimizedGraphManifest(manifest, optimized);
    const auto report = Epg::ParseSolverReportJson(reportJson);
    ValidateEpgSolverReport(manifest, sourceProfile, optimized, report);
    return optimized;
}

} // namespace

void ValidateEpgOptimizerProfileForManifest(
    const EpgTaskManifest &manifest,
    Epg::GraphProfile &profile)
{
    ValidateProfileCatalog(manifest, profile);
    ApplyEpgTaskCatalogDefaults(manifest, profile.topology);
    ValidateEpgTaskGraphManifest(manifest, profile.topology);
    ValidateProfileDiagnosticsCoverage(profile.topology, profile.diagnostics);
}

EpgRuntimeOptimizerResult
WriteOptimizedConfigForProfile(const EpgTaskManifest &manifest,
                               const Epg::GraphProfile &profile,
                               std::uint64_t nowMs)
{
    const auto &paths = manifest.artifactPaths;
    const auto plan = BuildEpgOptimizerPlan(
        manifest, profile.topology, profile.diagnostics);
    const std::string json = Epg::GraphConfigToJson(
        plan.config, MakeOptimizerStrings(manifest),
        MakeOptimizerNumbers(profile.metadata, nowMs));
    const std::string report = BuildEpgOptimizerSolverReport(
        manifest, profile.metadata, nowMs, plan.decisions);
    (void)ValidateGeneratedArtifacts(manifest, profile, json, report);
    const bool changed =
        EpgOptimizedConfigChanged(
            ReadEpgOptimizerFile(paths.optimizedConfigPath), json);
    WriteRequiredEpgOptimizerArtifactFile(paths.solverReportPath, report);
    WriteRequiredEpgOptimizerArtifactFile(paths.optimizedConfigPath, json);
    return {
        true,
        changed,
        changed ? "optimized config changed" : "optimized config refreshed",
        manifest.subgraphName,
        manifest.topologyVersion,
        manifest.subgraphName,
        paths.profilePath,
        profile.metadata.timestampMs,
        nowMs,
        Epg::NATIVE_EXACT_SOLVER_VERSION,
        paths.optimizedConfigPath,
        paths.solverReportPath,
    };
}

} // namespace SmartDrone::Core::Application
