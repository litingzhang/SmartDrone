#include "core/application/epg/epg_registry.h"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "core/application/epg/epg_task_manifest.h"

namespace smartdrone::core::application {
namespace {

bool FileReadable(const std::string &path)
{
    std::ifstream input(path);
    return static_cast<bool>(input);
}

std::string ReadTextFile(const std::string &path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open file: " + path);
    }
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
}

epg::GraphConfig CompileStaticEpgConfig(const EpgTaskManifest &manifest,
                                        epg::Registry &registry)
{
    auto config = epg::ParseGraphConfigDotFile(
        manifest.topologyPath, manifest.subgraphName, registry);
    ApplyEpgTaskCatalogDefaults(manifest, config);
    return config;
}

epg::GraphConfig CompileOptimizedEpgConfig(const EpgTaskManifest &manifest)
{
    const auto &paths = manifest.artifactPaths;
    const std::string json = ReadTextFile(paths.optimizedConfigPath);
    const auto optimized = epg::ParseOptimizedGraphJson(json);
    ValidateEpgOptimizedGraphManifest(manifest, optimized);
    const std::string reportJson = ReadTextFile(paths.solverReportPath);
    const auto report = epg::ParseSolverReportMetadataJson(reportJson);
    ValidateEpgSolverReportManifest(manifest, optimized.metadata, report);
    return optimized.config;
}

bool TryLoadOptimizedEpgConfig(const EpgTaskManifest &manifest,
                               epg::GraphConfig &config)
{
    const auto &paths = manifest.artifactPaths;
    if (!FileReadable(paths.optimizedConfigPath)) {
        return false;
    }

    std::cerr << "[epg] loading optimized graph config: "
              << paths.optimizedConfigPath << "\n";
    config = CompileOptimizedEpgConfig(manifest);
    return true;
}

void RegisterManifestAliases(epg::Registry &registry,
                             const EpgTaskManifest &manifest,
                             const EpgTaskFactoryResolver &resolver)
{
    for (const auto &alias : manifest.aliases) {
        auto factory = resolver(alias.targetType);
        if (!factory) {
            throw std::runtime_error(
                "missing EventPipelineGraph task factory: " +
                alias.targetType);
        }
        registry.RegisterTaskFactory(alias.alias, {}, {}, factory);
    }
}

} // namespace

void RegisterEpgTypes(epg::Registry &registry,
                      EpgDomain domain,
                      const EpgTaskFactoryResolver &resolver)
{
    const EpgTaskManifest &manifest = EpgManifestForDomain(domain);
    auto &catalog = epg::TypeCatalog::Global();
    catalog.RegisterReflectedMessageTypes(registry);
    catalog.RegisterReflectedTaskTypes(
        registry, EpgTaskCatalogTypes(manifest), resolver);
    RegisterManifestAliases(registry, manifest, resolver);
}

epg::GraphConfig CompileEpgConfig(EpgDomain domain,
                                  epg::Registry &registry)
{
    const EpgTaskManifest &manifest = EpgManifestForDomain(domain);
    epg::GraphConfig config;
    try {
        if (TryLoadOptimizedEpgConfig(manifest, config)) {
            return config;
        }
    } catch (const std::exception &error) {
        std::cerr << "[epg] optimized graph config rejected: "
                  << error.what() << "; falling back to static topology\n";
    }

    config = CompileStaticEpgConfig(manifest, registry);
    ValidateEpgTaskGraphManifest(manifest, config);
    return config;
}

} // namespace smartdrone::core::application
