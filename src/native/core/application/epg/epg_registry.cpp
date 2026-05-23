#include "core/application/epg/epg_registry.h"

#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/application/epg/epg_task_manifest.h"

namespace SmartDrone::Core::Application {
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

Epg::GraphConfig CompileStaticEpgConfig(const EpgTaskManifest &manifest,
                                        Epg::Registry &registry)
{
    auto config = Epg::ParseGraphConfigDotFile(
        manifest.topologyPath, manifest.subgraphName, registry);
    ApplyEpgTaskCatalogDefaults(manifest, config);
    return config;
}

std::map<std::string, std::string> BuildQueueTypeMap(
    const Epg::GraphConfig &config)
{
    std::map<std::string, std::string> queueTypes;
    for (const auto &queue : config.queues) {
        queueTypes[queue.name] = queue.type;
    }
    return queueTypes;
}

std::vector<Epg::PortSpec> BuildGraphPortSpecs(
    const std::map<Epg::PortId, std::string> &ports,
    const std::map<std::string, std::string> &queueTypes,
    const std::string &taskName)
{
    std::vector<Epg::PortSpec> specs;
    for (const auto &port : ports) {
        const auto queueIt = queueTypes.find(port.second);
        if (queueIt == queueTypes.end()) {
            throw std::runtime_error("task references missing queue: " +
                                     taskName + "." +
                                     std::to_string(port.first));
        }
        specs.push_back({port.first, queueIt->second});
    }
    return specs;
}

void MergeGraphTaskPorts(const Epg::GraphConfig &config,
                         Epg::Registry &registry)
{
    const auto queueTypes = BuildQueueTypeMap(config);
    for (const auto &task : config.tasks) {
        registry.MergeTaskPorts(
            task.type,
            BuildGraphPortSpecs(task.inputs, queueTypes, task.name),
            BuildGraphPortSpecs(task.outputs, queueTypes, task.name));
    }
}

Epg::GraphConfig CompileOptimizedEpgConfig(const EpgTaskManifest &manifest,
                                           Epg::Registry &registry)
{
    const auto &paths = manifest.artifactPaths;
    const std::string json = ReadTextFile(paths.optimizedConfigPath);
    const auto optimized = Epg::ParseOptimizedGraphJson(json);
    ValidateEpgOptimizedGraphManifest(manifest, optimized);
    const std::string reportJson = ReadTextFile(paths.solverReportPath);
    const auto report = Epg::ParseSolverReportJson(reportJson);
    if (FileReadable(paths.profilePath)) {
        const auto profile =
            Epg::ParseGraphProfileJson(ReadTextFile(paths.profilePath));
        ValidateEpgSolverReport(manifest, profile, optimized, report);
    } else {
        ValidateEpgSolverReport(manifest, optimized, report);
    }
    MergeGraphTaskPorts(optimized.config, registry);
    return optimized.config;
}

bool TryLoadOptimizedEpgConfig(const EpgTaskManifest &manifest,
                               Epg::Registry &registry,
                               Epg::GraphConfig &config)
{
    const auto &paths = manifest.artifactPaths;
    if (!FileReadable(paths.optimizedConfigPath)) {
        return false;
    }

    std::cerr << "[epg] loading optimized graph config: "
              << paths.optimizedConfigPath << "\n";
    config = CompileOptimizedEpgConfig(manifest, registry);
    return true;
}

void RegisterManifestAliases(Epg::Registry &registry,
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

void RegisterEpgTypes(Epg::Registry &registry,
                      EpgDomain domain,
                      const EpgTaskFactoryResolver &resolver)
{
    const EpgTaskManifest &manifest = EpgManifestForDomain(domain);
    auto &catalog = Epg::TypeCatalog::Global();
    catalog.RegisterReflectedMessageTypes(registry);
    catalog.RegisterReflectedTaskTypes(
        registry, EpgTaskCatalogTypes(manifest), resolver);
    RegisterManifestAliases(registry, manifest, resolver);
}

Epg::GraphConfig CompileEpgConfig(EpgDomain domain,
                                  Epg::Registry &registry)
{
    const EpgTaskManifest &manifest = EpgManifestForDomain(domain);
    Epg::GraphConfig config;
    try {
        if (TryLoadOptimizedEpgConfig(manifest, registry, config)) {
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

} // namespace SmartDrone::Core::Application
