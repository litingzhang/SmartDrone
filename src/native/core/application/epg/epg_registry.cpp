#include "core/application/epg/epg_registry.h"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/application/epg/epg_task_manifest.h"

namespace smartdrone::core::application {
namespace {

constexpr const char *kEpgTopologyPath = "config/epg/epg_topology.dot";

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

bool ContainsStringField(const std::string &text,
                         const std::string &field,
                         const std::string &value)
{
    const std::string needle =
        "\"" + field + "\": \"" + value + "\"";
    return text.find(needle) != std::string::npos;
}

epg::GraphConfig CompileStaticEpgConfig(const EpgTaskManifest &manifest,
                                        epg::Registry &registry)
{
    return epg::ParseGraphConfigDotFile(
        kEpgTopologyPath, manifest.subgraphName, registry);
}

epg::GraphConfig CompileOptimizedEpgConfig(const EpgTaskManifest &manifest)
{
    const std::string json = ReadTextFile(manifest.optimizedConfigPath);
    if (!ContainsStringField(json, "schema",
                             "smartdrone.epg.optimized_config.v1")) {
        throw std::runtime_error("optimized graph schema mismatch");
    }
    if (!ContainsStringField(json, "targetGraph", manifest.subgraphName)) {
        throw std::runtime_error("optimized graph target mismatch");
    }
    if (!ContainsStringField(json, "topologyVersion",
                             manifest.topologyVersion)) {
        throw std::runtime_error("optimized graph topology version mismatch");
    }
    return epg::ParseGraphConfigJson(json);
}

bool TryLoadOptimizedEpgConfig(const EpgTaskManifest &manifest,
                               epg::GraphConfig &config)
{
    if (!FileReadable(manifest.optimizedConfigPath)) {
        return false;
    }

    std::cerr << "[epg] loading optimized graph config: "
              << manifest.optimizedConfigPath << "\n";
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
    catalog.RegisterReflectedTaskTypes(registry, manifest.taskTypes, resolver);
    RegisterManifestAliases(registry, manifest, resolver);
}

epg::GraphConfig CompileEpgConfig(EpgDomain domain,
                                  epg::Registry &registry)
{
    const EpgTaskManifest &manifest = EpgManifestForDomain(domain);
    epg::GraphConfig config;
    try {
        if (TryLoadOptimizedEpgConfig(manifest, config)) {
            ValidateEpgTaskGraphManifest(manifest, config);
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
