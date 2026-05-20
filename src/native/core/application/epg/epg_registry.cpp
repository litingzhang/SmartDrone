#include "core/application/epg/epg_registry.h"

#include <stdexcept>
#include <string>
#include <vector>

#include "core/application/epg/epg_task_manifest.h"

namespace smartdrone::core::application {
namespace {

constexpr const char *kEpgTopologyPath = "config/epg/epg_topology.dot";

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
    epg::GraphConfig config = epg::ParseGraphConfigDotFile(
        kEpgTopologyPath, manifest.subgraphName, registry);
    ValidateEpgTaskGraphManifest(manifest, config);
    return config;
}

} // namespace smartdrone::core::application
