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

struct EpgTaskManifest {
    EpgDomain domain{EpgDomain::SystemRuntime};
    std::string subgraphName;
    std::string topologyVersion;
    std::string dfxSnapshotPath;
    std::string profilePath;
    std::string optimizedConfigPath;
    std::vector<std::string> taskTypes;
    std::vector<EpgTaskAliasManifestEntry> aliases;
    std::vector<EpgTaskCatalogEntry> catalog;
};

const EpgTaskManifest &EpgManifestForDomain(EpgDomain domain);
std::string EpgTaskCatalogJson(const EpgTaskManifest &manifest);
void ValidateEpgTaskFactoryManifest(
    const EpgTaskManifest &manifest,
    const EpgTaskFactoryResolver &resolver);
void ValidateEpgTaskGraphManifest(
    const EpgTaskManifest &manifest,
    const epg::GraphConfig &graphConfig);

} // namespace smartdrone::core::application
