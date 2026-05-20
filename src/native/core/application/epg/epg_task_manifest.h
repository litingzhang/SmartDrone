#pragma once

#include <string>
#include <vector>

#include "core/application/epg/epg_registry.h"

namespace smartdrone::core::application {

struct EpgTaskAliasManifestEntry {
    std::string alias;
    std::string targetType;
};

struct EpgTaskManifest {
    EpgDomain domain{EpgDomain::SystemRuntime};
    std::string subgraphName;
    std::string dfxSnapshotPath;
    std::vector<std::string> taskTypes;
    std::vector<EpgTaskAliasManifestEntry> aliases;
};

const EpgTaskManifest &EpgManifestForDomain(EpgDomain domain);
void ValidateEpgTaskFactoryManifest(
    const EpgTaskManifest &manifest,
    const EpgTaskFactoryResolver &resolver);
void ValidateEpgTaskGraphManifest(
    const EpgTaskManifest &manifest,
    const epg::GraphConfig &graphConfig);

} // namespace smartdrone::core::application
