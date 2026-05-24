#pragma once

#include "common/epg/epg_types.h"

namespace SmartDrone::Core::Application {

struct EpgTaskManifest;

void ValidateEpgSolverReportManifest(
    const EpgTaskManifest &manifest,
    const Epg::OptimizedGraphMetadata &optimizedMetadata,
    const Epg::SolverReportMetadata &reportMetadata);

void ValidateEpgSolverReportProfile(
    const EpgTaskManifest &manifest,
    const Epg::GraphProfileMetadata &profileMetadata,
    const Epg::OptimizedGraphMetadata &optimizedMetadata,
    const Epg::SolverReportMetadata &reportMetadata);

} // namespace SmartDrone::Core::Application
