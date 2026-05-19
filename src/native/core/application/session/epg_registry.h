#pragma once

#include <functional>
#include <memory>

#include "common/epg/epg.h"

namespace smartdrone::core::application {

enum class EpgDomain {
    SystemRuntime,
    SlamSession,
    CalibSession,
};

using EpgTaskFactoryResolver =
    epg::TypeCatalog::TaskFactoryResolver;

void RegisterEpgTypes(epg::Registry &registry,
                                     EpgDomain domain,
                                     const EpgTaskFactoryResolver &resolver);

epg::GraphConfig CompileEpgConfig(
    EpgDomain domain,
    epg::Registry &registry);

} // namespace smartdrone::core::application
