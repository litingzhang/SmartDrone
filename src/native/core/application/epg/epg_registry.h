#pragma once

#include "common/epg/epg.h"

namespace SmartDrone::Core::Application {

enum class EpgDomain {
    SystemRuntime,
    SlamSession,
    CalibSession,
};

using EpgTaskFactoryResolver =
    Epg::TypeCatalog::TaskFactoryResolver;

void RegisterEpgTypes(Epg::Registry &registry,
                      EpgDomain domain,
                      const EpgTaskFactoryResolver &resolver);

Epg::GraphConfig CompileEpgConfig(EpgDomain domain,
                                  Epg::Registry &registry);

} // namespace SmartDrone::Core::Application
