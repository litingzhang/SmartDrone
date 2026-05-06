#pragma once

#include <functional>
#include <memory>

#include "common/epg/epg.h"

namespace smartdrone::core::application {

enum class NativeEventPipelineGraphDomain {
    SlamSession,
    CalibSession,
};

using NativeEventPipelineGraphTaskFactoryResolver =
    epg::TypeCatalog::TaskFactoryResolver;

void RegisterNativeEventPipelineGraphTypes(epg::Registry &registry,
                                     NativeEventPipelineGraphDomain domain,
                                     const NativeEventPipelineGraphTaskFactoryResolver &resolver);

epg::GraphConfig CompileNativeEventPipelineGraphConfig(
    NativeEventPipelineGraphDomain domain,
    epg::Registry &registry);

} // namespace smartdrone::core::application
