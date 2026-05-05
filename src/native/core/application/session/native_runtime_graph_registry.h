#pragma once

#include <functional>
#include <memory>

#include "common/runtime_graph/runtime_graph.h"

namespace smartdrone::core::application {

enum class NativeRuntimeGraphDomain {
    SlamSession,
    CalibSession,
};

using NativeRuntimeGraphTaskFactoryResolver =
    smartdrone::runtime_graph::RuntimeGraphTypeCatalog::TaskFactoryResolver;

void RegisterNativeRuntimeGraphTypes(smartdrone::runtime_graph::Registry &registry,
                                     NativeRuntimeGraphDomain domain,
                                     const NativeRuntimeGraphTaskFactoryResolver &resolver);

smartdrone::runtime_graph::RuntimeGraphConfig CompileNativeRuntimeGraphConfig(
    NativeRuntimeGraphDomain domain,
    const smartdrone::runtime_graph::Registry &registry);

} // namespace smartdrone::core::application
