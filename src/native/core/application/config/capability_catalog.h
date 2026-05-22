#pragma once

#include "config_registry.h"
#include "core/domain/capabilities.h"

namespace SmartDrone::Core::Application {

struct CameraRuntimeProviderMetadata;

class CapabilityCatalog {
  public:
    static Domain::RuntimeCapabilities BuildDefault(
        const CameraRuntimeProviderMetadata &cameraProvider);
};

} // namespace SmartDrone::Core::Application
