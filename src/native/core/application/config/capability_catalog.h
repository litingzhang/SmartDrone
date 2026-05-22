#pragma once

#include "config_registry.h"
#include "core/domain/capabilities.h"

namespace SmartDrone::core::application {

struct CameraRuntimeProviderMetadata;

class CapabilityCatalog {
  public:
    static domain::RuntimeCapabilities BuildDefault(
        const CameraRuntimeProviderMetadata &cameraProvider);
};

} // namespace SmartDrone::core::application
