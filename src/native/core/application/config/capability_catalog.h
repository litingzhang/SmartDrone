#pragma once

#include "config_registry.h"
#include "core/domain/capabilities.h"

namespace smartdrone::core::application {

class CapabilityCatalog {
public:
    static domain::RuntimeCapabilities BuildDefault();
};

}  // namespace smartdrone::core::application
