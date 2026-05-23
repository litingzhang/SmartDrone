#pragma once

#include <cstdint>

namespace SmartDrone::Core::Domain {

enum class FeatureFrontend : uint8_t {
    Orb = 0,
    LK = 3,
    LkGfttPerFrame = 4,
    SuperPointLightGlue = 5,
    XFeatLightGlue = 6,
};

} // namespace SmartDrone::Core::Domain

using FeatureFrontend = SmartDrone::Core::Domain::FeatureFrontend;
