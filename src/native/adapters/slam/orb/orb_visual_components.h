#pragma once

#include <memory>

#include "adapters/slam/orb/orb_feature_options.h"
#include "core/ports/visual_feature_frontend.h"
#include "core/ports/visual_place_recognition.h"

namespace ORB_SLAM3 {
class ORBextractor;
}

namespace SmartDrone::Adapters::Slam {

std::unique_ptr<Core::Ports::IVisualFeatureFrontend>
CreateOrbVisualFeatureFrontend(const OrbFeatureExtractorOptions &options = {});

std::unique_ptr<Core::Ports::IVisualDescriptorProvider>
CreateOrbDescriptorProvider(ORB_SLAM3::ORBextractor *extractor);

std::unique_ptr<Core::Ports::IVisualVocabulary> CreateOrbVisualVocabulary();

} // namespace SmartDrone::Adapters::Slam
