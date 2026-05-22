#pragma once

#include <memory>

#include "adapters/slam/orb_feature_options.h"
#include "core/ports/visual_feature_frontend.h"
#include "core/ports/visual_place_recognition.h"

namespace ORB_SLAM3 {
class ORBextractor;
}

namespace SmartDrone::adapters::slam {

std::unique_ptr<core::ports::IVisualFeatureFrontend>
CreateOrbVisualFeatureFrontend(const OrbFeatureExtractorOptions &options = {});

std::unique_ptr<core::ports::IVisualDescriptorProvider>
CreateOrbDescriptorProvider(ORB_SLAM3::ORBextractor *extractor);

std::unique_ptr<core::ports::IVisualVocabulary> CreateOrbVisualVocabulary();

} // namespace SmartDrone::adapters::slam
