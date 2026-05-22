#include "adapters/slam/orb_visual_components.h"

#include "adapters/slam/orb_feature_utils.h"
#include "adapters/slam/orb_vocabulary.h"

namespace SmartDrone::adapters::slam {

std::unique_ptr<core::ports::IVisualFeatureFrontend>
CreateOrbVisualFeatureFrontend(const OrbFeatureExtractorOptions &options)
{
    return std::make_unique<DefaultOrbFeatureFrontend>(options);
}

std::unique_ptr<core::ports::IVisualDescriptorProvider>
CreateOrbDescriptorProvider(ORB_SLAM3::ORBextractor *extractor)
{
    return std::make_unique<OrbDescriptorProvider>(extractor);
}

std::unique_ptr<core::ports::IVisualVocabulary> CreateOrbVisualVocabulary()
{
    return std::make_unique<OrbVisualVocabulary>();
}

} // namespace SmartDrone::adapters::slam
