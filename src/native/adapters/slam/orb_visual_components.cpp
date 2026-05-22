#include "adapters/slam/orb_visual_components.h"

#include "adapters/slam/orb_feature_utils.h"
#include "adapters/slam/orb_vocabulary.h"

namespace SmartDrone::Adapters::Slam {

std::unique_ptr<Core::Ports::IVisualFeatureFrontend>
CreateOrbVisualFeatureFrontend(const OrbFeatureExtractorOptions &options)
{
    return std::make_unique<DefaultOrbFeatureFrontend>(options);
}

std::unique_ptr<Core::Ports::IVisualDescriptorProvider>
CreateOrbDescriptorProvider(ORB_SLAM3::ORBextractor *extractor)
{
    return std::make_unique<OrbDescriptorProvider>(extractor);
}

std::unique_ptr<Core::Ports::IVisualVocabulary> CreateOrbVisualVocabulary()
{
    return std::make_unique<OrbVisualVocabulary>();
}

} // namespace SmartDrone::Adapters::Slam
