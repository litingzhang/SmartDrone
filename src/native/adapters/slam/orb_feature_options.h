#pragma once

namespace SmartDrone::Adapters::Slam {

struct OrbFeatureExtractorOptions {
    int maxFeatures{1000};
    float scaleFactor{1.2f};
    int levels{8};
    int initialFastThreshold{20};
    int minimumFastThreshold{7};
};

} // namespace SmartDrone::Adapters::Slam
