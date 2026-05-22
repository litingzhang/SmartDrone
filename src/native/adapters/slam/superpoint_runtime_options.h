#pragma once

namespace SmartDrone::adapters::slam {

struct SuperPointTensorRtRuntimeOptions {
    int inputMaxWidth{640};
    int inputMaxHeight{480};
    int lightGluePoints{512};
    float lightGlueMinScore{0.02f};
    float lightGlueMaxYDiffPx{1.5f};
    float lightGlueMinDisparityPx{0.8f};
    int lightGlueEmptyDisableThreshold{3};
    int lightGlueLowYieldDisableThreshold{3};
    int lightGlueLowYieldMinPairs{8};
    int lightGlueEmptyCooldownFrames{120};
};

SuperPointTensorRtRuntimeOptions LoadSuperPointTensorRtRuntimeOptions();

} // namespace SmartDrone::adapters::slam
